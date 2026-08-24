#include "datacache_node.hpp"

#include <algorithm>
#include <stdexcept>

DataCacheNode::DataCacheNode() : Node("datacache_node") {
    declare_parameter<std::string>("config_path", "config.txt");
    configPath_ = get_parameter("config_path").as_string();

    loadConfiguration();
    createCoreComponents();
    reconcileStorage();
    createWatchdog();
    createUploader();
    registerEvents();
    createSubscriptions();
    createEventStatusChannel();
    createTriggerService();

    RCLCPP_INFO(get_logger(), "DataCacheNode initialized [config=%s, buffer_size=%d]",
                configPath_.c_str(), configuredBufferSize_);
}

void DataCacheNode::reconcileStorage() {
    const auto root = EventMonitor::loadRecordRoot(configManager_);
    const auto result = event_state::reconcile(root);
    if (result.recoveredFailures > 0 || result.clearedPending > 0) {
        RCLCPP_WARN(get_logger(),
                    "Storage reconciliation: recovered_failures=%zu cleared_pending=%zu",
                    result.recoveredFailures, result.clearedPending);
    }
}

void DataCacheNode::loadConfiguration() {
    configManager_ = std::make_shared<ConfigManager>(get_logger());
    if (!configManager_->loadConfig(configPath_)) {
        throw std::runtime_error("Failed to load config file: " + configPath_);
    }

    syncEnabled_ = configManager_->getBoolConfig("sync_enabled", true);
    syncRequiredForRecording_ = configManager_->getBoolConfig("sync_required_for_recording", false);
    triggerDedupeTtl_ = std::chrono::milliseconds(
        std::max(1, configManager_->getIntConfig("trigger_dedupe_ttl_ms", 60000)));
    if (syncRequiredForRecording_ && !syncEnabled_) {
        RCLCPP_WARN(get_logger(), "sync_required_for_recording=true requires sync_enabled; "
                                  "disabling strict recording mode");
        syncRequiredForRecording_ = false;
    }
}

void DataCacheNode::createCoreComponents() {
    configuredBufferSize_ = configManager_->getIntConfig("buffer_size", 1000);
    const auto bufferSize =
        configuredBufferSize_ > 0 ? static_cast<std::size_t>(configuredBufferSize_) : 0U;
    const auto bufferSeconds =
        std::max(0, configManager_->getIntConfig("buffer_duration_seconds", 30));
    constexpr std::size_t kMegabyte = 1024U * 1024U;
    const auto bufferMaxMb = std::max(0, configManager_->getIntConfig("buffer_max_mb", 1024));
    dataBuffer_ =
        std::make_shared<DataBuffer>(bufferSize, rclcpp::Duration::from_seconds(bufferSeconds),
                                     static_cast<std::size_t>(bufferMaxMb) * kMegabyte);
    pairIndex_ = std::make_shared<PairIndex>();

    const auto syncQueueSize = configManager_->getIntConfig("sync_queue_size", 100);
    const auto syncToleranceMs = std::max(0, configManager_->getIntConfig("sync_tolerance_ms", 20));
    synchronizer_ = std::make_shared<ApproximateSynchronizer>(
        static_cast<std::size_t>(std::max(1, syncQueueSize)),
        rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(syncToleranceMs) * 1000000LL),
        [this](const auto& image, const auto& cloud, const auto& difference) {
            pairIndex_->addMatched(rclcpp::Time(image->header.stamp),
                                   rclcpp::Time(cloud->header.stamp), difference);
        },
        [this](const std::string& sensor, const rclcpp::Time& timestamp, std::uint64_t count,
               const std::string& reason) {
            pairIndex_->addSingle(sensor, timestamp, reason);
            RCLCPP_WARN(get_logger(), "Approximate synchronizer dropped %s data (%llu total): %s",
                        sensor.c_str(), static_cast<unsigned long long>(count), reason.c_str());
        });

    eventMonitor_ = std::make_shared<EventMonitor>(
        dataBuffer_, configManager_, pairIndex_, get_logger(), get_clock(), this,
        [this]() { synchronizer_->flushUnmatched(); }, syncRequiredForRecording_);
    RCLCPP_INFO(get_logger(), "Recording root resolved to: %s",
                EventMonitor::loadRecordRoot(configManager_).string().c_str());
}

void DataCacheNode::createWatchdog() {
    if (!configManager_->getBoolConfig("watchdog_enabled", true)) {
        RCLCPP_INFO(get_logger(), "Sensor watchdog disabled by configuration");
        return;
    }

    watchdog_ = std::make_unique<SensorWatchdog>(
        [this](const std::string& sensor, bool stale, std::chrono::milliseconds elapsed) {
            if (stale) {
                RCLCPP_ERROR(get_logger(), "Sensor '%s' stale: no data for %.1f seconds",
                             sensor.c_str(), elapsed.count() / 1000.0);
            } else {
                RCLCPP_INFO(get_logger(), "Sensor '%s' recovered after %.1f seconds without data",
                            sensor.c_str(), elapsed.count() / 1000.0);
            }
        });
    const auto registerWatchdogSensor = [this](const char* sensor) {
        const auto fallback = configManager_->getIntConfig("watchdog_stale_timeout_ms", 1000);
        const auto timeoutMs =
            std::max(1, configManager_->getIntConfig(
                            "watchdog_" + std::string(sensor) + "_stale_timeout_ms", fallback));
        watchdog_->registerSensor(sensor, std::chrono::milliseconds(timeoutMs));
    };
    registerWatchdogSensor("camera");
    registerWatchdogSensor("lidar");

    const auto checkPeriodMs =
        std::max(1, configManager_->getIntConfig("watchdog_check_period_ms", 500));
    watchdogTimer_ = create_wall_timer(std::chrono::milliseconds(checkPeriodMs),
                                       [this]() { watchdog_->poll(); });
}

void DataCacheNode::createUploader() {
    uploadEnabled_ = configManager_->getBoolConfig("upload_enabled", false);
    if (!uploadEnabled_) {
        RCLCPP_INFO(get_logger(), "Event upload disabled by configuration");
        return;
    }

    UploadWorker::Config config;
    config.serviceName = configManager_->getConfig("upload_service_name");
    if (config.serviceName.empty()) {
        config.serviceName = "/upload_store";
    }
    config.timeoutSeconds =
        std::max(1L, static_cast<long>(configManager_->getIntConfig("upload_timeout_s", 30)));
    config.maxRetries = std::max(0, configManager_->getIntConfig("upload_max_retries", 5));
    config.scanPeriod = std::chrono::milliseconds(
        std::max(100, configManager_->getIntConfig("upload_scan_period_ms", 2000)));
    config.retryBackoff = std::chrono::milliseconds(
        std::max(1000, configManager_->getIntConfig("upload_retry_backoff_ms", 15000)));
    config.leaseTimeout = std::chrono::seconds(
        std::max(30, configManager_->getIntConfig("upload_lease_timeout_s", 300)));
    config.failedRescanPeriod = std::chrono::milliseconds(
        std::max(0, configManager_->getIntConfig("upload_failed_rescan_period_ms", 1800000)));
    uploadFailureAutoRetry_ = config.failedRescanPeriod.count() > 0;

    // 与 EventMonitor::loadRecordRoot 相同的解析规则, 保证存储与上传盯住同一目录
    const auto recordRoot = EventMonitor::loadRecordRoot(configManager_);

    uploadWorker_ = std::make_unique<UploadWorker>(recordRoot, config, get_logger(), this);
    uploadWorker_->start();
    RCLCPP_INFO(get_logger(), "Event upload enabled via RPC service: %s",
                config.serviceName.c_str());
}

void DataCacheNode::createEventStatusChannel() {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();
    eventStatusPublisher_ = create_publisher<datacache::msg::EventStatus>("/event_status", qos);
    lifecycleTimer_ =
        create_wall_timer(std::chrono::milliseconds(200), [this]() { pollEventLifecycles(); });
}

void DataCacheNode::registerEvents() {
    const auto registerRecordingEvent = [this](const std::string& eventName) {
        eventMonitor_->registerEvent(eventName, [this, eventName]() {
            RCLCPP_INFO(get_logger(), "Event '%s' triggered!", eventName.c_str());
            const auto accepted = eventMonitor_->recordDataAroundEvent(eventName);
            if (!accepted) {
                RCLCPP_ERROR(get_logger(), "Event '%s' was not accepted for storage",
                             eventName.c_str());
            }
            return accepted;
        });
    };

    auto eventNames =
        event_trigger_policy::parseEventNames(configManager_->getConfig("event_names"));
    if (eventNames.empty()) {
        RCLCPP_WARN(get_logger(), "event_names is empty; using legacy collision,hard_brake list");
        eventNames = {"collision", "hard_brake"};
    }
    for (const auto& eventName : eventNames) {
        if (configManager_->getBoolConfig("enable_" + eventName + "_event", false)) {
            registerRecordingEvent(eventName);
            RCLCPP_INFO(get_logger(), "Registered recording event: %s", eventName.c_str());
        }
    }
}

void DataCacheNode::createSubscriptions() {
    const auto sensorQos = rclcpp::SensorDataQoS();
    imageSub_ = create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", sensorQos,
        [this](const sensor_msgs::msg::Image::SharedPtr message) { handleImage(message); });
    cloudSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/point_cloud", sensorQos, [this](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
            handlePointCloud(message);
        });
}

void DataCacheNode::createTriggerService() {
    triggerService_ = create_service<datacache::srv::EventTrigger>(
        "/trigger_event",
        [this](const std::shared_ptr<datacache::srv::EventTrigger::Request> request,
               std::shared_ptr<datacache::srv::EventTrigger::Response> response) {
            handleTrigger(request, response);
        });
}

void DataCacheNode::handleImage(sensor_msgs::msg::Image::SharedPtr message) {
    if (watchdog_) {
        watchdog_->noteData("camera");
    }
    dataBuffer_->addData({SensorType::CAMERA, CameraData{message->header.stamp, message}});
    if (syncEnabled_) {
        synchronizer_->addImage(message);
    }
}

void DataCacheNode::handlePointCloud(sensor_msgs::msg::PointCloud2::SharedPtr message) {
    if (watchdog_) {
        watchdog_->noteData("lidar");
    }
    dataBuffer_->addData({SensorType::LIDAR, LidarData{message->header.stamp, message}});
    if (syncEnabled_) {
        synchronizer_->addPointCloud(message);
    }
}

void DataCacheNode::handleTrigger(
    const std::shared_ptr<datacache::srv::EventTrigger::Request>& request,
    const std::shared_ptr<datacache::srv::EventTrigger::Response>& response) {
    const auto eventName = request->event_name;
    const auto source = request->source.empty() ? "direct_rpc" : request->source;
    const auto triggerId = request->trigger_id.empty()
                               ? source + "-" + std::to_string(now().nanoseconds()) + "-" +
                                     std::to_string(nextDirectTriggerId_.fetch_add(1))
                               : request->trigger_id;
    {
        std::lock_guard<std::mutex> lock(triggerDedupeMutex_);
        const auto nowSteady = std::chrono::steady_clock::now();
        for (auto item = triggerResponses_.begin(); item != triggerResponses_.end();) {
            if (item->second.expiresAt <= nowSteady) {
                item = triggerResponses_.erase(item);
            } else {
                ++item;
            }
        }
        const auto cached = triggerResponses_.find(triggerId);
        if (cached != triggerResponses_.end()) {
            *response = cached->second.response;
            return;
        }
    }
    const auto cacheResponse = [this, &triggerId, &response]() {
        std::lock_guard<std::mutex> lock(triggerDedupeMutex_);
        triggerResponses_[triggerId] =
            CachedTriggerResponse{*response, std::chrono::steady_clock::now() + triggerDedupeTtl_};
    };
    if (eventName.empty()) {
        response->success = false;
        response->message = "Event name must not be empty";
        publishEventStatus(eventName, triggerId, source, datacache::msg::EventStatus::REJECTED,
                           response->message);
        cacheResponse();
        return;
    }

    if (!eventMonitor_->hasEvent(eventName)) {
        response->success = false;
        response->message = "Event '" + eventName + "' is not registered";
        publishEventStatus(eventName, triggerId, source, datacache::msg::EventStatus::REJECTED,
                           response->message);
        cacheResponse();
        return;
    }

    RCLCPP_INFO(get_logger(), "Event '%s' triggered by %s [%s]", eventName.c_str(), source.c_str(),
                triggerId.c_str());
    const auto capture = eventMonitor_->recordDataAroundEventDetailed(eventName);
    response->success = capture.accepted;
    response->capture_id = capture.directory.filename().string();
    response->record_directory = capture.directory.string();
    // Sensor status travels with the response so the trigger caller immediately
    // knows how complete the recorded data is.
    const auto sensorStatus = watchdog_ ? " [" + watchdog_->describeStatus() + "]" : "";
    response->message =
        capture.accepted ? "Event '" + eventName + "' accepted for storage" + sensorStatus
                         : "Event '" + eventName + "' rejected: " + capture.message + sensorStatus;
    publishEventStatus(eventName, triggerId, source,
                       capture.accepted ? datacache::msg::EventStatus::ACCEPTED
                                        : datacache::msg::EventStatus::REJECTED,
                       response->message, response->capture_id, capture.directory);
    if (capture.accepted) {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        eventLifecycles_[triggerId] = EventLifecycle{eventName,
                                                     triggerId,
                                                     source,
                                                     response->capture_id,
                                                     capture.directory,
                                                     std::chrono::steady_clock::now(),
                                                     false,
                                                     false};
    }
    cacheResponse();
}

void DataCacheNode::publishEventStatus(const std::string& eventName, const std::string& triggerId,
                                       const std::string& source, std::uint8_t status,
                                       const std::string& message, const std::string& captureId,
                                       const std::filesystem::path& directory) {
    if (!eventStatusPublisher_) {
        return;
    }
    datacache::msg::EventStatus update;
    update.status = status;
    update.event_name = eventName;
    update.trigger_id = triggerId;
    update.source = source;
    update.capture_id = captureId;
    update.record_directory = directory.string();
    update.message = message;
    update.updated_at = static_cast<builtin_interfaces::msg::Time>(now());
    eventStatusPublisher_->publish(update);
}

void DataCacheNode::pollEventLifecycles() {
    std::lock_guard<std::mutex> lock(lifecycleMutex_);
    const auto nowSteady = std::chrono::steady_clock::now();
    for (auto item = eventLifecycles_.begin(); item != eventLifecycles_.end();) {
        auto& lifecycle = item->second;
        std::error_code error;
        const bool stored = std::filesystem::exists(lifecycle.directory / ".complete", error);
        const bool recordFailed = std::filesystem::exists(lifecycle.directory / ".failed", error);
        const bool uploaded = std::filesystem::exists(lifecycle.directory / ".uploaded", error);
        const bool uploadFailed =
            std::filesystem::exists(lifecycle.directory / ".upload_failed", error);

        if (stored && !lifecycle.storedPublished) {
            publishEventStatus(lifecycle.eventName, lifecycle.triggerId, lifecycle.source,
                               datacache::msg::EventStatus::STORED, "recording complete",
                               lifecycle.captureId, lifecycle.directory);
            lifecycle.storedPublished = true;
        }
        if (recordFailed) {
            publishEventStatus(lifecycle.eventName, lifecycle.triggerId, lifecycle.source,
                               datacache::msg::EventStatus::RECORD_FAILED,
                               "recording failed during persistence", lifecycle.captureId,
                               lifecycle.directory);
            item = eventLifecycles_.erase(item);
            continue;
        }
        if (uploaded) {
            publishEventStatus(lifecycle.eventName, lifecycle.triggerId, lifecycle.source,
                               datacache::msg::EventStatus::UPLOADED, "RPC upload complete",
                               lifecycle.captureId, lifecycle.directory);
            item = eventLifecycles_.erase(item);
            continue;
        }
        if (uploadFailed && !lifecycle.uploadFailurePublished) {
            publishEventStatus(
                lifecycle.eventName, lifecycle.triggerId, lifecycle.source,
                uploadFailureAutoRetry_ ? datacache::msg::EventStatus::UPLOAD_RETRYING
                                        : datacache::msg::EventStatus::UPLOAD_FAILED,
                uploadFailureAutoRetry_ ? "RPC upload delayed; automatic retry scheduled"
                                        : "RPC upload retries exhausted",
                lifecycle.captureId, lifecycle.directory);
            lifecycle.uploadFailurePublished = true;
            if (!uploadFailureAutoRetry_) {
                item = eventLifecycles_.erase(item);
                continue;
            }
        } else if (!uploadFailed) {
            lifecycle.uploadFailurePublished = false;
        }
        if (!uploadEnabled_ && lifecycle.storedPublished) {
            item = eventLifecycles_.erase(item);
            continue;
        }
        if (!stored && nowSteady - lifecycle.acceptedAt > std::chrono::minutes(5)) {
            publishEventStatus(lifecycle.eventName, lifecycle.triggerId, lifecycle.source,
                               datacache::msg::EventStatus::RECORD_FAILED,
                               "recording did not complete before timeout", lifecycle.captureId,
                               lifecycle.directory);
            item = eventLifecycles_.erase(item);
            continue;
        }
        ++item;
    }
}
