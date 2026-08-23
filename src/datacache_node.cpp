#include "datacache_node.hpp"

#include <algorithm>
#include <stdexcept>

DataCacheNode::DataCacheNode() : Node("datacache_node") {
    declare_parameter<std::string>("config_path", "config.txt");
    configPath_ = get_parameter("config_path").as_string();

    loadConfiguration();
    createCoreComponents();
    createWatchdog();
    registerEvents();
    createSubscriptions();
    createTriggerService();

    RCLCPP_INFO(get_logger(), "DataCacheNode initialized [config=%s, buffer_size=%d]",
                configPath_.c_str(), configuredBufferSize_);
}

void DataCacheNode::loadConfiguration() {
    configManager_ = std::make_shared<ConfigManager>(get_logger());
    if (!configManager_->loadConfig(configPath_)) {
        throw std::runtime_error("Failed to load config file: " + configPath_);
    }

    syncEnabled_ = configManager_->getBoolConfig("sync_enabled", true);
    syncRequiredForRecording_ = configManager_->getBoolConfig(
        "sync_required_for_recording", false);
    if (syncRequiredForRecording_ && !syncEnabled_) {
        RCLCPP_WARN(get_logger(),
                    "sync_required_for_recording=true requires sync_enabled; "
                    "disabling strict recording mode");
        syncRequiredForRecording_ = false;
    }
}

void DataCacheNode::createCoreComponents() {
    configuredBufferSize_ = configManager_->getIntConfig("buffer_size", 1000);
    const auto bufferSize = configuredBufferSize_ > 0
        ? static_cast<std::size_t>(configuredBufferSize_) : 0U;
    const auto bufferSeconds = std::max(
        0, configManager_->getIntConfig("buffer_duration_seconds", 30));
    dataBuffer_ = std::make_shared<DataBuffer>(
        bufferSize, rclcpp::Duration::from_seconds(bufferSeconds));
    pairIndex_ = std::make_shared<PairIndex>();

    const auto syncQueueSize = configManager_->getIntConfig("sync_queue_size", 100);
    const auto syncToleranceMs = std::max(
        0, configManager_->getIntConfig("sync_tolerance_ms", 20));
    synchronizer_ = std::make_shared<ApproximateSynchronizer>(
        static_cast<std::size_t>(std::max(1, syncQueueSize)),
        rclcpp::Duration::from_nanoseconds(
            static_cast<int64_t>(syncToleranceMs) * 1000000LL),
        [this](const auto& image, const auto& cloud, const auto& difference) {
            pairIndex_->addMatched(rclcpp::Time(image->header.stamp),
                                   rclcpp::Time(cloud->header.stamp), difference);
        },
        [this](const std::string& sensor, const rclcpp::Time& timestamp,
               std::uint64_t count, const std::string& reason) {
            pairIndex_->addSingle(sensor, timestamp, reason);
            RCLCPP_WARN(get_logger(),
                        "Approximate synchronizer dropped %s data (%llu total): %s",
                        sensor.c_str(), static_cast<unsigned long long>(count),
                        reason.c_str());
        });

    eventMonitor_ = std::make_shared<EventMonitor>(
        dataBuffer_, configManager_, pairIndex_, get_logger(), get_clock(), this,
        [this]() { synchronizer_->flushUnmatched(); }, syncRequiredForRecording_);
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
        const auto timeoutMs = std::max(
            1, configManager_->getIntConfig(
                "watchdog_" + std::string(sensor) + "_stale_timeout_ms", fallback));
        watchdog_->registerSensor(sensor, std::chrono::milliseconds(timeoutMs));
    };
    registerWatchdogSensor("camera");
    registerWatchdogSensor("lidar");

    const auto checkPeriodMs = std::max(
        1, configManager_->getIntConfig("watchdog_check_period_ms", 500));
    watchdogTimer_ = create_wall_timer(
        std::chrono::milliseconds(checkPeriodMs), [this]() { watchdog_->poll(); });
}

void DataCacheNode::registerEvents() {
    const auto registerRecordingEvent = [this](const std::string& eventName) {
        eventMonitor_->registerEvent(eventName, [this, eventName]() {
            RCLCPP_INFO(get_logger(), "Event '%s' triggered!", eventName.c_str());
            const auto accepted = eventMonitor_->recordDataAroundEvent(eventName);
            if (!accepted) {
                RCLCPP_ERROR(get_logger(),
                             "Event '%s' was not accepted for storage", eventName.c_str());
            }
            return accepted;
        });
    };

    if (configManager_->getBoolConfig("enable_collision_event", false)) {
        registerRecordingEvent("collision");
    }
    if (configManager_->getBoolConfig("enable_hard_brake_event", false)) {
        registerRecordingEvent("hard_brake");
    }
}

void DataCacheNode::createSubscriptions() {
    const auto sensorQos = rclcpp::SensorDataQoS();
    imageSub_ = create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", sensorQos,
        [this](const sensor_msgs::msg::Image::SharedPtr message) {
            handleImage(message);
        });
    cloudSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/point_cloud", sensorQos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
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
    dataBuffer_->addData({SensorType::CAMERA,
                          CameraData{message->header.stamp, message}});
    if (syncEnabled_) {
        synchronizer_->addImage(message);
    }
}

void DataCacheNode::handlePointCloud(
    sensor_msgs::msg::PointCloud2::SharedPtr message) {
    if (watchdog_) {
        watchdog_->noteData("lidar");
    }
    dataBuffer_->addData({SensorType::LIDAR,
                          LidarData{message->header.stamp, message}});
    if (syncEnabled_) {
        synchronizer_->addPointCloud(message);
    }
}

void DataCacheNode::handleTrigger(
    const std::shared_ptr<datacache::srv::EventTrigger::Request>& request,
    const std::shared_ptr<datacache::srv::EventTrigger::Response>& response) {
    const auto eventName = request->event_name;
    if (eventName.empty()) {
        response->success = false;
        response->message = "Event name must not be empty";
        return;
    }

    const bool triggered = eventMonitor_->triggerEvent(eventName);
    response->success = triggered;
    // Sensor status travels with the response so the trigger caller immediately
    // knows how complete the recorded data is.
    const auto sensorStatus = watchdog_
        ? " [" + watchdog_->describeStatus() + "]" : "";
    response->message = triggered
        ? "Event '" + eventName + "' accepted for storage" + sensorStatus
        : "Event '" + eventName + "' was not accepted "
          "(unregistered, storage unavailable, or no synchronized pairs)" + sensorStatus;
}
