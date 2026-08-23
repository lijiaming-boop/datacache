#pragma once

#include "config_manager.hpp"
#include "databuffer.hpp"
#include "disk_space_manager.hpp"
#include "raw_storage_worker.hpp"
#include "pair_index.hpp"

#include <functional>
#include <chrono>
#include <algorithm>
#include <atomic>
#include <filesystem>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

class EventMonitor {
public:
    EventMonitor(std::shared_ptr<DataBuffer> dataBuffer,
                 std::shared_ptr<ConfigManager> configManager, std::shared_ptr<PairIndex> pairIndex,
                 rclcpp::Logger logger, std::shared_ptr<rclcpp::Clock> clock, rclcpp::Node* node,
                 std::function<void()> flushPendingPairs, bool requireSyncedData = false)
        : dataBuffer_(std::move(dataBuffer)), configManager_(std::move(configManager)),
          pairIndex_(std::move(pairIndex)), flushPendingPairs_(std::move(flushPendingPairs)),
          logger_(std::move(logger)), clock_(std::move(clock)), node_(node),
          diskManager_(std::make_shared<DiskSpaceManager>(loadRecordRoot(configManager_),
                                                          loadDiskPolicy(configManager_), logger_)),
          storageWorker_(std::make_unique<RawStorageWorker>(
              logger_,
              static_cast<std::size_t>(
                  std::max(1, configManager_->getIntConfig("max_pending_storage_jobs", 20))),
              diskManager_)),
          maxActiveCaptures_(static_cast<std::size_t>(
              std::max(1, configManager_->getIntConfig("max_active_event_captures", 16)))),
          sensorStallGrace_(rclcpp::Duration::from_seconds(
              std::max(0, configManager_->getIntConfig("sensor_stall_grace_ms", 5000)) / 1000.0)),
          schedulerTimer_(node_->create_wall_timer(
              std::chrono::milliseconds(
                  std::max(1, configManager_->getIntConfig("event_scheduler_period_ms", 50))),
              [this]() { processExpiredCaptures(); })) {
        requireSyncedData_ = requireSyncedData;
    }

    EventMonitor(const EventMonitor&) = delete;
    EventMonitor& operator=(const EventMonitor&) = delete;
    EventMonitor(EventMonitor&&) noexcept = default;
    EventMonitor& operator=(EventMonitor&&) noexcept = default;

    void registerEvent(const std::string& eventName, std::function<bool()> callback) {
        eventCallbacks_[eventName] = std::move(callback);
    }

    bool triggerEvent(const std::string& eventName) {
        const auto event = eventCallbacks_.find(eventName);
        if (event == eventCallbacks_.end()) {
            RCLCPP_WARN(logger_, "Event not found: %s", eventName.c_str());
            return false;
        }
        return event->second();
    }

    bool recordDataAroundEvent(const std::string& eventName) {
        if (flushPendingPairs_)
            flushPendingPairs_();
        const auto preSeconds =
            std::max(0, getEventIntConfig(eventName, "pre_time", "event_pre_time", 5));
        const auto postSeconds =
            std::max(0, getEventIntConfig(eventName, "post_time", "event_post_time", 5));
        const auto wallNow = clock_->now();
        // Window bounds must be expressed in the sensor time domain because
        // DataBuffer and PairIndex are keyed by header.stamp; node-clock bounds
        // would slice the wrong data whenever sensor and node clocks skew.
        const auto sensorNow = dataBuffer_->latestSensorTimestamp();
        if (!sensorNow.has_value()) {
            RCLCPP_WARN(logger_, "Event '%s' arrived before any sensor data; window may be empty",
                        eventName.c_str());
        }
        const auto eventTime = sensorNow.value_or(wallNow);
        const auto startTime = eventTime - rclcpp::Duration::from_seconds(preSeconds);

        RCLCPP_INFO(logger_, "Recording event '%s' [pre=%ds, post=%ds] sensor_time=%lld",
                    eventName.c_str(), preSeconds, postSeconds,
                    static_cast<long long>(eventTime.nanoseconds()));
        const auto outputDirectory = configManager_->getConfig("record_directory");
        // Directory names stay on the wall clock: two events can share the same
        // newest sensor frame, and the wall timestamp keeps names unique.
        const auto eventDirectory =
            std::filesystem::path(outputDirectory.empty() ? "records" : outputDirectory) /
            (eventName + "_" + std::to_string(wallNow.nanoseconds()));

        auto preWindowPairs = pairIndex_->getDataWithinTimeRange(startTime, eventTime);
        if (requireSyncedData_) {
            const bool hasMatchedPair =
                std::any_of(preWindowPairs.begin(), preWindowPairs.end(),
                            [](const PairRecord& record) { return record.status == "matched"; });
            if (!hasMatchedPair) {
                RCLCPP_ERROR(logger_,
                             "Rejecting event '%s': no synchronized camera/lidar pairs "
                             "in the pre-event window",
                             eventName.c_str());
                return false;
            }
        }

        if (postSeconds <= 0) {
            return enqueueRecords(eventDirectory, eventName,
                                  dataBuffer_->getDataWithinTimeRange(startTime, eventTime),
                                  std::move(preWindowPairs), false, true);
        }

        EventCaptureTask task{eventName + "_" + std::to_string(wallNow.nanoseconds()) + "_" +
                                  std::to_string(nextCaptureId_.fetch_add(1)),
                              eventName,
                              eventTime,
                              eventTime + rclcpp::Duration::from_seconds(postSeconds),
                              wallNow + rclcpp::Duration::from_seconds(postSeconds) +
                                  sensorStallGrace_,
                              eventDirectory};
        const auto taskId = task.taskId;
        if (!storageWorker_->reserve()) {
            RCLCPP_ERROR(logger_,
                         "Cannot accept event '%s': storage queue has no room for post-event data",
                         eventName.c_str());
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(captureMutex_);
            if (activeCaptures_.size() >= maxActiveCaptures_) {
                RCLCPP_ERROR(logger_,
                             "Cannot accept event '%s': maximum active event captures reached",
                             eventName.c_str());
                storageWorker_->releaseReservation();
                return false;
            }
            activeCaptures_.emplace(task.taskId, std::move(task));
        }

        const auto accepted = enqueueRecords(
            eventDirectory, eventName, dataBuffer_->getDataWithinTimeRange(startTime, eventTime),
            std::move(preWindowPairs));
        if (!accepted) {
            std::lock_guard<std::mutex> lock(captureMutex_);
            activeCaptures_.erase(taskId);
            storageWorker_->releaseReservation();
            RCLCPP_ERROR(logger_,
                         "Cannot accept event '%s': pre-event storage queue is unavailable",
                         eventName.c_str());
        }
        return accepted;
    }

    // 检查并收割到期的 post 窗口捕获任务。正常由 schedulerTimer_ 周期调用;
    // 公开为 public 供单元测试直接驱动(不依赖定时器回调被 spin 到)。
    void processExpiredCaptures() {
        const auto wallNow = clock_->now();
        const auto sensorNow = dataBuffer_->latestSensorTimestamp();
        std::vector<EventCaptureTask> expired;
        {
            std::lock_guard<std::mutex> lock(captureMutex_);
            for (auto it = activeCaptures_.begin(); it != activeCaptures_.end();) {
                // Post data completes in sensor time; the wall deadline is the
                // fallback so a stalled sensor cannot pin captures in memory.
                const bool postWindowComplete =
                    sensorNow.has_value() && *sensorNow >= it->second.endTime;
                if (postWindowComplete || wallNow >= it->second.wallDeadline) {
                    expired.push_back(std::move(it->second));
                    it = activeCaptures_.erase(it);
                } else {
                    ++it;
                }
            }
        }

        for (const auto& task : expired) {
            if (flushPendingPairs_)
                flushPendingPairs_();
            // pre 批次已含 [start, eventTime] 闭区间; post 批次排除 eventTime 边界,
            // 否则边界帧会写两次(同名文件被覆盖, manifest 出现重复行)
            auto records = dataBuffer_->getDataWithinTimeRange(task.eventTime, task.endTime);
            records.erase(std::remove_if(records.begin(), records.end(),
                                         [eventTime = task.eventTime](const SensorData& record) {
                                             return timestampOf(record) == eventTime;
                                         }),
                          records.end());
            auto pairs = pairIndex_->getDataWithinTimeRange(task.eventTime, task.endTime);
            pairs.erase(std::remove_if(pairs.begin(), pairs.end(),
                                       [eventTime = task.eventTime](const PairRecord& record) {
                                           const auto timestamp = record.hasCamera
                                                                      ? record.cameraTimestamp
                                                                      : record.lidarTimestamp;
                                           return timestamp == eventTime;
                                       }),
                        pairs.end());
            if (!enqueueRecords(task.directory, task.eventName, std::move(records),
                                std::move(pairs), true, true)) {
                RCLCPP_ERROR(logger_, "Post-event storage failed for '%s'", task.eventName.c_str());
            }
        }
    }

    // 配置里的 record_directory 解析为绝对路径; 存储与上传模块共用,
    // 保证两者盯住同一目录。public: DataCacheNode 启动日志与上传器也要用。
    static std::filesystem::path loadRecordRoot(const std::shared_ptr<ConfigManager>& config) {
        const auto directory = config->getConfig("record_directory");
        const std::filesystem::path configured(directory.empty() ? "records" : directory);
        // 解析为绝对路径: 相对路径随启动 CWD 变化, 不同启动方式会把 records
        // 散落在不同目录; 解析失败(如路径不存在导致的错误)则保留原值
        std::error_code error;
        const auto absolute = std::filesystem::absolute(configured, error);
        return error ? configured : absolute;
    }

private:
    static DiskSpaceManager::Policy loadDiskPolicy(const std::shared_ptr<ConfigManager>& config) {
        constexpr std::uintmax_t kMegabyte = 1024ULL * 1024ULL;
        DiskSpaceManager::Policy policy;
        policy.minFreeBytes = static_cast<std::uintmax_t>(
                                  std::max(0, config->getIntConfig("disk_min_free_mb", 512))) *
                              kMegabyte;
        policy.maxCapacityBytes =
            static_cast<std::uintmax_t>(
                std::max(0, config->getIntConfig("retention_max_capacity_mb", 10240))) *
            kMegabyte;
        policy.retentionDays = std::max(0, config->getIntConfig("retention_days", 30));
        policy.cleanupInterval = std::chrono::seconds(
            std::max(1, config->getIntConfig("disk_cleanup_interval_seconds", 60)));
        return policy;
    }

    struct EventCaptureTask {
        std::string taskId;
        std::string eventName;
        rclcpp::Time eventTime;    // sensor time domain (header.stamp)
        rclcpp::Time endTime;      // sensor time domain
        rclcpp::Time wallDeadline; // node clock; fires even if sensor time stalls
        std::filesystem::path directory;
    };

    static rclcpp::Time timestampOf(const SensorData& data) {
        return std::visit([](const auto& value) { return value.timestamp; }, data.data);
    }

    int getEventIntConfig(const std::string& eventName, const std::string& suffix,
                          const std::string& fallbackKey, int defaultValue) const {
        const auto eventKey = "event_" + eventName + "_" + suffix;
        return configManager_->getIntConfig(
            eventKey, configManager_->getIntConfig(fallbackKey, defaultValue));
    }

    bool getEventBoolConfig(const std::string& eventName, const std::string& suffix,
                            const std::string& fallbackKey, bool defaultValue) const {
        const auto eventKey = "event_" + eventName + "_" + suffix;
        return configManager_->getBoolConfig(
            eventKey, configManager_->getBoolConfig(fallbackKey, defaultValue));
    }

    bool enqueueRecords(const std::filesystem::path& directory, const std::string& eventName,
                        std::vector<SensorData> records, std::vector<PairRecord> pairs,
                        bool reserved = false, bool finalJob = false) const {
        const bool recordCamera =
            getEventBoolConfig(eventName, "record_camera", "record_camera", false);
        const bool recordLidar =
            getEventBoolConfig(eventName, "record_lidar", "record_lidar", false);
        const bool compressionEnabled = configManager_->getBoolConfig("compression_enabled", true);
        const auto compressionLevel =
            std::clamp(configManager_->getIntConfig("compression_level", 3), 1, 19);
        const bool keepRaw = configManager_->getBoolConfig("keep_raw_after_compression", false);
        const bool conversionEnabled = configManager_->getBoolConfig("conversion_enabled", true);
        const auto imageFormat = configManager_->getConfig("image_format").empty()
                                     ? "jpg"
                                     : configManager_->getConfig("image_format");
        const auto imageQuality =
            std::clamp(configManager_->getIntConfig("image_quality", 90), 1, 100);
        const auto pointCloudFormat = configManager_->getConfig("pointcloud_format").empty()
                                          ? "pcd"
                                          : configManager_->getConfig("pointcloud_format");
        return storageWorker_->enqueue(directory, std::move(records), recordCamera, recordLidar,
                                       compressionEnabled, compressionLevel, keepRaw,
                                       conversionEnabled, imageFormat, imageQuality,
                                       pointCloudFormat, std::move(pairs), reserved, finalJob);
    }

    std::shared_ptr<DataBuffer> dataBuffer_;
    std::shared_ptr<ConfigManager> configManager_;
    std::shared_ptr<PairIndex> pairIndex_;
    std::function<void()> flushPendingPairs_;
    std::unordered_map<std::string, std::function<bool()>> eventCallbacks_;
    rclcpp::Logger logger_;
    std::shared_ptr<rclcpp::Clock> clock_;
    rclcpp::Node* node_;
    std::shared_ptr<DiskSpaceManager> diskManager_;
    std::unique_ptr<RawStorageWorker> storageWorker_;
    std::size_t maxActiveCaptures_;
    rclcpp::Duration sensorStallGrace_{0, 0};
    bool requireSyncedData_{false};
    std::unordered_map<std::string, EventCaptureTask> activeCaptures_;
    mutable std::mutex captureMutex_;
    std::atomic<std::uint64_t> nextCaptureId_{0};
    rclcpp::TimerBase::SharedPtr schedulerTimer_;
};
