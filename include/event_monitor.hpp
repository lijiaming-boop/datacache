#pragma once

#include "config_manager.hpp"
#include "databuffer.hpp"
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
    EventMonitor(std::shared_ptr<DataBuffer> dataBuffer, std::shared_ptr<ConfigManager> configManager,
                 std::shared_ptr<PairIndex> pairIndex, rclcpp::Logger logger,
                 std::shared_ptr<rclcpp::Clock> clock, rclcpp::Node* node,
                 std::function<void()> flushPendingPairs)
        : dataBuffer_(std::move(dataBuffer)), configManager_(std::move(configManager)),
          pairIndex_(std::move(pairIndex)),
          flushPendingPairs_(std::move(flushPendingPairs)),
          logger_(std::move(logger)), clock_(std::move(clock)), node_(node),
          storageWorker_(std::make_unique<RawStorageWorker>(logger_,
              static_cast<std::size_t>(std::max(1, configManager_->getIntConfig(
                  "max_pending_storage_jobs", 20)))),
          maxActiveCaptures_(static_cast<std::size_t>(std::max(1, configManager_->getIntConfig(
              "max_active_event_captures", 16)))),
          schedulerTimer_(node_->create_wall_timer(
              std::chrono::milliseconds(std::max(1, configManager_->getIntConfig(
                  "event_scheduler_period_ms", 50))),
              [this]() { processExpiredCaptures(); })) {}

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
        if (flushPendingPairs_) flushPendingPairs_();
        const auto preSeconds = std::max(0, getEventIntConfig(
            eventName, "pre_time", "event_pre_time", 5));
        const auto postSeconds = std::max(0, getEventIntConfig(
            eventName, "post_time", "event_post_time", 5));
        const auto now = clock_->now();
        const auto startTime = now - rclcpp::Duration::from_seconds(preSeconds);

        RCLCPP_INFO(logger_, "Recording event '%s' [pre=%ds, post=%ds]", eventName.c_str(), preSeconds, postSeconds);
        const auto outputDirectory = configManager_->getConfig("record_directory");
        const auto eventDirectory = std::filesystem::path(outputDirectory.empty() ? "records" : outputDirectory)
            / (eventName + "_" + std::to_string(now.nanoseconds()));

        if (postSeconds <= 0) {
            return enqueueRecords(eventDirectory, eventName,
                                  dataBuffer_->getDataWithinTimeRange(startTime, now),
                                  pairIndex_->getDataWithinTimeRange(startTime, now));
        }

        EventCaptureTask task{
            eventName + "_" + std::to_string(now.nanoseconds()) + "_" +
                std::to_string(nextCaptureId_.fetch_add(1)),
            eventName, now,
            now + rclcpp::Duration::from_seconds(postSeconds), eventDirectory};
        const auto taskId = task.taskId;
        if (!storageWorker_->reserve()) {
            RCLCPP_ERROR(logger_, "Cannot accept event '%s': storage queue has no room for post-event data",
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

        const auto accepted = enqueueRecords(eventDirectory, eventName,
                                             dataBuffer_->getDataWithinTimeRange(startTime, now),
                                             pairIndex_->getDataWithinTimeRange(startTime, now));
        if (!accepted) {
            std::lock_guard<std::mutex> lock(captureMutex_);
            activeCaptures_.erase(taskId);
            storageWorker_->releaseReservation();
            RCLCPP_ERROR(logger_, "Cannot accept event '%s': pre-event storage queue is unavailable",
                         eventName.c_str());
        }
        return accepted;
    }

private:
    struct EventCaptureTask {
        std::string taskId;
        std::string eventName;
        rclcpp::Time eventTime;
        rclcpp::Time endTime;
        std::filesystem::path directory;
    };

    void processExpiredCaptures() {
        const auto now = clock_->now();
        std::vector<EventCaptureTask> expired;
        {
            std::lock_guard<std::mutex> lock(captureMutex_);
            for (auto it = activeCaptures_.begin(); it != activeCaptures_.end();) {
                if (now >= it->second.endTime) {
                    expired.push_back(std::move(it->second));
                    it = activeCaptures_.erase(it);
                } else {
                    ++it;
                }
            }
        }

        for (const auto& task : expired) {
            if (flushPendingPairs_) flushPendingPairs_();
            if (!enqueueRecords(task.directory, task.eventName,
                                dataBuffer_->getDataWithinTimeRange(task.eventTime, task.endTime),
                                pairIndex_->getDataWithinTimeRange(task.eventTime, task.endTime), true)) {
                RCLCPP_ERROR(logger_, "Post-event storage failed for '%s'",
                             task.eventName.c_str());
            }
        }
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

    bool enqueueRecords(const std::filesystem::path& directory,
                        const std::string& eventName,
                        std::vector<SensorData> records,
                        std::vector<PairRecord> pairs,
                        bool reserved = false) const {
        const bool recordCamera = getEventBoolConfig(
            eventName, "record_camera", "record_camera", false);
        const bool recordLidar = getEventBoolConfig(
            eventName, "record_lidar", "record_lidar", false);
        const bool compressionEnabled = configManager_->getBoolConfig("compression_enabled", true);
        const auto compressionLevel = std::clamp(
            configManager_->getIntConfig("compression_level", 3), 1, 19);
        const bool keepRaw = configManager_->getBoolConfig("keep_raw_after_compression", false);
        const bool conversionEnabled = configManager_->getBoolConfig("conversion_enabled", true);
        const auto imageFormat = configManager_->getConfig("image_format").empty()
            ? "jpg" : configManager_->getConfig("image_format");
        const auto imageQuality = std::clamp(
            configManager_->getIntConfig("image_quality", 90), 1, 100);
        const auto pointCloudFormat = configManager_->getConfig("pointcloud_format").empty()
            ? "pcd" : configManager_->getConfig("pointcloud_format");
        return storageWorker_->enqueue(directory, std::move(records), recordCamera, recordLidar,
                                       compressionEnabled, compressionLevel, keepRaw,
                                       conversionEnabled, imageFormat, imageQuality, pointCloudFormat,
                                       std::move(pairs), reserved);
    }

    std::shared_ptr<DataBuffer> dataBuffer_;
    std::shared_ptr<PairIndex> pairIndex_;
    std::function<void()> flushPendingPairs_;
    std::unordered_map<std::string, std::function<bool()>> eventCallbacks_;
    std::shared_ptr<ConfigManager> configManager_;
    rclcpp::Logger logger_;
    std::shared_ptr<rclcpp::Clock> clock_;
    rclcpp::Node* node_;
    std::unique_ptr<RawStorageWorker> storageWorker_;
    std::size_t maxActiveCaptures_;
    std::unordered_map<std::string, EventCaptureTask> activeCaptures_;
    mutable std::mutex captureMutex_;
    std::atomic<std::uint64_t> nextCaptureId_{0};
    rclcpp::TimerBase::SharedPtr schedulerTimer_;
};
