#pragma once

#include "config_manager.hpp"
#include "databuffer.hpp"
#include "raw_storage_worker.hpp"

#include <functional>
#include <chrono>
#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

class EventMonitor {
public:
    EventMonitor(std::shared_ptr<DataBuffer> dataBuffer, std::shared_ptr<ConfigManager> configManager,
                 rclcpp::Logger logger, std::shared_ptr<rclcpp::Clock> clock, rclcpp::Node* node)
        : dataBuffer_(std::move(dataBuffer)), configManager_(std::move(configManager)),
          logger_(std::move(logger)), clock_(std::move(clock)), node_(node),
          storageWorker_(std::make_unique<RawStorageWorker>(logger_,
              static_cast<std::size_t>(std::max(1, configManager_->getIntConfig(
                  "max_pending_storage_jobs", 20))))) {}

    EventMonitor(const EventMonitor&) = delete;
    EventMonitor& operator=(const EventMonitor&) = delete;
    EventMonitor(EventMonitor&&) noexcept = default;
    EventMonitor& operator=(EventMonitor&&) noexcept = default;

    void registerEvent(const std::string& eventName, std::function<void()> callback) {
        eventCallbacks_[eventName] = std::move(callback);
    }

    bool triggerEvent(const std::string& eventName) {
        const auto event = eventCallbacks_.find(eventName);
        if (event == eventCallbacks_.end()) {
            RCLCPP_WARN(logger_, "Event not found: %s", eventName.c_str());
            return false;
        }
        event->second();
        return true;
    }

    void recordDataAroundEvent(const std::string& eventName) {
        const auto preSeconds = std::max(0, configManager_->getIntConfig("event_pre_time", 5));
        const auto postSeconds = std::max(0, configManager_->getIntConfig("event_post_time", 5));
        const auto now = clock_->now();
        const auto startTime = now - rclcpp::Duration::from_seconds(preSeconds);

        RCLCPP_INFO(logger_, "Recording event '%s' [pre=%ds, post=%ds]", eventName.c_str(), preSeconds, postSeconds);
        const auto outputDirectory = configManager_->getConfig("record_directory");
        const auto eventDirectory = std::filesystem::path(outputDirectory.empty() ? "records" : outputDirectory)
            / (eventName + "_" + std::to_string(now.nanoseconds()));

        enqueueRecords(eventDirectory, dataBuffer_->getDataWithinTimeRange(startTime, now));

        if (postSeconds > 0) {
            postTimer_ = node_->create_wall_timer(std::chrono::seconds(postSeconds),
                [this, eventDirectory, now, postSeconds]() {
                    enqueueRecords(eventDirectory, dataBuffer_->getDataWithinTimeRange(
                        now, now + rclcpp::Duration::from_seconds(postSeconds)));
                    postTimer_->cancel();
                });
        }
    }

private:
    void enqueueRecords(const std::filesystem::path& directory,
                        std::vector<SensorData> records) const {
        const bool recordCamera = configManager_->getBoolConfig("record_camera", false);
        const bool recordLidar = configManager_->getBoolConfig("record_lidar", false);
        const bool compressionEnabled = configManager_->getBoolConfig("compression_enabled", true);
        const auto compressionLevel = std::clamp(
            configManager_->getIntConfig("compression_level", 3), 1, 19);
        const bool keepRaw = configManager_->getBoolConfig("keep_raw_after_compression", false);
        storageWorker_->enqueue(directory, std::move(records), recordCamera, recordLidar,
                                 compressionEnabled, compressionLevel, keepRaw);
    }

    std::shared_ptr<DataBuffer> dataBuffer_;
    std::unordered_map<std::string, std::function<void()>> eventCallbacks_;
    std::shared_ptr<ConfigManager> configManager_;
    rclcpp::Logger logger_;
    std::shared_ptr<rclcpp::Clock> clock_;
    rclcpp::Node* node_;
    rclcpp::TimerBase::SharedPtr postTimer_;
    std::unique_ptr<RawStorageWorker> storageWorker_;
};
