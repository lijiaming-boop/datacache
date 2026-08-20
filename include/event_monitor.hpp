#pragma once

#include "config_manager.hpp"
#include "databuffer.hpp"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

class EventMonitor {
public:
    EventMonitor(std::shared_ptr<DataBuffer> dataBuffer, std::shared_ptr<ConfigManager> configManager,
                 rclcpp::Logger logger, std::shared_ptr<rclcpp::Clock> clock)
        : dataBuffer_(std::move(dataBuffer)), configManager_(std::move(configManager)),
          logger_(std::move(logger)), clock_(std::move(clock)) {}

    EventMonitor(const EventMonitor&) = delete;
    EventMonitor& operator=(const EventMonitor&) = delete;
    EventMonitor(EventMonitor&&) noexcept = default;
    EventMonitor& operator=(EventMonitor&&) noexcept = default;

    void registerEvent(const std::string& eventName, std::function<void()> callback) {
        eventCallbacks_[eventName] = std::move(callback);
    }

    bool triggerEvent(const std::string& eventName) const {
        const auto event = eventCallbacks_.find(eventName);
        if (event == eventCallbacks_.end()) {
            RCLCPP_WARN(logger_, "Event not found: %s", eventName.c_str());
            return false;
        }
        event->second();
        return true;
    }

    void recordDataAroundEvent(const std::string& eventName) const {
        const int preSeconds = configManager_->getIntConfig("event_pre_time", 5);
        const int postSeconds = configManager_->getIntConfig("event_post_time", 5);
        const auto now = clock_->now();
        const auto startTime = now - rclcpp::Duration::from_seconds(preSeconds);
        const auto endTime = now + rclcpp::Duration::from_seconds(postSeconds);

        RCLCPP_INFO(logger_, "Recording data around event: %s", eventName.c_str());
        const bool recordCamera = configManager_->getBoolConfig("record_camera", false);
        const bool recordLidar = configManager_->getBoolConfig("record_lidar", false);

        for (const auto& event : dataBuffer_->getDataWithinTimeRange(startTime, endTime)) {
            if (event.type == SensorType::CAMERA && recordCamera) {
                const auto& cam = std::get<CameraData>(event.data);
                RCLCPP_INFO(logger_, "CAMERA data recorded [stamp: %.6f]",
                            cam.timestamp.seconds());
            } else if (event.type == SensorType::LIDAR && recordLidar) {
                const auto& lidar = std::get<LidarData>(event.data);
                RCLCPP_INFO(logger_, "LIDAR data recorded [stamp: %.6f]",
                            lidar.timestamp.seconds());
            }
        }
    }

private:
    std::shared_ptr<DataBuffer> dataBuffer_;
    std::unordered_map<std::string, std::function<void()>> eventCallbacks_;
    std::shared_ptr<ConfigManager> configManager_;
    rclcpp::Logger logger_;
    std::shared_ptr<rclcpp::Clock> clock_;
};
