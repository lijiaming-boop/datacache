#pragma once

#include "config_manager.hpp"
#include "databuffer.hpp"
#include "event_monitor.hpp"
#include "approximate_synchronizer.hpp"
#include "pair_index.hpp"

#include <memory>
#include <algorithm>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "datacache/srv/event_trigger.hpp"

class DataCacheNode : public rclcpp::Node {
public:
    DataCacheNode() : Node("datacache_node") {
        declare_parameter<std::string>("config_path", "config.txt");
        const auto configPath = get_parameter("config_path").as_string();

        configManager_ = std::make_shared<ConfigManager>(get_logger());
        if (!configManager_->loadConfig(configPath)) {
            throw std::runtime_error("Failed to load config file: " + configPath);
        }
        syncEnabled_ = configManager_->getBoolConfig("sync_enabled", true);
        syncRequiredForRecording_ = configManager_->getBoolConfig(
            "sync_required_for_recording", false);
        if (syncRequiredForRecording_ && !syncEnabled_) {
            RCLCPP_WARN(get_logger(),
                        "sync_required_for_recording=true requires sync_enabled; disabling strict recording mode");
            syncRequiredForRecording_ = false;
        }

        const int configuredSize = configManager_->getIntConfig("buffer_size", 1000);
        const auto bufferSize = configuredSize > 0 ? static_cast<std::size_t>(configuredSize) : 0U;
        const auto bufferSeconds = std::max(0, configManager_->getIntConfig("buffer_duration_seconds", 30));
        dataBuffer_ = std::make_shared<DataBuffer>(
            bufferSize, rclcpp::Duration::from_seconds(bufferSeconds));
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
            [this](const std::string& sensor, const rclcpp::Time& timestamp,
                   std::uint64_t count, const std::string& reason) {
                pairIndex_->addSingle(sensor, timestamp, reason);
                RCLCPP_WARN(get_logger(),
                            "Approximate synchronizer dropped %s data (%llu total): %s",
                            sensor.c_str(), static_cast<unsigned long long>(count), reason.c_str());
            });

        eventMonitor_ = std::make_shared<EventMonitor>(
            dataBuffer_, configManager_, pairIndex_, get_logger(), get_clock(), this,
            [this]() { synchronizer_->flushUnmatched(); });

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

        if (configManager_->getBoolConfig("enable_collision_event", false)) {
            registerRecordingEvent("collision");
        }
        if (configManager_->getBoolConfig("enable_hard_brake_event", false)) {
            registerRecordingEvent("hard_brake");
        }

        const auto sensorQos = rclcpp::SensorDataQoS();

        imageSub_ = create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", sensorQos,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                // Raw retention is independent from pairing. A missing lidar
                // sample must never evict or suppress camera data.
                dataBuffer_->addData({SensorType::CAMERA,
                                      CameraData{msg->header.stamp, msg}});
                if (syncEnabled_) {
                    synchronizer_->addImage(msg);
                }
            });

        cloudSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud", sensorQos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                dataBuffer_->addData({SensorType::LIDAR,
                                      LidarData{msg->header.stamp, msg}});
                if (syncEnabled_) {
                    synchronizer_->addPointCloud(msg);
                }
            });

        triggerService_ = create_service<datacache::srv::EventTrigger>(
            "/trigger_event",
            [this](const std::shared_ptr<datacache::srv::EventTrigger::Request> request,
                   std::shared_ptr<datacache::srv::EventTrigger::Response> response) {
                const auto eventName = request->event_name;
                if (eventName.empty()) {
                    response->success = false;
                    response->message = "Event name must not be empty";
                    return;
                }

                const bool triggered = eventMonitor_->triggerEvent(eventName);
                response->success = triggered;
                response->message = triggered
                    ? "Event '" + eventName + "' accepted for storage"
                    : "Event '" + eventName + "' was not accepted (unregistered or storage unavailable)";
            });

        RCLCPP_INFO(get_logger(), "DataCacheNode initialized [config=%s, buffer_size=%d]",
                    configPath.c_str(), configuredSize);
    }

private:
    std::shared_ptr<ConfigManager> configManager_;
    std::shared_ptr<DataBuffer> dataBuffer_;
    std::shared_ptr<ApproximateSynchronizer> synchronizer_;
    std::shared_ptr<PairIndex> pairIndex_;
    bool syncEnabled_{true};
    bool syncRequiredForRecording_{false};
    std::shared_ptr<EventMonitor> eventMonitor_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSub_;
    rclcpp::Service<datacache::srv::EventTrigger>::SharedPtr triggerService_;
};
