#pragma once

#include "config_manager.hpp"
#include "databuffer.hpp"
#include "event_monitor.hpp"

#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

class DataCacheNode : public rclcpp::Node {
public:
    DataCacheNode() : Node("datacache_node") {
        declare_parameter<std::string>("config_path", "config.txt");
        const auto configPath = get_parameter("config_path").as_string();

        configManager_ = std::make_shared<ConfigManager>(get_logger());
        if (!configManager_->loadConfig(configPath)) {
            throw std::runtime_error("Failed to load config file: " + configPath);
        }

        const int configuredSize = configManager_->getIntConfig("buffer_size", 1000);
        const auto bufferSize = configuredSize > 0 ? static_cast<std::size_t>(configuredSize) : 0U;
        dataBuffer_ = std::make_shared<DataBuffer>(bufferSize);

        eventMonitor_ = std::make_shared<EventMonitor>(dataBuffer_, configManager_, get_logger(), get_clock());

        if (configManager_->getBoolConfig("enable_collision_event", false)) {
            eventMonitor_->registerEvent("collision", [this]() {
                RCLCPP_INFO(get_logger(), "Event 'collision' triggered!");
                eventMonitor_->recordDataAroundEvent("collision");
            });
        }

        const auto sensorQos = rclcpp::SensorDataQoS();

        imageSub_ = create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", sensorQos,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                dataBuffer_->addData({SensorType::CAMERA,
                                      CameraData{msg->header.stamp, msg}});
            });

        cloudSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud", sensorQos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                dataBuffer_->addData({SensorType::LIDAR,
                                      LidarData{msg->header.stamp, msg}});
            });

        triggerService_ = create_service<std_srvs::srv::Trigger>(
            "/trigger_event",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                // Trigger.srv 的 Request 为空，无法携带事件名，触发默认的 collision 事件
                const std::string eventName = "collision";
                const bool triggered = eventMonitor_->triggerEvent(eventName);
                response->success = triggered;
                response->message = triggered
                    ? "Event '" + eventName + "' triggered"
                    : "Event '" + eventName + "' not found";
            });

        RCLCPP_INFO(get_logger(), "DataCacheNode initialized [config=%s, buffer_size=%d]",
                    configPath.c_str(), configuredSize);
    }

private:
    std::shared_ptr<ConfigManager> configManager_;
    std::shared_ptr<DataBuffer> dataBuffer_;
    std::shared_ptr<EventMonitor> eventMonitor_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr triggerService_;
};
