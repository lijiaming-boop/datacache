#pragma once

#include "approximate_synchronizer.hpp"
#include "config_manager.hpp"
#include "databuffer.hpp"
#include "event_monitor.hpp"
#include "pair_index.hpp"
#include "sensor_watchdog.hpp"
#include "upload_worker.hpp"

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "datacache/srv/event_trigger.hpp"

class DataCacheNode : public rclcpp::Node {
public:
    DataCacheNode();

private:
    void loadConfiguration();
    void createCoreComponents();
    void createWatchdog();
    void createUploader();
    void registerEvents();
    void createSubscriptions();
    void createTriggerService();

    void handleImage(sensor_msgs::msg::Image::SharedPtr message);
    void handlePointCloud(sensor_msgs::msg::PointCloud2::SharedPtr message);
    void handleTrigger(const std::shared_ptr<datacache::srv::EventTrigger::Request>& request,
                       const std::shared_ptr<datacache::srv::EventTrigger::Response>& response);

    std::shared_ptr<ConfigManager> configManager_;
    std::shared_ptr<DataBuffer> dataBuffer_;
    std::shared_ptr<ApproximateSynchronizer> synchronizer_;
    std::shared_ptr<PairIndex> pairIndex_;
    std::shared_ptr<EventMonitor> eventMonitor_;
    std::unique_ptr<SensorWatchdog> watchdog_;
    rclcpp::TimerBase::SharedPtr watchdogTimer_;
    std::unique_ptr<UploadWorker> uploadWorker_;

    std::string configPath_;
    int configuredBufferSize_{0};
    bool syncEnabled_{true};
    bool syncRequiredForRecording_{false};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSub_;
    rclcpp::Service<datacache::srv::EventTrigger>::SharedPtr triggerService_;
};
