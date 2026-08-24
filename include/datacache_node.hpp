#pragma once

#include "approximate_synchronizer.hpp"
#include "config_manager.hpp"
#include "databuffer.hpp"
#include "event_monitor.hpp"
#include "event_trigger_policy.hpp"
#include "pair_index.hpp"
#include "sensor_watchdog.hpp"
#include "upload_worker.hpp"

#include <memory>
#include <filesystem>
#include <chrono>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "datacache/msg/event_status.hpp"
#include "datacache/srv/event_trigger.hpp"

class DataCacheNode : public rclcpp::Node {
public:
    DataCacheNode();

private:
    void loadConfiguration();
    void createCoreComponents();
    void reconcileStorage();
    void createWatchdog();
    void createUploader();
    void registerEvents();
    void createSubscriptions();
    void createTriggerService();
    void createEventStatusChannel();
    void pollEventLifecycles();
    void publishEventStatus(const std::string& eventName, const std::string& triggerId,
                            const std::string& source, std::uint8_t status,
                            const std::string& message, const std::string& captureId = {},
                            const std::filesystem::path& directory = {});

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
    bool uploadEnabled_{false};
    bool uploadFailureAutoRetry_{true};

    struct EventLifecycle {
        std::string eventName;
        std::string triggerId;
        std::string source;
        std::string captureId;
        std::filesystem::path directory;
        std::chrono::steady_clock::time_point acceptedAt;
        bool storedPublished{false};
        bool uploadFailurePublished{false};
    };
    struct CachedTriggerResponse {
        datacache::srv::EventTrigger::Response response;
        std::chrono::steady_clock::time_point expiresAt;
    };
    std::mutex lifecycleMutex_;
    std::unordered_map<std::string, EventLifecycle> eventLifecycles_;
    std::mutex triggerDedupeMutex_;
    std::unordered_map<std::string, CachedTriggerResponse> triggerResponses_;
    std::chrono::milliseconds triggerDedupeTtl_{60000};
    std::atomic<std::uint64_t> nextDirectTriggerId_{0};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSub_;
    rclcpp::Service<datacache::srv::EventTrigger>::SharedPtr triggerService_;
    rclcpp::Publisher<datacache::msg::EventStatus>::SharedPtr eventStatusPublisher_;
    rclcpp::TimerBase::SharedPtr lifecycleTimer_;
};
