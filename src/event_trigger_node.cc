#include <chrono>
#include <exception>
#include <future>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "datacache/srv/event_trigger.hpp"

class EventTriggerNode : public rclcpp::Node {
public:
    EventTriggerNode() : Node("event_trigger_node") {
        declare_parameter<int>("interval", 0);
        declare_parameter<std::string>("event_name", "collision");

        const int interval = get_parameter("interval").as_int();
        eventName_ = get_parameter("event_name").as_string();

        clientGroup_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        serviceGroup_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        client_ = create_client<datacache::srv::EventTrigger>("/trigger_event",
                                                              rclcpp::ServicesQoS(), clientGroup_);

        requestService_ = create_service<datacache::srv::EventTrigger>(
            "/request_trigger",
            [this](const std::shared_ptr<datacache::srv::EventTrigger::Request> request,
                   std::shared_ptr<datacache::srv::EventTrigger::Response> response) {
                forwardTrigger(*request, *response);
            },
            rclcpp::ServicesQoS(), serviceGroup_);

        if (interval > 0) {
            const auto period = std::chrono::seconds(interval);
            timer_ = create_wall_timer(period, [this]() { sendTriggerAsync(eventName_); });
            RCLCPP_INFO(get_logger(), "Auto-trigger enabled [event=%s, interval=%ds]",
                        eventName_.c_str(), interval);
        } else {
            RCLCPP_INFO(get_logger(), "Manual-trigger mode [event=%s]", eventName_.c_str());
        }
    }

private:
    // /request_trigger 是强语义代理：使用独立回调组和多线程执行器，
    // 等待 datacache_node 的真实结果后再回复调用方。
    void forwardTrigger(const datacache::srv::EventTrigger::Request& incoming,
                        datacache::srv::EventTrigger::Response& outgoing) {
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(get_logger(), "/trigger_event service not available");
            outgoing.success = false;
            outgoing.message = "Failed to reach datacache_node";
            return;
        }

        auto request = std::make_shared<datacache::srv::EventTrigger::Request>();
        request->event_name = incoming.event_name.empty() ? eventName_ : incoming.event_name;
        request->trigger_id = incoming.trigger_id;
        request->source = incoming.source.empty() ? "request_trigger" : incoming.source;
        auto future = client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            client_->remove_pending_request(future);
            outgoing.success = false;
            outgoing.message = "Timed out waiting for datacache_node";
            return;
        }
        try {
            outgoing = *future.get();
        } catch (const std::exception& error) {
            outgoing.success = false;
            outgoing.message = std::string("Trigger RPC failed: ") + error.what();
        }
    }

    bool sendTriggerAsync(const std::string& eventName) {
        if (!client_->service_is_ready()) {
            RCLCPP_WARN(get_logger(), "/trigger_event service not available");
            return false;
        }
        auto request = std::make_shared<datacache::srv::EventTrigger::Request>();
        request->event_name = eventName;
        request->source = "event_trigger_timer";
        client_->async_send_request(
            request,
            [this, eventName](rclcpp::Client<datacache::srv::EventTrigger>::SharedFuture future) {
                try {
                    const auto result = future.get();
                    if (result->success) {
                        RCLCPP_INFO(get_logger(), "Event triggered: %s", result->message.c_str());
                    } else {
                        RCLCPP_WARN(get_logger(), "Trigger failed: %s", result->message.c_str());
                    }
                } catch (const std::exception& error) {
                    RCLCPP_ERROR(get_logger(), "Trigger RPC failed: %s", error.what());
                }
            });
        return true;
    }

    std::string eventName_;
    rclcpp::Client<datacache::srv::EventTrigger>::SharedPtr client_;
    rclcpp::Service<datacache::srv::EventTrigger>::SharedPtr requestService_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr clientGroup_;
    rclcpp::CallbackGroup::SharedPtr serviceGroup_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EventTriggerNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
