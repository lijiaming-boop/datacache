#include <chrono>
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

        client_ = create_client<datacache::srv::EventTrigger>("/trigger_event");

        requestService_ = create_service<datacache::srv::EventTrigger>(
            "/request_trigger",
            [this](const std::shared_ptr<datacache::srv::EventTrigger::Request> request,
                   std::shared_ptr<datacache::srv::EventTrigger::Response> response) {
                const auto name = request->event_name.empty() ? eventName_ : request->event_name;
                response->success = sendTrigger(name);
                response->message = response->success
                                        ? "Queued event '" + name + "' for datacache_node"
                                        : "Failed to reach datacache_node";
            });

        if (interval > 0) {
            const auto period = std::chrono::seconds(interval);
            timer_ = create_wall_timer(period, [this]() { sendTrigger(eventName_); });
            RCLCPP_INFO(get_logger(), "Auto-trigger enabled [event=%s, interval=%ds]",
                        eventName_.c_str(), interval);
        } else {
            RCLCPP_INFO(get_logger(), "Manual-trigger mode [event=%s]", eventName_.c_str());
        }
    }

private:
    // 异步发送触发请求：服务/定时器回调线程里不能同步等待结果
    // （节点已在执行器里 spin，重入 spin_until_future_complete 会抛异常），
    // 结果通过完成回调在执行器线程里打印
    bool sendTrigger(const std::string& eventName) {
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(get_logger(), "/trigger_event service not available");
            return false;
        }

        auto request = std::make_shared<datacache::srv::EventTrigger::Request>();
        request->event_name = eventName;
        client_->async_send_request(
            request,
            [this, eventName](rclcpp::Client<datacache::srv::EventTrigger>::SharedFuture future) {
                const auto result = future.get();
                if (result->success) {
                    RCLCPP_INFO(get_logger(), "Event triggered: %s", result->message.c_str());
                } else {
                    RCLCPP_WARN(get_logger(), "Trigger failed: %s", result->message.c_str());
                }
            });
        return true;
    }

    std::string eventName_;
    rclcpp::Client<datacache::srv::EventTrigger>::SharedPtr client_;
    rclcpp::Service<datacache::srv::EventTrigger>::SharedPtr requestService_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EventTriggerNode>());
    rclcpp::shutdown();
    return 0;
}
