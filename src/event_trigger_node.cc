#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class EventTriggerNode : public rclcpp::Node {
public:
    EventTriggerNode() : Node("event_trigger_node") {
        declare_parameter<int>("interval", 0);
        declare_parameter<std::string>("event_name", "collision");

        const int interval = get_parameter("interval").as_int();
        eventName_ = get_parameter("event_name").as_string();

        client_ = create_client<std_srvs::srv::Trigger>("/trigger_event");

        requestService_ = create_service<std_srvs::srv::Trigger>(
            "/request_trigger",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                const std::string name = request->message.empty() ? eventName_ : request->message;
                const bool ok = sendTrigger(name);
                response->success = ok;
                response->message = ok
                    ? "Forwarded event '" + name + "' to datacache_node"
                    : "Failed to reach datacache_node";
            });

        if (interval > 0) {
            const auto period = std::chrono::seconds(interval);
            timer_ = create_wall_timer(period, [this]() {
                sendTrigger(eventName_);
            });
            RCLCPP_INFO(get_logger(), "Auto-trigger enabled [event=%s, interval=%ds]",
                        eventName_.c_str(), interval);
        } else {
            RCLCPP_INFO(get_logger(), "Manual-trigger mode [event=%s]", eventName_.c_str());
        }
    }

private:
    bool sendTrigger(const std::string& eventName) {
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(get_logger(), "/trigger_event service not available");
            return false;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        request->message = eventName;

        auto future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(get_node_base_interface(), future,
                                               std::chrono::seconds(3))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_WARN(get_logger(), "Trigger request timed out");
            return false;
        }

        const auto result = future.get();
        if (result->success) {
            RCLCPP_INFO(get_logger(), "Event triggered: %s", result->message.c_str());
        } else {
            RCLCPP_WARN(get_logger(), "Trigger failed: %s", result->message.c_str());
        }
        return result->success;
    }

    std::string eventName_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr requestService_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EventTriggerNode>());
    rclcpp::shutdown();
    return 0;
}
