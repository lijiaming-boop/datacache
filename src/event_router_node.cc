#include "event_router_policy.hpp"
#include "event_trigger_policy.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "datacache/msg/event_signal.hpp"
#include "datacache/msg/event_status.hpp"
#include "datacache/srv/event_trigger.hpp"

class EventRouterNode : public rclcpp::Node {
public:
    EventRouterNode() : Node("event_router_node") {
        declare_parameter<std::string>("signal_topic", "/event_signal");
        declare_parameter<std::string>("status_topic", "/event_status");
        declare_parameter<std::string>("service_name", "/trigger_event");
        declare_parameter<int>("dedupe_ttl_ms", 60000);
        declare_parameter<int>("cooldown_ms", 1000);

        const auto dedupeTtl = std::chrono::milliseconds(
            std::max<std::int64_t>(1, get_parameter("dedupe_ttl_ms").as_int()));
        const auto cooldown = std::chrono::milliseconds(
            std::max<std::int64_t>(0, get_parameter("cooldown_ms").as_int()));
        policy_ = std::make_unique<event_router_policy::EventRouterPolicy>(dedupeTtl, cooldown);

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();
        statusPublisher_ = create_publisher<datacache::msg::EventStatus>(
            get_parameter("status_topic").as_string(), qos);
        triggerClient_ =
            create_client<datacache::srv::EventTrigger>(get_parameter("service_name").as_string());
        signalSubscription_ = create_subscription<datacache::msg::EventSignal>(
            get_parameter("signal_topic").as_string(), qos,
            [this](datacache::msg::EventSignal::SharedPtr signal) { route(std::move(signal)); });

        RCLCPP_INFO(get_logger(), "Event router ready [dedupe=%lldms, cooldown=%lldms]",
                    static_cast<long long>(dedupeTtl.count()),
                    static_cast<long long>(cooldown.count()));
    }

private:
    void route(datacache::msg::EventSignal::SharedPtr signal) {
        if (!event_trigger_policy::validEventName(signal->event_name)) {
            publishStatus(*signal, datacache::msg::EventStatus::REJECTED, "invalid event name");
            return;
        }
        if (signal->source.empty()) {
            signal->source = "unknown";
        }
        if (signal->trigger_id.empty()) {
            signal->trigger_id = signal->source + "-" + std::to_string(now().nanoseconds()) + "-" +
                                 std::to_string(nextTriggerId_.fetch_add(1));
        }

        publishStatus(*signal, datacache::msg::EventStatus::RECEIVED, "event received");
        if (!triggerClient_->service_is_ready()) {
            publishStatus(*signal, datacache::msg::EventStatus::REJECTED,
                          "trigger service unavailable");
            return;
        }
        const auto decision = policy_->evaluate(signal->event_name, signal->trigger_id);
        if (decision == event_router_policy::Decision::Duplicate) {
            publishStatus(*signal, datacache::msg::EventStatus::REJECTED, "duplicate trigger_id");
            return;
        }
        if (decision == event_router_policy::Decision::CoolingDown) {
            publishStatus(*signal, datacache::msg::EventStatus::REJECTED, "event is cooling down");
            return;
        }
        auto request = std::make_shared<datacache::srv::EventTrigger::Request>();
        request->event_name = signal->event_name;
        request->trigger_id = signal->trigger_id;
        request->source = signal->source;
        triggerClient_->async_send_request(
            request,
            [this, signal](rclcpp::Client<datacache::srv::EventTrigger>::SharedFuture future) {
                try {
                    const auto response = future.get();
                    if (!response->success) {
                        policy_->release(signal->trigger_id);
                    }
                } catch (const std::exception& error) {
                    policy_->release(signal->trigger_id);
                    publishStatus(*signal, datacache::msg::EventStatus::REJECTED,
                                  std::string("trigger RPC failed: ") + error.what());
                }
            });
    }

    void publishStatus(const datacache::msg::EventSignal& signal, std::uint8_t status,
                       const std::string& message) {
        datacache::msg::EventStatus update;
        update.status = status;
        update.event_name = signal.event_name;
        update.trigger_id = signal.trigger_id;
        update.source = signal.source;
        update.message = message;
        update.updated_at = static_cast<builtin_interfaces::msg::Time>(now());
        statusPublisher_->publish(update);
    }

    std::unique_ptr<event_router_policy::EventRouterPolicy> policy_;
    rclcpp::Publisher<datacache::msg::EventStatus>::SharedPtr statusPublisher_;
    rclcpp::Subscription<datacache::msg::EventSignal>::SharedPtr signalSubscription_;
    rclcpp::Client<datacache::srv::EventTrigger>::SharedPtr triggerClient_;
    std::atomic<std::uint64_t> nextTriggerId_{0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EventRouterNode>());
    rclcpp::shutdown();
    return 0;
}
