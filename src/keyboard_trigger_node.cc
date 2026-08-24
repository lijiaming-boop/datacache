#include "event_trigger_policy.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cerrno>
#include <cstring>
#include <memory>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include "datacache/msg/event_signal.hpp"
#include "datacache/msg/event_status.hpp"

class KeyboardTriggerNode : public rclcpp::Node {
public:
    KeyboardTriggerNode() : Node("keyboard_trigger_node") {
        declare_parameter<std::vector<std::string>>(
            "key_bindings", {"c=collision", "b=hard_brake", "m=manual_capture"});
        declare_parameter<int>("release_quiet_ms", 700);
        declare_parameter<std::string>("signal_topic", "/event_signal");
        declare_parameter<std::string>("status_topic", "/event_status");
        declare_parameter<std::string>("quit_key", "q");
        declare_parameter<bool>("wait_for_upload", false);
        declare_parameter<int>("pending_timeout_ms", 600000);

        bindings_ =
            event_trigger_policy::parseKeyBindings(get_parameter("key_bindings").as_string_array());
        const auto quietMs = std::max<std::int64_t>(1, get_parameter("release_quiet_ms").as_int());
        debouncer_ = std::make_unique<event_trigger_policy::QuietPeriodDebouncer>(
            std::chrono::milliseconds(quietMs));
        const auto quit = get_parameter("quit_key").as_string();
        quitKey_ = quit.empty() ? '\0' : quit.front();
        waitForUpload_ = get_parameter("wait_for_upload").as_bool();
        pendingTimeout_ = std::chrono::milliseconds(
            std::max<std::int64_t>(1000, get_parameter("pending_timeout_ms").as_int()));
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();
        signalPublisher_ = create_publisher<datacache::msg::EventSignal>(
            get_parameter("signal_topic").as_string(), qos);
        statusSubscription_ = create_subscription<datacache::msg::EventStatus>(
            get_parameter("status_topic").as_string(), qos,
            [this](datacache::msg::EventStatus::SharedPtr status) { handleStatus(*status); });
        pendingTimer_ = create_wall_timer(std::chrono::seconds(30), [this]() { prunePending(); });

        if (bindings_.empty()) {
            throw std::runtime_error("keyboard key_bindings contains no valid key=event entries");
        }
        std::string description;
        for (const auto& [key, event] : bindings_) {
            if (!description.empty()) {
                description += ", ";
            }
            description += key + std::string("=") + event;
        }
        RCLCPP_INFO(get_logger(), "Keyboard trigger ready [%s, quit=%c]", description.c_str(),
                    quitKey_ ? quitKey_ : '-');
        inputThread_ = std::thread([this]() { readInput(); });
    }

    ~KeyboardTriggerNode() override {
        stopping_.store(true);
        if (inputThread_.joinable()) {
            inputThread_.join();
        }
    }

private:
    class TerminalMode {
    public:
        TerminalMode() {
            if (!::isatty(STDIN_FILENO) || ::tcgetattr(STDIN_FILENO, &original_) != 0) {
                return;
            }
            auto raw = original_;
            raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 0;
            active_ = ::tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
        }

        ~TerminalMode() {
            if (active_) {
                ::tcsetattr(STDIN_FILENO, TCSANOW, &original_);
            }
        }

    private:
        termios original_{};
        bool active_{false};
    };

    void readInput() {
        TerminalMode terminal;
        while (!stopping_.load() && rclcpp::ok()) {
            fd_set descriptors;
            FD_ZERO(&descriptors);
            FD_SET(STDIN_FILENO, &descriptors);
            timeval timeout{0, 100000};
            const auto ready = ::select(STDIN_FILENO + 1, &descriptors, nullptr, nullptr, &timeout);
            if (ready < 0) {
                if (errno == EINTR) {
                    continue;
                }
                RCLCPP_ERROR(get_logger(), "Keyboard select failed: %s", std::strerror(errno));
                return;
            }
            if (ready == 0) {
                continue;
            }
            char input[32];
            const auto count = ::read(STDIN_FILENO, input, sizeof(input));
            if (count < 0) {
                if (errno != EINTR && errno != EAGAIN) {
                    RCLCPP_ERROR(get_logger(), "Keyboard read failed: %s", std::strerror(errno));
                    return;
                }
                continue;
            }
            if (count == 0) {
                return;
            }
            for (ssize_t index = 0; index < count; ++index) {
                handleKey(input[index]);
            }
        }
    }

    void handleKey(char key) {
        if (quitKey_ != '\0' && key == quitKey_) {
            RCLCPP_INFO(get_logger(), "Quit key received");
            rclcpp::shutdown();
            return;
        }
        const auto binding = bindings_.find(key);
        if (binding == bindings_.end() || !debouncer_->accept(key)) {
            return;
        }
        sendTrigger(key, binding->second);
    }

    void sendTrigger(char key, const std::string& eventName) {
        datacache::msg::EventSignal signal;
        signal.event_name = eventName;
        signal.source = "keyboard";
        signal.trigger_id = "keyboard-" + std::to_string(now().nanoseconds()) + "-" +
                            std::to_string(nextTriggerId_.fetch_add(1));
        signal.occurred_at = static_cast<builtin_interfaces::msg::Time>(now());
        {
            std::lock_guard<std::mutex> lock(pendingMutex_);
            pending_[signal.trigger_id] = {key, eventName, std::chrono::steady_clock::now()};
        }
        signalPublisher_->publish(signal);
        RCLCPP_INFO(get_logger(), "Key '%c' published event '%s' [%s]", key, eventName.c_str(),
                    signal.trigger_id.c_str());
    }

    void handleStatus(const datacache::msg::EventStatus& status) {
        std::lock_guard<std::mutex> lock(pendingMutex_);
        const auto pending = pending_.find(status.trigger_id);
        if (pending == pending_.end()) {
            return;
        }
        const auto key = pending->second.key;
        if (status.status == datacache::msg::EventStatus::ACCEPTED) {
            RCLCPP_INFO(get_logger(), "Key '%c' accepted: %s", key, status.message.c_str());
        } else if (status.status == datacache::msg::EventStatus::STORED) {
            RCLCPP_INFO(get_logger(), "Key '%c' stored: %s", key, status.record_directory.c_str());
            if (!waitForUpload_) {
                pending_.erase(pending);
            }
        } else if (status.status == datacache::msg::EventStatus::UPLOADED) {
            RCLCPP_INFO(get_logger(), "Key '%c' uploaded: %s", key,
                        status.record_directory.c_str());
            pending_.erase(pending);
        } else if (status.status == datacache::msg::EventStatus::UPLOAD_RETRYING) {
            RCLCPP_WARN(get_logger(), "Key '%c' upload delayed: %s", key, status.message.c_str());
        } else if (status.status == datacache::msg::EventStatus::REJECTED ||
                   status.status == datacache::msg::EventStatus::RECORD_FAILED ||
                   status.status == datacache::msg::EventStatus::UPLOAD_FAILED) {
            RCLCPP_WARN(get_logger(), "Key '%c' failed: %s", key, status.message.c_str());
            pending_.erase(pending);
        }
    }

    void prunePending() {
        std::lock_guard<std::mutex> lock(pendingMutex_);
        const auto cutoff = std::chrono::steady_clock::now() - pendingTimeout_;
        for (auto item = pending_.begin(); item != pending_.end();) {
            if (item->second.createdAt < cutoff) {
                RCLCPP_WARN(get_logger(), "Dropping stale pending trigger %s", item->first.c_str());
                item = pending_.erase(item);
            } else {
                ++item;
            }
        }
    }

    std::map<char, std::string> bindings_;
    std::unique_ptr<event_trigger_policy::QuietPeriodDebouncer> debouncer_;
    char quitKey_{'q'};
    rclcpp::Publisher<datacache::msg::EventSignal>::SharedPtr signalPublisher_;
    rclcpp::Subscription<datacache::msg::EventStatus>::SharedPtr statusSubscription_;
    rclcpp::TimerBase::SharedPtr pendingTimer_;
    struct PendingEntry {
        char key;
        std::string eventName;
        std::chrono::steady_clock::time_point createdAt;
    };
    std::mutex pendingMutex_;
    std::unordered_map<std::string, PendingEntry> pending_;
    bool waitForUpload_{false};
    std::chrono::milliseconds pendingTimeout_{600000};
    std::atomic<std::uint64_t> nextTriggerId_{0};
    std::thread inputThread_;
    std::atomic<bool> stopping_{false};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardTriggerNode>());
    rclcpp::shutdown();
    return 0;
}
