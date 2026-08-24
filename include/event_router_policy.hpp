#pragma once

#include <chrono>
#include <string>
#include <unordered_map>
#include <utility>

namespace event_router_policy {

enum class Decision {
    Accept,
    Duplicate,
    CoolingDown,
};

class EventRouterPolicy {
public:
    using Clock = std::chrono::steady_clock;

    EventRouterPolicy(std::chrono::milliseconds dedupeTtl, std::chrono::milliseconds cooldown)
        : dedupeTtl_(dedupeTtl), cooldown_(cooldown) {}

    Decision evaluate(const std::string& eventName, const std::string& triggerId,
                      Clock::time_point now = Clock::now()) {
        prune(now);
        if (seenTriggers_.find(triggerId) != seenTriggers_.end()) {
            return Decision::Duplicate;
        }
        seenTriggers_[triggerId] = now + dedupeTtl_;

        const auto previous = lastAccepted_.find(eventName);
        if (previous != lastAccepted_.end() && now - previous->second < cooldown_) {
            seenTriggers_.erase(triggerId);
            return Decision::CoolingDown;
        }
        lastAccepted_[eventName] = now;
        acceptedByTrigger_[triggerId] = std::make_pair(eventName, now);
        return Decision::Accept;
    }

    void release(const std::string& triggerId) {
        seenTriggers_.erase(triggerId);
        const auto accepted = acceptedByTrigger_.find(triggerId);
        if (accepted == acceptedByTrigger_.end()) {
            return;
        }
        const auto latest = lastAccepted_.find(accepted->second.first);
        if (latest != lastAccepted_.end() && latest->second == accepted->second.second) {
            lastAccepted_.erase(latest);
        }
        acceptedByTrigger_.erase(accepted);
    }

private:
    void prune(Clock::time_point now) {
        for (auto item = seenTriggers_.begin(); item != seenTriggers_.end();) {
            if (item->second <= now) {
                acceptedByTrigger_.erase(item->first);
                item = seenTriggers_.erase(item);
            } else {
                ++item;
            }
        }
    }

    std::chrono::milliseconds dedupeTtl_;
    std::chrono::milliseconds cooldown_;
    std::unordered_map<std::string, Clock::time_point> seenTriggers_;
    std::unordered_map<std::string, Clock::time_point> lastAccepted_;
    std::unordered_map<std::string, std::pair<std::string, Clock::time_point>> acceptedByTrigger_;
};

} // namespace event_router_policy
