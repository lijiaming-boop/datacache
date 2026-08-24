#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

// Receive-time freshness monitor for sensor streams. Flags sensors whose data
// stopped arriving, based on steady_clock rather than sensor header stamps:
// header stamps can freeze or jump when a device clock misbehaves and would
// hide an actual outage.
class SensorWatchdog {
public:
    // elapsed semantics: with stale=true it is the time since the last
    // received message; with stale=false it is how long the sensor was stale.
    using TransitionCallback = std::function<void(const std::string& sensor, bool stale,
                                                  std::chrono::milliseconds elapsed)>;

    explicit SensorWatchdog(TransitionCallback onTransition)
        : onTransition_(std::move(onTransition)) {}

    SensorWatchdog(const SensorWatchdog&) = delete;
    SensorWatchdog& operator=(const SensorWatchdog&) = delete;

    // The timeout counts from registration, so a sensor that never publishes
    // anything is flagged once the timeout elapses.
    void registerSensor(const std::string& name, std::chrono::milliseconds staleTimeout) {
        std::lock_guard<std::mutex> lock(mutex_);
        sensors_.emplace(name, SensorState{staleTimeout, std::chrono::steady_clock::now(),
                                           std::chrono::steady_clock::time_point{}, false});
    }

    void noteData(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto sensor = sensors_.find(name);
        if (sensor != sensors_.end()) {
            sensor->second.lastReceive = std::chrono::steady_clock::now();
        }
    }

    // Fires the transition callback once per stale/fresh flip, not per poll.
    void poll() {
        std::vector<Transition> transitions;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto now = std::chrono::steady_clock::now();
            for (auto& [name, state] : sensors_) {
                const bool stale = now - state.lastReceive > state.staleTimeout;
                if (stale == state.stale) {
                    continue;
                }
                state.stale = stale;
                const auto elapsed = stale ? now - state.lastReceive : now - state.staleSince;
                transitions.push_back(Transition{
                    name, stale, std::chrono::duration_cast<std::chrono::milliseconds>(elapsed)});
                if (stale) {
                    state.staleSince = now;
                }
            }
        }
        for (const auto& transition : transitions) {
            if (onTransition_) {
                onTransition_(transition.sensor, transition.stale, transition.elapsed);
            }
        }
    }

    bool isStale(const std::string& name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto sensor = sensors_.find(name);
        return sensor != sensors_.end() && sensor->second.stale;
    }

    // Human-readable snapshot such as "camera: ok, lidar: STALE 2.3s".
    std::string describeStatus() const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = std::chrono::steady_clock::now();
        std::string description;
        for (const auto& [name, state] : sensors_) {
            if (!description.empty()) {
                description += ", ";
            }
            description += name + ": ";
            if (!state.stale) {
                description += "ok";
            } else {
                const auto silent =
                    std::chrono::duration_cast<std::chrono::milliseconds>(now - state.lastReceive);
                description += "STALE " + formatSeconds(silent);
            }
        }
        return description;
    }

private:
    struct SensorState {
        std::chrono::milliseconds staleTimeout{0};
        std::chrono::steady_clock::time_point lastReceive{};
        std::chrono::steady_clock::time_point staleSince{};
        bool stale{false};
    };

    struct Transition {
        std::string sensor;
        bool stale;
        std::chrono::milliseconds elapsed;
    };

    static std::string formatSeconds(std::chrono::milliseconds milliseconds) {
        const auto tenths = milliseconds.count() / 100;
        return std::to_string(tenths / 10) + "." + std::to_string(tenths % 10) + "s";
    }

    TransitionCallback onTransition_;
    std::map<std::string, SensorState> sensors_;
    mutable std::mutex mutex_;
};
