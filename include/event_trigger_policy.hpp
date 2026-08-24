#pragma once

#include <chrono>
#include <cctype>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace event_trigger_policy {

inline std::string trim(const std::string& value) {
    std::size_t begin = 0;
    while (begin < value.size() && std::isspace(static_cast<unsigned char>(value[begin]))) {
        ++begin;
    }
    std::size_t end = value.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
        --end;
    }
    return value.substr(begin, end - begin);
}

inline bool validEventName(const std::string& name) {
    if (name.empty() || name.size() > 96 ||
        !std::isalnum(static_cast<unsigned char>(name.front()))) {
        return false;
    }
    for (const auto character : name) {
        if (!std::isalnum(static_cast<unsigned char>(character)) && character != '_' &&
            character != '-' && character != '.') {
            return false;
        }
    }
    return true;
}

inline std::vector<std::string> parseEventNames(const std::string& configured) {
    std::vector<std::string> events;
    std::set<std::string> seen;
    std::size_t begin = 0;
    while (begin <= configured.size()) {
        const auto separator = configured.find(',', begin);
        const auto name = trim(configured.substr(begin, separator - begin));
        if (validEventName(name) && seen.insert(name).second) {
            events.push_back(name);
        }
        if (separator == std::string::npos) {
            break;
        }
        begin = separator + 1;
    }
    return events;
}

inline std::map<char, std::string> parseKeyBindings(const std::vector<std::string>& configured) {
    std::map<char, std::string> bindings;
    for (const auto& entry : configured) {
        const auto separator = entry.find('=');
        if (separator == std::string::npos) {
            continue;
        }
        const auto key = trim(entry.substr(0, separator));
        const auto event = trim(entry.substr(separator + 1));
        if (key.size() == 1 && validEventName(event)) {
            bindings[key.front()] = event;
        }
    }
    return bindings;
}

class QuietPeriodDebouncer {
public:
    using Clock = std::chrono::steady_clock;

    explicit QuietPeriodDebouncer(std::chrono::milliseconds quietPeriod)
        : quietPeriod_(quietPeriod) {}

    bool accept(char key, Clock::time_point now = Clock::now()) {
        const auto found = lastInput_.find(key);
        const bool accepted = found == lastInput_.end() || now - found->second >= quietPeriod_;
        lastInput_[key] = now;
        return accepted;
    }

private:
    std::chrono::milliseconds quietPeriod_;
    std::unordered_map<char, Clock::time_point> lastInput_;
};

} // namespace event_trigger_policy
