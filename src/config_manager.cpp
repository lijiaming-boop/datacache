#include "config_manager.hpp"

#include "config_keys.hpp"

#include <fstream>
#include <sstream>

ConfigManager::ConfigManager(rclcpp::Logger logger) : logger_(logger) {}

bool ConfigManager::loadConfig(const std::string& configFilePath) {
    std::ifstream configFile(configFilePath);
    if (!configFile.is_open()) {
        RCLCPP_ERROR(logger_, "Unable to open config file: %s", configFilePath.c_str());
        return false;
    }

    config_.clear();
    validationProblems_.clear();
    std::string line;
    while (std::getline(configFile, line)) {
        const auto commentPosition = line.find('#');
        if (commentPosition != std::string::npos) {
            line.erase(commentPosition);
        }

        std::stringstream stream(line);
        std::string key;
        std::string value;
        if (std::getline(stream, key, '=') && std::getline(stream, value)) {
            config_[trim(key)] = trim(value);
        }
    }
    return validate();
}

// 拼写错误的键会静默落到代码默认值, 是"配置明明写了却不生效"的主要来源;
// 这里逐键对清单校验并显式告警, 严格模式直接拒绝启动。
bool ConfigManager::validate() {
    static const auto parseInt = [](const std::string& text) {
        // 与 getIntConfig 的 stoi 语义一致但要求整串消费, 拒绝 "12abc" 这类残留垃圾
        try {
            std::size_t consumed = 0;
            std::stoi(text, &consumed);
            return consumed == text.size();
        } catch (const std::exception&) {
            return false;
        }
    };

    const bool strict = getBoolConfig("config_strict", false);
    for (const auto& [key, value] : config_) {
        config_keys::Type type;
        if (!config_keys::lookupType(key, type)) {
            validationProblems_.push_back("unknown config key '" + key + "'");
            RCLCPP_WARN(logger_, "Unknown config key '%s' (typo? value ignored: '%s')", key.c_str(),
                        value.c_str());
            continue;
        }
        const char* expected = nullptr;
        if (type == config_keys::Type::Int && !parseInt(value)) {
            expected = "int";
        } else if (type == config_keys::Type::Bool && value != "true" && value != "false" &&
                   value != "1" && value != "0") {
            expected = "bool (true/false/1/0)";
        }
        if (expected != nullptr) {
            validationProblems_.push_back(std::string("invalid ") + expected + " value for '" +
                                          key + "': '" + value + "'");
            RCLCPP_WARN(logger_, "Invalid value for %s: '%s' (expected %s)", key.c_str(),
                        value.c_str(), expected);
        }
    }
    if (strict && !validationProblems_.empty()) {
        RCLCPP_ERROR(logger_,
                     "config_strict=true and the config file has %zu problem(s); refusing to load",
                     validationProblems_.size());
        return false;
    }
    return true;
}

std::string ConfigManager::getConfig(const std::string& key) const {
    const auto item = config_.find(key);
    return item == config_.end() ? "" : item->second;
}

int ConfigManager::getIntConfig(const std::string& key, int defaultValue) const {
    const auto item = config_.find(key);
    if (item == config_.end()) {
        return defaultValue;
    }

    try {
        return std::stoi(item->second);
    } catch (const std::exception&) {
        RCLCPP_WARN(logger_, "Invalid value for %s, using default", key.c_str());
        return defaultValue;
    }
}

bool ConfigManager::getBoolConfig(const std::string& key, bool defaultValue) const {
    const auto item = config_.find(key);
    if (item == config_.end()) {
        return defaultValue;
    }
    if (item->second == "true" || item->second == "1") {
        return true;
    }
    if (item->second == "false" || item->second == "0") {
        return false;
    }
    return defaultValue;
}

std::string ConfigManager::trim(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}
