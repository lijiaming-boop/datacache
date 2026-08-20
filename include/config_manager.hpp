#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

class ConfigManager {
public:
    explicit ConfigManager(rclcpp::Logger logger = rclcpp::get_logger("config_manager"))
        : logger_(logger) {}

    bool loadConfig(const std::string& configFilePath) {
        std::ifstream configFile(configFilePath);
        if (!configFile.is_open()) {
            RCLCPP_ERROR(logger_, "Unable to open config file: %s", configFilePath.c_str());
            return false;
        }

        config_.clear();
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
        return true;
    }

    std::string getConfig(const std::string& key) const {
        const auto item = config_.find(key);
        return item == config_.end() ? "" : item->second;
    }

    int getIntConfig(const std::string& key, int defaultValue) const {
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

    bool getBoolConfig(const std::string& key, bool defaultValue) const {
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

private:
    static std::string trim(const std::string& value) {
        const auto first = value.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) {
            return "";
        }
        const auto last = value.find_last_not_of(" \t\r\n");
        return value.substr(first, last - first + 1);
    }

    rclcpp::Logger logger_;
    std::unordered_map<std::string, std::string> config_;
};
