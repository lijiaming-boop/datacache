#pragma once

#include <string>
#include <unordered_map>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

class ConfigManager {
public:
    explicit ConfigManager(
        rclcpp::Logger logger = rclcpp::get_logger("config_manager"));

    bool loadConfig(const std::string& configFilePath);
    std::string getConfig(const std::string& key) const;
    int getIntConfig(const std::string& key, int defaultValue) const;
    bool getBoolConfig(const std::string& key, bool defaultValue) const;

private:
    static std::string trim(const std::string& value);

    rclcpp::Logger logger_;
    std::unordered_map<std::string, std::string> config_;
};
