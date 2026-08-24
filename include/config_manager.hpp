#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

class ConfigManager {
public:
    explicit ConfigManager(rclcpp::Logger logger = rclcpp::get_logger("config_manager"));

    // 解析并校验配置。未知键/类型不合法的值总是记入 validationProblems 并告警；
    // 配置文件里 config_strict=true 时任一问题都使加载失败（节点拒绝启动）。
    bool loadConfig(const std::string& configFilePath);
    std::string getConfig(const std::string& key) const;
    int getIntConfig(const std::string& key, int defaultValue) const;
    bool getBoolConfig(const std::string& key, bool defaultValue) const;

    // 最近一次 loadConfig 发现的问题（未知键、类型不合法的值），供测试与
    // 调用方检查；空向量 = 配置完全合法。
    const std::vector<std::string>& validationProblems() const { return validationProblems_; }

private:
    // 校验 config_ 中的每个键：必须在 config_keys 清单内且值可按声明类型解析。
    // 返回 false 表示存在至少一个问题且 config_strict=true。
    bool validate();

    static std::string trim(const std::string& value);

    rclcpp::Logger logger_;
    std::unordered_map<std::string, std::string> config_;
    std::vector<std::string> validationProblems_;
};
