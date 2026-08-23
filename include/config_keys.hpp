#pragma once

#include <string>
#include <vector>

// 全部合法配置键的权威清单，供 ConfigManager 在加载时校验拼写错误的键。
// 新增配置键时必须同步在此登记，否则用户配置文件里的新键会被当作拼写错误
// 告警（config_strict=true 时直接拒绝启动）。
namespace config_keys {

enum class Type {
    String,
    Int,
    Bool,
};

inline const std::vector<std::pair<const char*, Type>>& exactKeys() {
    static const std::vector<std::pair<const char*, Type>> keys = {
        {"buffer_size", Type::Int},
        {"buffer_duration_seconds", Type::Int},
        {"sync_enabled", Type::Bool},
        {"sync_queue_size", Type::Int},
        {"sync_tolerance_ms", Type::Int},
        {"sync_required_for_recording", Type::Bool},
        {"max_pending_storage_jobs", Type::Int},
        {"max_active_event_captures", Type::Int},
        {"event_scheduler_period_ms", Type::Int},
        {"sensor_stall_grace_ms", Type::Int},
        {"watchdog_enabled", Type::Bool},
        {"watchdog_check_period_ms", Type::Int},
        {"watchdog_stale_timeout_ms", Type::Int},
        {"compression_enabled", Type::Bool},
        {"compression_level", Type::Int},
        {"keep_raw_after_compression", Type::Bool},
        {"conversion_enabled", Type::Bool},
        {"image_format", Type::String},
        {"image_quality", Type::Int},
        {"pointcloud_format", Type::String},
        {"disk_min_free_mb", Type::Int},
        {"retention_days", Type::Int},
        {"retention_max_capacity_mb", Type::Int},
        {"disk_cleanup_interval_seconds", Type::Int},
        {"upload_enabled", Type::Bool},
        {"upload_url", Type::String},
        {"upload_scan_period_ms", Type::Int},
        {"upload_timeout_s", Type::Int},
        {"upload_max_retries", Type::Int},
        {"upload_retry_backoff_ms", Type::Int},
        {"event_pre_time", Type::Int},
        {"event_post_time", Type::Int},
        {"record_camera", Type::Bool},
        {"record_lidar", Type::Bool},
        {"record_directory", Type::String},
        {"config_strict", Type::Bool},
    };
    return keys;
}

// 按前缀/后缀生成的键族（<event>/<sensor> 段可为任意非空名字，事件名可含 '_'）:
//   enable_<event>_event                       Bool
//   event_<event>_pre_time / _post_time        Int
//   event_<event>_record_camera / _record_lidar Bool
//   watchdog_<sensor>_stale_timeout_ms         Int
inline bool matchesPatternKey(const std::string& key, Type& type) {
    // starts_with/ends_with 是 C++20 API, 项目为 C++17, 用 compare 实现
    const auto hasAffixes = [&key](const std::string& prefix, const std::string& suffix) {
        return key.size() > prefix.size() + suffix.size() &&
               key.compare(0, prefix.size(), prefix) == 0 &&
               key.compare(key.size() - suffix.size(), suffix.size(), suffix) == 0;
    };
    if (hasAffixes("enable_", "_event")) {
        type = Type::Bool;
        return true;
    }
    if (hasAffixes("event_", "_pre_time") || hasAffixes("event_", "_post_time")) {
        type = Type::Int;
        return true;
    }
    if (hasAffixes("event_", "_record_camera") || hasAffixes("event_", "_record_lidar")) {
        type = Type::Bool;
        return true;
    }
    if (hasAffixes("watchdog_", "_stale_timeout_ms")) {
        type = Type::Int;
        return true;
    }
    return false;
}

// 键的声明类型；未知键返回 false。全局回退键（如 event_pre_time）在精确清单里，
// 精确匹配优先于模式匹配，因此不会被误认成"event_<空>_pre_time"。
inline bool lookupType(const std::string& key, Type& type) {
    for (const auto& [name, keyType] : exactKeys()) {
        if (key == name) {
            type = keyType;
            return true;
        }
    }
    return matchesPatternKey(key, type);
}

} // namespace config_keys
