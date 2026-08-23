#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

// 磁盘空间管理：写前剩余空间检查 + 按天数/容量的事件目录保留清理。
// 所有方法只在 RawStorageWorker 的后台线程上调用，无需加锁。
class DiskSpaceManager {
public:
    struct Policy {
        std::uintmax_t minFreeBytes{0};     // 写前最低剩余空间，0 = 禁用
        std::uintmax_t maxCapacityBytes{0}; // records 总量上限，0 = 禁用
        int retentionDays{0};               // 事件目录保留天数，0 = 禁用
        std::chrono::seconds cleanupInterval{60};
    };

    DiskSpaceManager(std::filesystem::path recordRoot, Policy policy, rclcpp::Logger logger)
        : recordRoot_(std::move(recordRoot)), policy_(policy), logger_(std::move(logger)) {
        // steady_clock 以开机为纪元，零点回拨一个间隔，确保首次调用不被误节流
        lastSweep_ = std::chrono::steady_clock::now() - policy_.cleanupInterval;
        if (policy_.minFreeBytes > 0 || policy_.maxCapacityBytes > 0 || policy_.retentionDays > 0) {
            constexpr std::uintmax_t kMegabyte = 1024ULL * 1024ULL;
            RCLCPP_INFO(logger_,
                        "Disk space management for %s: min_free=%llu MB, capacity=%llu MB, "
                        "retention_days=%d, cleanup_interval=%lld s",
                        recordRoot_.string().c_str(),
                        static_cast<unsigned long long>(policy_.minFreeBytes / kMegabyte),
                        static_cast<unsigned long long>(policy_.maxCapacityBytes / kMegabyte),
                        policy_.retentionDays,
                        static_cast<long long>(policy_.cleanupInterval.count()));
        }
    }

    DiskSpaceManager(const DiskSpaceManager&) = delete;
    DiskSpaceManager& operator=(const DiskSpaceManager&) = delete;

    // 写前调用：剩余空间充足返回 true；不足时强制清理后复查，仍不足返回 false
    bool prepareForWrite() {
        if (policy_.minFreeBytes == 0) {
            return true;
        }
        if (availableBytes() >= policy_.minFreeBytes) {
            return true;
        }
        RCLCPP_WARN(logger_,
                    "Free disk space below %llu MB; running retention cleanup before writing",
                    megabytes(policy_.minFreeBytes));
        enforceRetention(true);
        if (availableBytes() >= policy_.minFreeBytes) {
            return true;
        }
        RCLCPP_ERROR(logger_, "Free disk space still below %llu MB after cleanup",
                     megabytes(policy_.minFreeBytes));
        return false;
    }

    // 天数 + 容量保留清理；非 force 时按 cleanupInterval 节流
    void enforceRetention(bool force = false) {
        const bool ageEnabled = policy_.retentionDays > 0;
        const bool capacityEnabled = policy_.maxCapacityBytes > 0;
        if (!ageEnabled && !capacityEnabled) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (!force && now - lastSweep_ < policy_.cleanupInterval) {
            return;
        }
        lastSweep_ = now;

        auto eventDirs = collectEventDirs();

        if (ageEnabled) {
            const auto cutoffNs = systemNowNs() - static_cast<std::int64_t>(policy_.retentionDays) *
                                                      24LL * 3600LL * 1000000000LL;
            // collectEventDirs 已按时间戳升序排序，越过阈值即可停止
            for (auto it = eventDirs.begin();
                 it != eventDirs.end() && it->timestampNs < cutoffNs;) {
                if (deleteDir(*it, "older than retention days")) {
                    it = eventDirs.erase(it);
                } else {
                    ++it;
                }
            }
        }

        if (capacityEnabled) {
            std::uintmax_t totalBytes = 0;
            for (auto& dir : eventDirs) {
                dir.sizeBytes = directorySize(dir.path);
                totalBytes += dir.sizeBytes;
            }
            for (const auto& dir : eventDirs) {
                if (totalBytes <= policy_.maxCapacityBytes) {
                    break;
                }
                if (deleteDir(dir, "record capacity exceeded")) {
                    totalBytes -= std::min(totalBytes, dir.sizeBytes);
                }
            }
        }
    }

private:
    struct EventDir {
        std::filesystem::path path;
        std::int64_t timestampNs;
        std::uintmax_t sizeBytes{0};
    };

    // 收集 recordRoot_ 下形如 <event>_<纳秒时间戳> 的目录，按时间戳升序返回；
    // 命名不匹配的目录（含普通文件）一律不动，避免误删无关数据
    std::vector<EventDir> collectEventDirs() const {
        std::vector<EventDir> dirs;
        std::error_code error;
        for (std::filesystem::directory_iterator it(recordRoot_, error), end; !error && it != end;
             it.increment(error)) {
            std::error_code entryError;
            if (!it->is_directory(entryError) || entryError) {
                continue;
            }
            EventDir dir{it->path(), parseEventTimestamp(it->path().filename().string()), 0};
            if (dir.timestampNs >= 0) {
                dirs.push_back(std::move(dir));
            }
        }
        std::sort(dirs.begin(), dirs.end(), [](const EventDir& left, const EventDir& right) {
            return left.timestampNs < right.timestampNs;
        });
        return dirs;
    }

    // 事件名本身可含 '_'（如 hard_brake），因此取最后一个 '_' 之后的纯数字段；
    // 下界 9 位（纳秒级 epoch）用于排除 <event>_123 之类的无关命名
    static std::int64_t parseEventTimestamp(const std::string& name) {
        const auto separator = name.find_last_of('_');
        if (separator == std::string::npos || separator == 0 || separator + 1 >= name.size()) {
            return -1;
        }
        const auto digits = name.substr(separator + 1);
        if (digits.size() < 9) {
            return -1;
        }
        std::int64_t value = 0;
        for (const char c : digits) {
            if (c < '0' || c > '9') {
                return -1;
            }
            const auto digit = c - '0';
            if (value > (std::numeric_limits<std::int64_t>::max() - digit) / 10) {
                return -1;
            }
            value = value * 10 + digit;
        }
        return value;
    }

    static std::uintmax_t directorySize(const std::filesystem::path& directory) {
        std::uintmax_t total = 0;
        std::error_code iterationError;
        for (std::filesystem::recursive_directory_iterator it(directory, iterationError), end;
             !iterationError && it != end; it.increment(iterationError)) {
            std::error_code error;
            if (it->is_regular_file(error) && !error) {
                const auto size = it->file_size(error);
                if (!error) {
                    total += size;
                }
            }
        }
        return total;
    }

    // 事件目录名中的纳秒时间戳来自节点系统时钟，与 system_clock 对比计算年龄
    static std::int64_t systemNowNs() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }

    std::uintmax_t availableBytes() const {
        std::error_code error;
        auto info = std::filesystem::space(recordRoot_, error);
        if (error) {
            info = std::filesystem::space(".", error); // root 尚未创建时退回当前目录所在卷
        }
        if (error) {
            return std::numeric_limits<std::uintmax_t>::max(); // stat 失败按充足处理，不中断录制
        }
        return info.available;
    }

    bool deleteDir(const EventDir& dir, const char* reason) const {
        std::error_code error;
        std::filesystem::remove_all(dir.path, error);
        if (error) {
            RCLCPP_WARN(logger_, "Failed to delete event directory %s: %s",
                        dir.path.string().c_str(), error.message().c_str());
            return false;
        }
        RCLCPP_INFO(logger_, "Deleted event directory %s (%s)", dir.path.string().c_str(), reason);
        return true;
    }

    static unsigned long long megabytes(std::uintmax_t bytes) {
        return static_cast<unsigned long long>(bytes / (1024ULL * 1024ULL));
    }

    std::filesystem::path recordRoot_;
    Policy policy_;
    rclcpp::Logger logger_;
    std::chrono::steady_clock::time_point lastSweep_{};
};
