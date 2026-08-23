// DiskSpaceManager 单元测试: 按天数清理、按容量从最旧开始清理、写前空间
// 检查、非 force 调用节流、低空间强制清理。
// (移植自 build/verify_disk 下未纳入版本管理的本地验证程序)

#include "disk_space_manager.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

void writeFile(const fs::path& path, std::uintmax_t bytes) {
    fs::create_directories(path.parent_path());
    std::ofstream file(path, std::ios::binary);
    std::vector<char> data(1024, 'x');
    while (bytes > 0) {
        const auto chunk = std::min<std::uintmax_t>(bytes, data.size());
        file.write(data.data(), static_cast<std::streamsize>(chunk));
        bytes -= chunk;
    }
}

std::string eventDirName(double daysAgo) {
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    return std::to_string(now - static_cast<std::int64_t>(daysAgo * 86400.0 * 1e9));
}

struct RootFixture {
    fs::path root;

    RootFixture() : root(fs::temp_directory_path() /
                         ("datacache_disk_" + std::to_string(
                              std::chrono::steady_clock::now().time_since_epoch().count()))) {
        std::error_code error;
        fs::remove_all(root, error);
        fs::create_directories(root);
    }

    ~RootFixture() {
        std::error_code error;
        fs::remove_all(root, error);
    }
};

}  // namespace

TEST(DiskSpaceManagerTest, DeletesDirectoriesOlderThanRetentionDays) {
    RootFixture fixture;
    const auto oldDir = "collision_" + eventDirName(3.0);
    const auto recentDir = "hard_brake_" + eventDirName(0.01);
    fs::create_directories(fixture.root / oldDir);
    fs::create_directories(fixture.root / recentDir);
    writeFile(fixture.root / "notes.txt", 16);
    fs::create_directories(fixture.root / "unrelated_dir");
    fs::create_directories(fixture.root / "_12345678901234");  // 空事件名, 不匹配
    fs::create_directories(fixture.root / "foo_123");          // 数字段过短, 不匹配

    DiskSpaceManager::Policy policy;
    policy.retentionDays = 1;
    policy.cleanupInterval = std::chrono::seconds(0);
    DiskSpaceManager manager(fixture.root, policy, rclcpp::get_logger("test"));
    manager.enforceRetention(true);

    EXPECT_FALSE(fs::exists(fixture.root / oldDir));
    EXPECT_TRUE(fs::exists(fixture.root / recentDir));
    // 命名不匹配的目录与普通文件一律不动
    EXPECT_TRUE(fs::exists(fixture.root / "unrelated_dir"));
    EXPECT_TRUE(fs::exists(fixture.root / "notes.txt"));
    EXPECT_TRUE(fs::exists(fixture.root / "_12345678901234"));
    EXPECT_TRUE(fs::exists(fixture.root / "foo_123"));
}

TEST(DiskSpaceManagerTest, CapacityCleanupDeletesOldestFirst) {
    RootFixture fixture;
    const auto dirA = "collision_" + eventDirName(10.0);
    const auto dirB = "collision_" + eventDirName(5.0);
    const auto dirC = "collision_" + eventDirName(0.0);
    writeFile(fixture.root / dirA / "data.bin", 2048);
    writeFile(fixture.root / dirB / "data.bin", 2048);
    writeFile(fixture.root / dirC / "data.bin", 2048);

    DiskSpaceManager::Policy policy;
    policy.maxCapacityBytes = 5120;  // 总量 6144, 删最旧的 A 后剩 4096 <= 5120
    policy.cleanupInterval = std::chrono::seconds(0);
    DiskSpaceManager manager(fixture.root, policy, rclcpp::get_logger("test"));
    manager.enforceRetention(true);

    EXPECT_FALSE(fs::exists(fixture.root / dirA));
    EXPECT_TRUE(fs::exists(fixture.root / dirB));
    EXPECT_TRUE(fs::exists(fixture.root / dirC));
}

TEST(DiskSpaceManagerTest, PrepareForWriteChecksMinimumFreeSpace) {
    RootFixture fixture;
    fs::create_directories(fixture.root / ("collision_" + eventDirName(0.0)));

    {
        DiskSpaceManager::Policy policy;  // 全部禁用 → 恒通过
        DiskSpaceManager manager(fixture.root, policy, rclcpp::get_logger("test"));
        EXPECT_TRUE(manager.prepareForWrite());
    }
    {
        DiskSpaceManager::Policy policy;
        policy.minFreeBytes = 1;  // 1 字节阈值, 正常磁盘必然通过
        DiskSpaceManager manager(fixture.root, policy, rclcpp::get_logger("test"));
        EXPECT_TRUE(manager.prepareForWrite());
    }
    {
        DiskSpaceManager::Policy policy;
        policy.minFreeBytes = std::numeric_limits<std::uintmax_t>::max() / 2;  // 不可能满足
        DiskSpaceManager manager(fixture.root, policy, rclcpp::get_logger("test"));
        EXPECT_FALSE(manager.prepareForWrite());
    }
}

TEST(DiskSpaceManagerTest, NonForcedSweepIsThrottled) {
    RootFixture fixture;
    const auto stale1 = "collision_" + eventDirName(2.0);
    const auto stale2 = "collision_" + eventDirName(1.5);
    fs::create_directories(fixture.root / stale1);
    fs::create_directories(fixture.root / stale2);

    DiskSpaceManager::Policy policy;
    policy.retentionDays = 1;
    policy.cleanupInterval = std::chrono::seconds(3600);  // 一小时内只扫一次
    DiskSpaceManager manager(fixture.root, policy, rclcpp::get_logger("test"));

    manager.enforceRetention();  // 首次扫描: stale1、stale2 都被删
    EXPECT_FALSE(fs::exists(fixture.root / stale1));
    fs::create_directories(fixture.root / stale2);

    manager.enforceRetention();  // 被节流: stale2 应存活
    EXPECT_TRUE(fs::exists(fixture.root / stale2));

    manager.enforceRetention(true);  // 强制: stale2 被删
    EXPECT_FALSE(fs::exists(fixture.root / stale2));
}

TEST(DiskSpaceManagerTest, LowSpaceForcesCleanupBeforeRefusing) {
    RootFixture fixture;
    const auto stale = "collision_" + eventDirName(2.0);
    fs::create_directories(fixture.root / stale);

    DiskSpaceManager::Policy policy;
    policy.minFreeBytes = std::numeric_limits<std::uintmax_t>::max() / 2;
    policy.retentionDays = 1;
    policy.cleanupInterval = std::chrono::seconds(3600);
    DiskSpaceManager manager(fixture.root, policy, rclcpp::get_logger("test"));

    EXPECT_FALSE(manager.prepareForWrite());
    EXPECT_FALSE(fs::exists(fixture.root / stale));  // 拒绝前先强制执行天数清理
}
