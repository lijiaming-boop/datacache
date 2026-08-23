// EventMonitor 单元测试: pre/post 窗口切分、post 调度器的传感器/墙钟两条到期
// 路径、sync_required 拒绝、预留回滚与并发事件上限。
// processExpiredCaptures() 公开为测试缝隙, 不依赖定时器回调被 spin 到。

#include "event_monitor.hpp"
#include "record_io.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

// 构造 Node 需要已初始化的 context
class RosContext : public ::testing::Environment {
public:
    void SetUp() override { rclcpp::init(0, nullptr); }
    void TearDown() override { rclcpp::shutdown(); }
};

[[maybe_unused]] const auto* g_rosContext = ::testing::AddGlobalTestEnvironment(new RosContext);

rclcpp::Time timeAt(std::int64_t nanoseconds) {
    return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

SensorData cameraAt(std::int64_t nanoseconds) {
    auto image = std::make_shared<sensor_msgs::msg::Image>();
    image->header.stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
    image->header.stamp.nanosec =
        static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
    // 字段必须自洽(width/height/step/encoding), 否则 verifyEventDirectory 判失败
    image->height = 2;
    image->width = 3;
    image->encoding = "bgr8";
    image->is_bigendian = 0;
    image->step = 9;
    image->data.resize(18);
    return SensorData{SensorType::CAMERA, CameraData{timeAt(nanoseconds), image}};
}

std::filesystem::path makeTempDir(const std::string& tag) {
    static std::uint64_t counter = 0;
    const auto dir = std::filesystem::temp_directory_path() /
        ("datacache_event_monitor_" + tag + "_" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
         "_" + std::to_string(++counter));
    std::filesystem::create_directories(dir);
    return dir;
}

bool waitFor(const std::function<bool()>& predicate, int timeoutMs) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return predicate();
}

std::vector<std::filesystem::path> listEventDirs(const std::filesystem::path& root) {
    std::vector<std::filesystem::path> dirs;
    std::error_code error;
    for (std::filesystem::directory_iterator it(root, error), end;
         !error && it != end; it.increment(error)) {
        std::error_code entryError;
        if (it->is_directory(entryError) && !entryError) {
            dirs.push_back(it->path());
        }
    }
    return dirs;
}

bool anyCompleteMarker(const std::filesystem::path& root) {
    for (const auto& dir : listEventDirs(root)) {
        if (std::filesystem::exists(dir / ".complete")) {
            return true;
        }
    }
    return false;
}

struct MonitorFixture {
    std::filesystem::path root;
    std::filesystem::path recordsRoot;
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<DataBuffer> buffer;
    std::shared_ptr<ConfigManager> config;
    std::shared_ptr<PairIndex> pairs;
    std::unique_ptr<EventMonitor> monitor;

    MonitorFixture(int preSeconds, int postSeconds, int pendingJobs, int maxActive,
                   int graceMs, bool requireSynced = false)
        : root(makeTempDir("fixture")),
          recordsRoot(root / "records"),
          node(rclcpp::Node::make_shared("event_monitor_test")),
          buffer(std::make_shared<DataBuffer>(1000)),
          config(std::make_shared<ConfigManager>(rclcpp::get_logger("test"))),
          pairs(std::make_shared<PairIndex>()) {
        const auto configFile = root / "config.txt";
        {
            std::ofstream out(configFile);
            out << "record_directory=" << recordsRoot.string() << "\n"
                << "max_pending_storage_jobs=" << pendingJobs << "\n"
                << "max_active_event_captures=" << maxActive << "\n"
                << "event_scheduler_period_ms=10\n"
                << "sensor_stall_grace_ms=" << graceMs << "\n"
                << "event_pre_time=" << preSeconds << "\n"
                << "event_post_time=" << postSeconds << "\n"
                << "record_camera=true\n"
                << "record_lidar=true\n"
                << "compression_enabled=false\n"
                << "conversion_enabled=false\n"
                << "disk_min_free_mb=0\n"
                << "retention_days=0\n"
                << "retention_max_capacity_mb=0\n";
        }
        if (!config->loadConfig(configFile.string())) {
            ADD_FAILURE() << "cannot load test config";
        }
        monitor = std::make_unique<EventMonitor>(
            buffer, config, pairs, rclcpp::get_logger("test"),
            node->get_clock(), node.get(), nullptr, requireSynced);
        monitor->registerEvent("collision",
                               [this] { return monitor->recordDataAroundEvent("collision"); });
    }

    ~MonitorFixture() {
        monitor.reset();
        std::error_code error;
        std::filesystem::remove_all(root, error);
    }
};

TEST(EventMonitorTest, SensorDeadlineCompletesPostWindowWithoutBoundaryDuplicate) {
    MonitorFixture fixture(/*pre=*/1, /*post=*/1, /*pending=*/4, /*active=*/2, /*grace=*/0);

    // 事件时刻 = 缓冲区最新水位 1s; pre 窗口 [0, 1s] 共 3 帧
    fixture.buffer->addData(cameraAt(0));
    fixture.buffer->addData(cameraAt(500'000'000));
    fixture.buffer->addData(cameraAt(1'000'000'000));
    ASSERT_TRUE(fixture.monitor->triggerEvent("collision"));

    // 传感器水位(1s)未越过 endTime(2s), 墙钟也未到 deadline → 不收割
    fixture.monitor->processExpiredCaptures();
    EXPECT_FALSE(anyCompleteMarker(fixture.recordsRoot));

    // post 数据到达, 水位越过 endTime → 收割 post 批次并写 .complete
    fixture.buffer->addData(cameraAt(1'500'000'000));
    fixture.buffer->addData(cameraAt(2'000'000'000));
    fixture.monitor->processExpiredCaptures();
    ASSERT_TRUE(waitFor([&] { return anyCompleteMarker(fixture.recordsRoot); }, 5000))
        << "post window was not persisted in time";

    const auto eventDirs = listEventDirs(fixture.recordsRoot);
    ASSERT_EQ(eventDirs.size(), 1U);
    const auto entries = record_io::readManifest(eventDirs[0]);
    ASSERT_EQ(entries.size(), 5U);

    std::vector<std::int64_t> stamps;
    for (const auto& entry : entries) {
        stamps.push_back(entry.timestampNs);
    }
    std::sort(stamps.begin(), stamps.end());
    const std::vector<std::int64_t> expected{
        0, 500'000'000LL, 1'000'000'000LL, 1'500'000'000LL, 2'000'000'000LL};
    EXPECT_EQ(stamps, expected);
    // eventTime 边界帧(1s)属于 pre 闭区间, post 批次不应重复写入
    EXPECT_EQ(std::count(stamps.begin(), stamps.end(), 1'000'000'000LL), 1);

    const auto report = record_io::verifyEventDirectory(eventDirs[0]);
    EXPECT_TRUE(report.ok()) << (report.problems.empty() ? "" : report.problems[0]);
}

TEST(EventMonitorTest, WallDeadlineExpiresCaptureWhenSensorStalls) {
    MonitorFixture fixture(/*pre=*/0, /*post=*/1, /*pending=*/4, /*active=*/2, /*grace=*/0);

    fixture.buffer->addData(cameraAt(1'000'000'000));
    ASSERT_TRUE(fixture.monitor->triggerEvent("collision"));

    // 传感器停转: endTime(2s)永远等不到, 只有墙钟 deadline(触发后 1s)兜底
    fixture.monitor->processExpiredCaptures();
    EXPECT_FALSE(anyCompleteMarker(fixture.recordsRoot));

    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    fixture.monitor->processExpiredCaptures();
    ASSERT_TRUE(waitFor([&] { return anyCompleteMarker(fixture.recordsRoot); }, 5000))
        << "wall deadline did not flush the stalled capture";
}

TEST(EventMonitorTest, RejectsEventWithoutSyncedPairWhenRequired) {
    MonitorFixture fixture(/*pre=*/1, /*post=*/1, /*pending=*/4, /*active=*/2,
                           /*grace=*/0, /*requireSynced=*/true);

    fixture.buffer->addData(cameraAt(1'000'000'000));  // 无任何配对账本记录
    EXPECT_FALSE(fixture.monitor->triggerEvent("collision"));
    EXPECT_TRUE(listEventDirs(fixture.recordsRoot).empty());
}

TEST(EventMonitorTest, RollsBackReservationWhenPreEnqueueFails) {
    // max_pending_storage_jobs=1: reserve 占掉唯一槽位后, pre 批次必然入队失败,
    // 事件应整体回滚(不留捕获任务、不留半目录)
    MonitorFixture fixture(/*pre=*/1, /*post=*/1, /*pending=*/1, /*active=*/2, /*grace=*/0);

    fixture.buffer->addData(cameraAt(1'000'000'000));
    EXPECT_FALSE(fixture.monitor->triggerEvent("collision"));

    // 回滚后不应残留捕获任务: 传感器越过 endTime 也不该有任何落盘
    fixture.buffer->addData(cameraAt(3'000'000'000));
    fixture.monitor->processExpiredCaptures();
    EXPECT_FALSE(waitFor([&] { return anyCompleteMarker(fixture.recordsRoot); }, 500));
    EXPECT_TRUE(listEventDirs(fixture.recordsRoot).empty());
}

TEST(EventMonitorTest, MaxActiveCapturesRejectsExcessEvents) {
    // post=10s(传感器时间) 远未到期, 前两个事件保持活跃占满上限
    MonitorFixture fixture(/*pre=*/1, /*post=*/10, /*pending=*/8, /*active=*/2, /*grace=*/0);

    fixture.buffer->addData(cameraAt(1'000'000'000));
    EXPECT_TRUE(fixture.monitor->triggerEvent("collision"));
    EXPECT_TRUE(fixture.monitor->triggerEvent("collision"));
    EXPECT_FALSE(fixture.monitor->triggerEvent("collision"));
}

}  // namespace
