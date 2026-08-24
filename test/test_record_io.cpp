#include "record_io.hpp"
#include "raw_storage_worker.hpp"
#include "pair_index.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace {

std::filesystem::path makeTempDir(const std::string& tag) {
    static std::uint64_t counter = 0;
    const auto dir = std::filesystem::temp_directory_path() /
                     ("datacache_record_io_" + tag + "_" +
                      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
                      "_" + std::to_string(++counter));
    std::filesystem::create_directories(dir);
    return dir;
}

bool waitFor(const std::function<bool()>& predicate, int timeoutMs) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return predicate();
}

void stampHeader(std_msgs::msg::Header& header, std::int64_t nanoseconds) {
    header.stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
    header.stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
}

rclcpp::Time rosTimeAt(std::int64_t nanoseconds) {
    // RCL_ROS_TIME 与 header.stamp 构造的时间源一致; 字面量 0 直接传会命中
    // Time(int32_t, uint32_t) 重载产生歧义, 因此统一走 int64_t
    return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

sensor_msgs::msg::Image::SharedPtr makeImage(std::int64_t nanoseconds) {
    auto image = std::make_shared<sensor_msgs::msg::Image>();
    stampHeader(image->header, nanoseconds);
    image->height = 2;
    image->width = 3;
    image->encoding = "bgr8";
    image->is_bigendian = 0;
    image->step = 9;
    image->data.resize(18);
    for (std::size_t i = 0; i < image->data.size(); ++i) {
        image->data[i] = static_cast<std::uint8_t>((i * 17 + 3) & 0xFF);
    }
    return image;
}

sensor_msgs::msg::PointCloud2::SharedPtr makeCloud(std::int64_t nanoseconds) {
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    stampHeader(cloud->header, nanoseconds);
    cloud->height = 1;
    cloud->width = 4;
    cloud->point_step = 12; // x/y/z float32
    for (const char* field : {"x", "y", "z"}) {
        sensor_msgs::msg::PointField pointField;
        pointField.name = field;
        pointField.offset = static_cast<std::uint32_t>((field[0] - 'x') * 4);
        pointField.datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointField.count = 1;
        cloud->fields.push_back(pointField);
    }
    cloud->is_bigendian = false;
    cloud->is_dense = true;
    cloud->row_step = cloud->width * cloud->point_step;
    cloud->data.resize(cloud->row_step);
    for (std::size_t i = 0; i < cloud->data.size(); ++i) {
        cloud->data[i] = static_cast<std::uint8_t>(i & 0xFF);
    }
    return cloud;
}

struct Fixture {
    std::filesystem::path root;
    std::filesystem::path eventDir;
    sensor_msgs::msg::Image::SharedPtr image;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;

    Fixture()
        : root(makeTempDir("fixture")), eventDir(root / "collision_1700000000000000000"),
          image(makeImage(1'700'000'000'000'000'000LL)),
          cloud(makeCloud(1'700'000'000'010'000'000LL)) {}

    ~Fixture() {
        std::error_code error;
        std::filesystem::remove_all(root, error);
    }

    // 通过 RawStorageWorker 落盘并等待 .complete 标记
    void write(const std::vector<PairRecord>& pairs = {}) {
        RawStorageWorker worker(rclcpp::get_logger("test"), 4);
        std::vector<SensorData> records;
        records.push_back(
            SensorData{SensorType::CAMERA, CameraData{rclcpp::Time(image->header.stamp), image}});
        records.push_back(
            SensorData{SensorType::LIDAR, LidarData{rclcpp::Time(cloud->header.stamp), cloud}});
        ASSERT_TRUE(worker.enqueue(eventDir, std::move(records), true, true, true /*compression*/,
                                   3, false, false /*conversion*/, "jpg", 90, "pcd", pairs, false,
                                   true));
        ASSERT_TRUE(
            waitFor([this] { return std::filesystem::exists(eventDir / ".complete"); }, 5000))
            << "storage worker did not finish in time";
    }
};

TEST(RecordIoTest, WriteReadRoundtrip) {
    Fixture fixture;
    PairIndex index;
    index.addMatched(rclcpp::Time(fixture.image->header.stamp),
                     rclcpp::Time(fixture.cloud->header.stamp),
                     rclcpp::Duration::from_nanoseconds(10'000'000));
    // 查询边界与账本时间同为 RCL_ROS_TIME(来自 header.stamp 的默认时间源)
    fixture.write(
        index.getDataWithinTimeRange(rosTimeAt(0), rosTimeAt(2'000'000'000'000'000'000LL)));

    // manifest 结构
    const auto entries = record_io::readManifest(fixture.eventDir);
    ASSERT_EQ(entries.size(), 2U);
    EXPECT_EQ(entries[0].sensor, "camera");
    EXPECT_EQ(entries[1].sensor, "lidar");
    EXPECT_EQ(entries[0].encoding, "zstd");
    ASSERT_GT(entries[0].file.size(), 4U);
    EXPECT_EQ(entries[0].file.compare(entries[0].file.size() - 4, 4, ".zst"), 0);

    // pairs.csv 账本
    const auto pairs = record_io::readPairs(fixture.eventDir);
    ASSERT_EQ(pairs.size(), 1U);
    EXPECT_EQ(pairs[0].status, "matched");
    EXPECT_EQ(pairs[0].timeDiffNs, 10'000'000);

    // 完整性校验通过
    const auto report = record_io::verifyEventDirectory(fixture.eventDir);
    EXPECT_TRUE(report.ok()) << (report.problems.empty() ? "" : report.problems[0]);
    EXPECT_EQ(report.verifiedEntries, 2U);

    // 相机字节级还原
    std::vector<std::uint8_t> bytes;
    std::string error;
    ASSERT_TRUE(record_io::loadRecordBytes(fixture.eventDir, entries[0], bytes, error)) << error;
    sensor_msgs::msg::Image restored;
    ASSERT_TRUE(record_io::deserializeMessage(bytes, restored, error)) << error;
    EXPECT_EQ(restored.width, fixture.image->width);
    EXPECT_EQ(restored.height, fixture.image->height);
    EXPECT_EQ(restored.encoding, fixture.image->encoding);
    EXPECT_EQ(restored.data, fixture.image->data);

    // 雷达字节级还原
    ASSERT_TRUE(record_io::loadRecordBytes(fixture.eventDir, entries[1], bytes, error)) << error;
    sensor_msgs::msg::PointCloud2 restoredCloud;
    ASSERT_TRUE(record_io::deserializeMessage(bytes, restoredCloud, error)) << error;
    EXPECT_EQ(restoredCloud.width, fixture.cloud->width);
    EXPECT_EQ(restoredCloud.fields.size(), 3U);
    EXPECT_EQ(restoredCloud.data, fixture.cloud->data);
}

TEST(RecordIoTest, VerifyDetectsCorruption) {
    Fixture fixture;
    fixture.write();

    ASSERT_TRUE(record_io::verifyEventDirectory(fixture.eventDir).ok());

    // 破坏一条压缩记录
    const auto entries = record_io::readManifest(fixture.eventDir);
    std::ofstream corrupt(fixture.eventDir / entries[0].file, std::ios::binary | std::ios::trunc);
    corrupt << "not a zstd frame at all";
    corrupt.close();

    const auto report = record_io::verifyEventDirectory(fixture.eventDir);
    EXPECT_FALSE(report.ok());
    EXPECT_EQ(report.failedEntries, 1U);
    EXPECT_EQ(report.verifiedEntries, 1U);
    EXPECT_GE(report.problems.size(), 2U); // SHA-256 mismatch + record decode failure
}

TEST(RecordIoTest, VerifyHandlesMissingManifestAndSensorFilter) {
    const auto empty = makeTempDir("empty");

    record_io::VerificationReport report;
    record_io::VerifyOptions options;
    options.sensor = "camera";
    report = record_io::verifyEventDirectory(empty, options);
    EXPECT_FALSE(report.ok());
    EXPECT_EQ(report.totalEntries, 0U);

    Fixture fixture;
    fixture.write();
    options.sensor = "lidar";
    const auto lidarOnly = record_io::verifyEventDirectory(fixture.eventDir, options);
    EXPECT_TRUE(lidarOnly.ok());
    EXPECT_EQ(lidarOnly.totalEntries, 1U);

    std::error_code error;
    std::filesystem::remove_all(empty, error);
}

TEST(RecordIoTest, TornManifestLinesAreReportedNotThrown) {
    Fixture fixture;
    fixture.write();

    // 追加一行撕裂的 manifest 行(崩溃残留): 解析不抛异常, 计入 problems,
    // 且 --verify 语义下(verifyEventDirectory)整体判定为失败
    {
        std::ofstream append(fixture.eventDir / "manifest.csv", std::ios::app);
        append << "camera,170000000"; // 撕裂: 字段不足
        append.flush();
    }
    std::vector<std::string> problems;
    const auto entries = record_io::readManifest(fixture.eventDir, &problems);
    EXPECT_EQ(entries.size(), 2U);
    ASSERT_EQ(problems.size(), 1U);
    EXPECT_NE(problems[0].find("manifest.csv line"), std::string::npos);

    const auto report = record_io::verifyEventDirectory(fixture.eventDir);
    EXPECT_FALSE(report.ok());
    EXPECT_EQ(report.failedEntries, 1U);
}

TEST(RecordIoTest, TornPairsLinesAreReportedNotThrown) {
    const auto dir = makeTempDir("torn_pairs");
    {
        std::ofstream output(dir / "pairs.csv", std::ios::trunc);
        output << "pair_id,status,camera_timestamp,lidar_timestamp,time_diff_ns,reason\n";
        output << "1,matched,1700000000000000000,1700000000100000000,10000000,\n";
        output << "2,matched,not-a-number,,,"; // 撕裂: 时间戳字段损坏
        output.flush();
    }
    std::vector<std::string> problems;
    const auto pairs = record_io::readPairs(dir, &problems);
    EXPECT_EQ(pairs.size(), 1U);
    ASSERT_EQ(problems.size(), 1U);
    EXPECT_NE(problems[0].find("pairs.csv line"), std::string::npos);

    std::error_code error;
    std::filesystem::remove_all(dir, error);
}

} // namespace
