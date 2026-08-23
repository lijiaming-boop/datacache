#include "approximate_synchronizer.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace {

using Image = sensor_msgs::msg::Image;
using PointCloud = sensor_msgs::msg::PointCloud2;

struct MatchLog {
    std::int64_t imageNs;
    std::int64_t cloudNs;
    std::int64_t diffNs;
};

struct DropLog {
    std::string sensor;
    std::int64_t timestampNs;
    std::uint64_t count;
    std::string reason;
};

rclcpp::Time timeAt(std::int64_t nanoseconds) {
    // 统一用 RCL_ROS_TIME, 与同步器内部从 header.stamp 构造的时间源一致
    return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

Image::SharedPtr imageAt(std::int64_t nanoseconds) {
    auto image = std::make_shared<Image>();
    image->header.stamp.sec =
        static_cast<std::int32_t>(nanoseconds / 1000000000LL);
    image->header.stamp.nanosec =
        static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
    return image;
}

PointCloud::SharedPtr cloudAt(std::int64_t nanoseconds) {
    auto cloud = std::make_shared<PointCloud>();
    cloud->header.stamp.sec =
        static_cast<std::int32_t>(nanoseconds / 1000000000LL);
    cloud->header.stamp.nanosec =
        static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
    return cloud;
}

class SynchronizerHarness {
public:
    SynchronizerHarness(std::size_t queueSize, std::int64_t toleranceMs)
        : synchronizer_(
              queueSize, rclcpp::Duration::from_nanoseconds(toleranceMs * 1'000'000),
              [this](const Image::SharedPtr& image, const PointCloud::SharedPtr& cloud,
                     rclcpp::Duration difference) {
                  matches_.push_back(MatchLog{
                      rclcpp::Time(image->header.stamp).nanoseconds(),
                      rclcpp::Time(cloud->header.stamp).nanoseconds(),
                      difference.nanoseconds()});
              },
              [this](const std::string& sensor, const rclcpp::Time& timestamp,
                     std::uint64_t count, const std::string& reason) {
                  drops_.push_back(DropLog{
                      sensor, timestamp.nanoseconds(), count, reason});
              }) {}

    ApproximateSynchronizer synchronizer_;
    std::vector<MatchLog> matches_;
    std::vector<DropLog> drops_;
};

TEST(ApproximateSynchronizerTest, MatchesWithinTolerance) {
    SynchronizerHarness harness(10, 20);
    harness.synchronizer_.addImage(imageAt(100'000'000));
    harness.synchronizer_.addPointCloud(cloudAt(110'000'000));
    // 容差边界上的第二对
    harness.synchronizer_.addImage(imageAt(200'000'000));
    harness.synchronizer_.addPointCloud(cloudAt(220'000'000));

    ASSERT_EQ(harness.matches_.size(), 2U);
    EXPECT_EQ(harness.matches_[0].imageNs, 100'000'000);
    EXPECT_EQ(harness.matches_[0].cloudNs, 110'000'000);
    EXPECT_EQ(harness.matches_[0].diffNs, 10'000'000);
    EXPECT_EQ(harness.matches_[1].diffNs, 20'000'000);
    EXPECT_TRUE(harness.drops_.empty());
}

TEST(ApproximateSynchronizerTest, DropsMessageOutsideTolerance) {
    SynchronizerHarness harness(10, 20);
    harness.synchronizer_.addImage(imageAt(100'000'000));
    // 雷达晚了 100 ms: 图像落后无法再配对, 应作为超容差丢弃
    harness.synchronizer_.addPointCloud(cloudAt(200'000'000));
    // 随后的图像与残留雷达差 200 ms, 雷达也被丢弃
    harness.synchronizer_.addImage(imageAt(400'000'000));

    EXPECT_TRUE(harness.matches_.empty());
    ASSERT_EQ(harness.drops_.size(), 2U);
    EXPECT_EQ(harness.drops_[0].sensor, "camera");
    EXPECT_EQ(harness.drops_[0].timestampNs, 100'000'000);
    EXPECT_EQ(harness.drops_[0].reason, "outside synchronization tolerance");
    EXPECT_EQ(harness.drops_[0].count, 1U);
    EXPECT_EQ(harness.drops_[1].sensor, "lidar");
    EXPECT_EQ(harness.drops_[1].timestampNs, 200'000'000);
    EXPECT_EQ(harness.drops_[1].count, 1U);
}

TEST(ApproximateSynchronizerTest, QueueFullDropsOldest) {
    SynchronizerHarness harness(2, 20);
    harness.synchronizer_.addImage(imageAt(100'000'000));
    harness.synchronizer_.addImage(imageAt(200'000'000));
    harness.synchronizer_.addImage(imageAt(300'000'000));
    // 第三张图像触发队满丢弃最旧的 100ms; 雷达到达后 200ms 图像超容差再被丢弃,
    // 300ms 图像与雷达成功配对
    harness.synchronizer_.addPointCloud(cloudAt(300'000'000));

    ASSERT_EQ(harness.matches_.size(), 1U);
    EXPECT_EQ(harness.matches_[0].imageNs, 300'000'000);
    ASSERT_EQ(harness.drops_.size(), 2U);
    EXPECT_EQ(harness.drops_[0].sensor, "camera");
    EXPECT_EQ(harness.drops_[0].timestampNs, 100'000'000);
    EXPECT_EQ(harness.drops_[0].reason, "synchronizer queue full");
    EXPECT_EQ(harness.drops_[0].count, 1U);
    EXPECT_EQ(harness.drops_[1].sensor, "camera");
    EXPECT_EQ(harness.drops_[1].timestampNs, 200'000'000);
    EXPECT_EQ(harness.drops_[1].reason, "outside synchronization tolerance");
    EXPECT_EQ(harness.drops_[1].count, 2U);  // 每传感器累计计数
}

TEST(ApproximateSynchronizerTest, FlushUnmatchedReportsBothSides) {
    SynchronizerHarness harness(10, 20);
    harness.synchronizer_.addImage(imageAt(100'000'000));
    harness.synchronizer_.addPointCloud(cloudAt(900'000'000));

    harness.synchronizer_.flushUnmatched("event window closed");
    // flush 之前, 图像先因超容差被丢弃, 雷达留队等待
    ASSERT_GE(harness.drops_.size(), 2U);
    EXPECT_EQ(harness.drops_[0].reason, "outside synchronization tolerance");
    EXPECT_EQ(harness.drops_.back().sensor, "lidar");
    EXPECT_EQ(harness.drops_.back().reason, "event window closed");
    EXPECT_TRUE(harness.matches_.empty());

    // flush 后队列清空
    SynchronizerHarness flushed(10, 20);
    flushed.synchronizer_.addImage(imageAt(50));
    flushed.synchronizer_.flushUnmatched();
    ASSERT_EQ(flushed.drops_.size(), 1U);
    EXPECT_EQ(flushed.drops_[0].sensor, "camera");
    EXPECT_EQ(flushed.drops_[0].reason, "event window closed");
}

TEST(ApproximateSynchronizerTest, SingleSideAloneNeverMatchesOrDrops) {
    SynchronizerHarness harness(5, 20);
    for (int i = 0; i < 5; ++i) {
        harness.synchronizer_.addImage(imageAt(i * 100'000'000));
    }
    EXPECT_TRUE(harness.matches_.empty());
    EXPECT_TRUE(harness.drops_.empty());
}

}  // namespace
