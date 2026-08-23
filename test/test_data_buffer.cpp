#include "databuffer.hpp"

#include <gtest/gtest.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace {

rclcpp::Time timeAt(std::int64_t nanoseconds) {
    // 统一用 RCL_ROS_TIME, 与生产代码从 header.stamp 构造的时间源一致
    return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

SensorData cameraAt(std::int64_t nanoseconds) {
    auto image = std::make_shared<sensor_msgs::msg::Image>();
    image->header.stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
    image->header.stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
    return SensorData{SensorType::CAMERA, CameraData{timeAt(nanoseconds), image}};
}

SensorData lidarAt(std::int64_t nanoseconds) {
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->header.stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
    cloud->header.stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
    return SensorData{SensorType::LIDAR, LidarData{timeAt(nanoseconds), cloud}};
}

TEST(DataBufferTest, CountCapAppliesPerSensorType) {
    DataBuffer buffer(3);
    for (int i = 0; i < 5; ++i) {
        buffer.addData(cameraAt(i * 1000));
    }
    // 相机被限制在各自最新的 3 条, 不挤占雷达容量
    for (int i = 0; i < 5; ++i) {
        buffer.addData(lidarAt(i * 1000));
    }
    const auto all = buffer.getDataWithinTimeRange(timeAt(0), timeAt(100000));
    EXPECT_EQ(all.size(), 6U);

    // 保留的是时间戳 2/3/4 的各 3 条; 查询不区分传感器类型, t=2000 处两类各一条
    const auto atTwoThousand = buffer.getDataWithinTimeRange(timeAt(2000), timeAt(2000));
    ASSERT_EQ(atTwoThousand.size(), 2U);
    EXPECT_EQ(atTwoThousand[0].type, SensorType::CAMERA);
    EXPECT_EQ(atTwoThousand[1].type, SensorType::LIDAR);
    const auto oldestKept = buffer.getDataWithinTimeRange(timeAt(0), timeAt(1000));
    EXPECT_EQ(oldestKept.size(), 0U);
}

TEST(DataBufferTest, AgeEvictionUsesGlobalNewestWatermark) {
    DataBuffer buffer(100, rclcpp::Duration::from_seconds(10));
    buffer.addData(cameraAt(0));
    buffer.addData(cameraAt(5'000'000'000));
    EXPECT_EQ(buffer.getDataWithinTimeRange(timeAt(0), timeAt(10'000'000'000)).size(), 2U);

    // 雷达时间戳跳到 100 s, 相机数据全部早于水位(100-10 s)被老化
    buffer.addData(lidarAt(100'000'000'000));
    const auto remaining = buffer.getDataWithinTimeRange(timeAt(0), timeAt(200'000'000'000));
    ASSERT_EQ(remaining.size(), 1U);
    EXPECT_EQ(remaining[0].type, SensorType::LIDAR);
}

TEST(DataBufferTest, TimeRangeQueryFiltersBothEnds) {
    DataBuffer buffer(100);
    buffer.addData(cameraAt(1000));
    buffer.addData(cameraAt(2000));
    buffer.addData(lidarAt(3000));

    const auto both = buffer.getDataWithinTimeRange(timeAt(1000), timeAt(2000));
    EXPECT_EQ(both.size(), 2U);
    // (1001, 1999) 开区间内无任何数据; 2000 在闭区间边界上被包含
    const auto exclusive = buffer.getDataWithinTimeRange(timeAt(1001), timeAt(1999));
    EXPECT_EQ(exclusive.size(), 0U);
}

TEST(DataBufferTest, LatestSensorTimestampTracksNewest) {
    DataBuffer buffer(10);
    EXPECT_FALSE(buffer.latestSensorTimestamp().has_value());

    buffer.addData(cameraAt(1000));
    buffer.addData(lidarAt(2000));
    ASSERT_TRUE(buffer.latestSensorTimestamp().has_value());
    EXPECT_EQ(*buffer.latestSensorTimestamp(), timeAt(2000));

    // 更早的时间戳不会回退水位
    buffer.addData(cameraAt(500));
    EXPECT_EQ(*buffer.latestSensorTimestamp(), timeAt(2000));
}

} // namespace
