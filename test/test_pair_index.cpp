#include "pair_index.hpp"

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

namespace {

rclcpp::Time timeAt(std::int64_t nanoseconds) {
    // 统一用 RCL_ROS_TIME, 与生产代码从 header.stamp 构造的时间源一致
    return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

TEST(PairIndexTest, MatchedRecordsCarryBothTimestamps) {
    PairIndex index;
    index.addMatched(timeAt(1000), timeAt(1'010'000'000),
                     rclcpp::Duration::from_nanoseconds(10'000'000));

    const auto records = index.getDataWithinTimeRange(timeAt(0), timeAt(2'000'000'000));
    ASSERT_EQ(records.size(), 1U);
    EXPECT_EQ(records[0].status, "matched");
    EXPECT_TRUE(records[0].hasCamera);
    EXPECT_TRUE(records[0].hasLidar);
    EXPECT_EQ(records[0].cameraTimestamp, timeAt(1000));
    EXPECT_EQ(records[0].lidarTimestamp, timeAt(1'010'000'000));
    EXPECT_EQ(records[0].difference.nanoseconds(), 10'000'000);
    EXPECT_EQ(records[0].pairId, 1U);
}

TEST(PairIndexTest, SingleSidedRecordsSeparateCameraAndLidar) {
    PairIndex index;
    index.addSingle("camera", timeAt(500), "synchronizer queue full");
    index.addSingle("lidar", timeAt(600), "outside synchronization tolerance");

    const auto records = index.getDataWithinTimeRange(timeAt(0), timeAt(1000));
    ASSERT_EQ(records.size(), 2U);

    EXPECT_EQ(records[0].status, "camera_only");
    EXPECT_TRUE(records[0].hasCamera);
    EXPECT_FALSE(records[0].hasLidar);
    EXPECT_EQ(records[0].reason, "synchronizer queue full");

    EXPECT_EQ(records[1].status, "lidar_only");
    EXPECT_FALSE(records[1].hasCamera);
    EXPECT_TRUE(records[1].hasLidar);
    EXPECT_EQ(records[1].reason, "outside synchronization tolerance");
}

TEST(PairIndexTest, TimeRangeFiltersOnAvailableTimestamp) {
    PairIndex index;
    index.addMatched(timeAt(1000), timeAt(1000), rclcpp::Duration(0, 0));
    index.addSingle("camera", timeAt(5000), "event window closed");

    EXPECT_EQ(index.getDataWithinTimeRange(timeAt(0), timeAt(1000)).size(), 1U);
    EXPECT_EQ(index.getDataWithinTimeRange(timeAt(5000), timeAt(6000)).size(), 1U);
    EXPECT_EQ(index.getDataWithinTimeRange(timeAt(2000), timeAt(4000)).size(), 0U);
}

TEST(PairIndexTest, TrimKeepsNewestHundredThousand) {
    PairIndex index;
    for (int i = 0; i < 100'050; ++i) {
        index.addSingle("camera", timeAt(i), "bulk");
    }
    const auto records = index.getDataWithinTimeRange(timeAt(0), timeAt(1'000'000));
    ASSERT_EQ(records.size(), 100'000U);
    // 淘汰最旧: 剩余 50..100049
    EXPECT_EQ(records.front().cameraTimestamp, timeAt(50));
    EXPECT_EQ(records.back().cameraTimestamp, timeAt(100'049));
}

} // namespace
