#pragma once

#include "data.hpp"

#include <cstddef>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

// 按条数 + 年龄约束的传感器环形缓存。
// 每类传感器一条独立 deque, 到达序 ≈ 时间戳序, 因此计数淘汰与年龄淘汰都是
// 队首弹出: 单条插入摊还 O(1), 不随缓存规模增长。
class DataBuffer {
public:
    DataBuffer(std::size_t maxSize, rclcpp::Duration maxAge = rclcpp::Duration::from_seconds(0))
        : maxSize_(maxSize), maxAge_(maxAge) {}

    void addData(SensorData data) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (maxSize_ == 0) {
            return;
        }
        const auto timestamp = timestampOf(data);

        if (!hasLatestTimestamp_ || timestamp > latestTimestamp_) {
            latestTimestamp_ = timestamp;
            hasLatestTimestamp_ = true;
        }
        // 晚到超过年龄上限的数据不入队: 水位推进时它必然被淘汰, 入队只是浪费
        if (maxAge_.nanoseconds() > 0 && timestamp < latestTimestamp_ - maxAge_) {
            return;
        }
        auto& queue = queues_[data.type];
        queue.push_back(std::move(data));
        while (queue.size() > maxSize_) {
            queue.pop_front();
        }
        evictExpiredLocked();
    }

    std::vector<SensorData> getDataWithinTimeRange(const rclcpp::Time& start,
                                                   const rclcpp::Time& end) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<SensorData> result;
        for (const auto& entry : queues_) {
            for (const auto& data : entry.second) {
                const auto timestamp = timestampOf(data);
                if (timestamp >= start && timestamp <= end) {
                    result.push_back(data);
                }
            }
        }
        return result;
    }

    std::vector<SensorData> getAllData() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<SensorData> result;
        for (const auto& entry : queues_) {
            result.insert(result.end(), entry.second.begin(), entry.second.end());
        }
        return result;
    }

    // Newest sensor header.stamp across all buffered data; event windows are
    // resolved against this so their bounds stay in the sensor time domain.
    std::optional<rclcpp::Time> latestSensorTimestamp() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!hasLatestTimestamp_) {
            return std::nullopt;
        }
        return latestTimestamp_;
    }

private:
    // 各队列按各自到达序排列, 队首即最老; 水位(全局最新时间戳 - maxAge)之前的
    // 数据从队首连续弹出。少数乱序到达且落后超过年龄上限的帧在入队时已被拒绝,
    // 因此队首弹出即可覆盖全部过期数据。
    void evictExpiredLocked() {
        if (maxAge_.nanoseconds() <= 0 || !hasLatestTimestamp_) {
            return;
        }
        const auto oldestAllowed = latestTimestamp_ - maxAge_;
        for (auto& entry : queues_) {
            auto& queue = entry.second;
            while (!queue.empty() && timestampOf(queue.front()) < oldestAllowed) {
                queue.pop_front();
            }
        }
    }

    static rclcpp::Time timestampOf(const SensorData& data) {
        return std::visit([](const auto& value) { return value.timestamp; }, data.data);
    }

    mutable std::mutex mutex_;
    std::map<SensorType, std::deque<SensorData>> queues_;
    std::size_t maxSize_;
    rclcpp::Duration maxAge_;
    rclcpp::Time latestTimestamp_;
    bool hasLatestTimestamp_{false};
};
