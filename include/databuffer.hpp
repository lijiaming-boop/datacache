#pragma once

#include "data.hpp"

#include <algorithm>
#include <cstddef>
#include <deque>
#include <limits>
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
    DataBuffer(std::size_t maxSize, rclcpp::Duration maxAge = rclcpp::Duration::from_seconds(0),
               std::size_t maxBytesPerSensor = 0)
        : maxSize_(maxSize), maxAge_(maxAge), maxBytesPerSensor_(maxBytesPerSensor) {}

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
        auto& bytes = queueBytes_[data.type];
        const auto addedBytes = payloadBytes(data);
        if (queue.empty() || timestamp >= timestampOf(queue.back())) {
            queue.push_back(std::move(data));
        } else {
            const auto position =
                std::upper_bound(queue.begin(), queue.end(), timestamp,
                                 [](const rclcpp::Time& value, const SensorData& candidate) {
                                     return value < timestampOf(candidate);
                                 });
            queue.insert(position, std::move(data));
        }
        bytes = addedBytes > std::numeric_limits<std::size_t>::max() - bytes
                    ? std::numeric_limits<std::size_t>::max()
                    : bytes + addedBytes;
        while (queue.size() > maxSize_ || (maxBytesPerSensor_ > 0 && bytes > maxBytesPerSensor_)) {
            popFront(queue, bytes);
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
    // Each per-sensor queue is timestamp ordered. Normal in-order arrivals stay O(1);
    // occasional out-of-order frames take O(N) insertion so age/count eviction remains correct.
    void evictExpiredLocked() {
        if (maxAge_.nanoseconds() <= 0 || !hasLatestTimestamp_) {
            return;
        }
        const auto oldestAllowed = latestTimestamp_ - maxAge_;
        for (auto& entry : queues_) {
            auto& queue = entry.second;
            auto& bytes = queueBytes_[entry.first];
            while (!queue.empty() && timestampOf(queue.front()) < oldestAllowed) {
                popFront(queue, bytes);
            }
        }
    }

    static rclcpp::Time timestampOf(const SensorData& data) {
        return std::visit([](const auto& value) { return value.timestamp; }, data.data);
    }

    static std::size_t payloadBytes(const SensorData& data) {
        if (data.type == SensorType::CAMERA) {
            const auto& image = std::get<CameraData>(data.data).image;
            return image ? image->data.size() : 0;
        }
        const auto& cloud = std::get<LidarData>(data.data).cloud;
        return cloud ? cloud->data.size() : 0;
    }

    static void popFront(std::deque<SensorData>& queue, std::size_t& bytes) {
        const auto removed = payloadBytes(queue.front());
        bytes = removed > bytes ? 0 : bytes - removed;
        queue.pop_front();
    }

    mutable std::mutex mutex_;
    std::map<SensorType, std::deque<SensorData>> queues_;
    std::map<SensorType, std::size_t> queueBytes_;
    std::size_t maxSize_;
    rclcpp::Duration maxAge_;
    std::size_t maxBytesPerSensor_{0};
    rclcpp::Time latestTimestamp_;
    bool hasLatestTimestamp_{false};
};
