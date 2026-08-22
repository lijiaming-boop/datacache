#pragma once

#include "data.hpp"

#include <algorithm>
#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>
#include <vector>

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
        buffer_.push_back(std::move(data));

        while (countOfType(data.type) > maxSize_) {
            const auto oldest = std::min_element(
                buffer_.begin(), buffer_.end(),
                [type = data.type](const SensorData& left, const SensorData& right) {
                    if (left.type != type) return false;
                    if (right.type != type) return true;
                    return timestampOf(left) < timestampOf(right);
                });
            if (oldest != buffer_.end() && oldest->type == data.type) {
                buffer_.erase(oldest);
            } else {
                break;
            }
        }

        if (!hasLatestTimestamp_ || timestamp > latestTimestamp_) {
            latestTimestamp_ = timestamp;
            hasLatestTimestamp_ = true;
        }
        if (maxAge_.nanoseconds() > 0 && hasLatestTimestamp_) {
            const auto oldestAllowed = latestTimestamp_ - maxAge_;
            buffer_.erase(
                std::remove_if(buffer_.begin(), buffer_.end(),
                    [&](const SensorData& item) {
                        return timestampOf(item) < oldestAllowed;
                    }),
                buffer_.end());
        }
    }

    std::vector<SensorData> getDataWithinTimeRange(
        const rclcpp::Time& start,
        const rclcpp::Time& end) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<SensorData> result;
        for (const auto& data : buffer_) {
            const auto timestamp = timestampOf(data);
            if (timestamp >= start && timestamp <= end) {
                result.push_back(data);
            }
        }
        return result;
    }

    std::vector<SensorData> getAllData() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return {buffer_.begin(), buffer_.end()};
    }

private:
    std::size_t countOfType(SensorType type) const {
        return static_cast<std::size_t>(std::count_if(
            buffer_.begin(), buffer_.end(),
            [type](const SensorData& item) { return item.type == type; }));
    }

    static rclcpp::Time timestampOf(const SensorData& data) {
        return std::visit([](const auto& value) { return value.timestamp; }, data.data);
    }

    mutable std::mutex mutex_;
    std::deque<SensorData> buffer_;
    std::size_t maxSize_;
    rclcpp::Duration maxAge_;
    rclcpp::Time latestTimestamp_;
    bool hasLatestTimestamp_{false};
};
