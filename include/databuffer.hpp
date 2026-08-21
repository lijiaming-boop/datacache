#pragma once

#include "data.hpp"

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
        buffer_.push_back(std::move(data));

        while (buffer_.size() > maxSize_) {
            buffer_.pop_front();
        }

        if (maxAge_.nanoseconds() > 0 && !buffer_.empty()) {
            const auto newest = timestampOf(buffer_.back());
            const auto oldestAllowed = newest - maxAge_;
            while (!buffer_.empty() && timestampOf(buffer_.front()) < oldestAllowed) {
                buffer_.pop_front();
            }
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
    static rclcpp::Time timestampOf(const SensorData& data) {
        return std::visit([](const auto& value) { return value.timestamp; }, data.data);
    }

    mutable std::mutex mutex_;
    std::deque<SensorData> buffer_;
    std::size_t maxSize_;
    rclcpp::Duration maxAge_;
};
