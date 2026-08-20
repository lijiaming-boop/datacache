#pragma once

#include "data.hpp"

#include <cstddef>
#include <deque>
#include <mutex>

class DataBuffer {
public:
    explicit DataBuffer(std::size_t maxSize) : maxSize_(maxSize) {}

    void addData(const SensorData& data) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (maxSize_ == 0) {
            return;
        }
        if (buffer_.size() >= maxSize_) {
            buffer_.pop_front();
        }
        buffer_.push_back(data);
    }

    std::vector<SensorData> getDataWithinTimeRange(
        const rclcpp::Time& start,
        const rclcpp::Time& end) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<SensorData> result;
        for (const auto& data : buffer_) {
            const auto timestamp = data.type == SensorType::CAMERA
                ? std::get<CameraData>(data.data).timestamp
                : std::get<LidarData>(data.data).timestamp;
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
    mutable std::mutex mutex_;
    std::deque<SensorData> buffer_;
    std::size_t maxSize_;
};
