#pragma once

#include <cstdint>
#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/time.hpp>

struct PairRecord {
    std::uint64_t pairId;
    bool hasCamera;
    bool hasLidar;
    rclcpp::Time cameraTimestamp;
    rclcpp::Time lidarTimestamp;
    rclcpp::Duration difference;
    std::string status;
    std::string reason;
};

class PairIndex {
public:
    void addMatched(const rclcpp::Time& camera, const rclcpp::Time& lidar,
                    const rclcpp::Duration& difference) {
        std::lock_guard<std::mutex> lock(mutex_);
        records_.push_back(PairRecord{nextId_++, true, true, camera, lidar,
                                      difference, "matched", ""});
        trim();
    }

    void addSingle(const std::string& sensor, const rclcpp::Time& timestamp,
                   const std::string& reason) {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool camera = sensor == "camera";
        records_.push_back(PairRecord{nextId_++, camera, !camera,
                                      camera ? timestamp : rclcpp::Time(),
                                      camera ? rclcpp::Time() : timestamp,
                                      rclcpp::Duration(0, 0),
                                      camera ? "camera_only" : "lidar_only", reason});
        trim();
    }

    std::vector<PairRecord> getDataWithinTimeRange(const rclcpp::Time& start,
                                                    const rclcpp::Time& end) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<PairRecord> result;
        for (const auto& record : records_) {
            const auto timestamp = record.hasCamera ? record.cameraTimestamp : record.lidarTimestamp;
            if (timestamp >= start && timestamp <= end) {
                result.push_back(record);
            }
        }
        return result;
    }

private:
    void trim() {
        constexpr std::size_t maxRecords = 100000;
        if (records_.size() > maxRecords) {
            records_.erase(records_.begin(), records_.begin() +
                           static_cast<std::ptrdiff_t>(records_.size() - maxRecords));
        }
    }

    mutable std::mutex mutex_;
    std::vector<PairRecord> records_;
    std::uint64_t nextId_{1};
};
