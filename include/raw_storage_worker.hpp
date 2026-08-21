#pragma once

#include "data.hpp"

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <zstd.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

class RawStorageWorker {
public:
    explicit RawStorageWorker(rclcpp::Logger logger, std::size_t maxPendingJobs = 20)
        : logger_(std::move(logger)), maxPendingJobs_(maxPendingJobs),
          worker_(&RawStorageWorker::run, this) {}

    RawStorageWorker(const RawStorageWorker&) = delete;
    RawStorageWorker& operator=(const RawStorageWorker&) = delete;

    ~RawStorageWorker() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        condition_.notify_one();
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    bool enqueue(const std::filesystem::path& directory,
                 std::vector<SensorData> records,
                 bool recordCamera,
                 bool recordLidar,
                 bool compressionEnabled,
                 int compressionLevel,
                 bool keepRaw) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_ || jobs_.size() >= maxPendingJobs_) {
            RCLCPP_ERROR(logger_, "Raw storage queue is full or stopping; dropping %zu records",
                         records.size());
            return false;
        }

        jobs_.push_back(Job{directory, std::move(records), recordCamera, recordLidar,
                            compressionEnabled, compressionLevel, keepRaw});
        condition_.notify_one();
        return true;
    }

private:
    struct Job {
        std::filesystem::path directory;
        std::vector<SensorData> records;
        bool recordCamera;
        bool recordLidar;
        bool compressionEnabled;
        int compressionLevel;
        bool keepRaw;
    };

    static rclcpp::Time timestampOf(const SensorData& data) {
        return std::visit([](const auto& value) { return value.timestamp; }, data.data);
    }

    static void serialize(const SensorData& data, rclcpp::SerializedMessage& serialized) {
        if (data.type == SensorType::CAMERA) {
            rclcpp::Serialization<sensor_msgs::msg::Image>().serialize_message(
                std::get<CameraData>(data.data).image.get(), &serialized);
        } else {
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2>().serialize_message(
                std::get<LidarData>(data.data).cloud.get(), &serialized);
        }
    }

    static bool writeAtomically(const std::filesystem::path& target,
                                const void* data, std::size_t size) {
        const auto temporary = target.string() + ".tmp";
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        if (!output) {
            return false;
        }
        output.write(static_cast<const char*>(data), static_cast<std::streamsize>(size));
        output.close();
        if (!output) {
            return false;
        }

        std::error_code error;
        std::filesystem::rename(temporary, target, error);
        if (error) {
            std::filesystem::remove(temporary);
            return false;
        }
        return true;
    }

    void run() {
        while (true) {
            Job job;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this]() { return stopping_ || !jobs_.empty(); });
                if (jobs_.empty() && stopping_) {
                    return;
                }
                job = std::move(jobs_.front());
                jobs_.pop_front();
            }
            writeJob(job);
        }
    }

    void writeJob(const Job& job) const {
        std::error_code error;
        std::filesystem::create_directories(job.directory, error);
        if (error) {
            RCLCPP_ERROR(logger_, "Unable to create raw record directory %s: %s",
                         job.directory.string().c_str(), error.message().c_str());
            return;
        }

        std::ofstream manifest(job.directory / "manifest.csv", std::ios::app);
        if (!manifest) {
            RCLCPP_ERROR(logger_, "Unable to open raw record manifest in %s",
                         job.directory.string().c_str());
            return;
        }
        if (manifest.tellp() == 0) {
            manifest << "sensor,timestamp,file,encoding\n";
        }

        for (const auto& record : job.records) {
            const bool enabled = record.type == SensorType::CAMERA
                ? job.recordCamera : job.recordLidar;
            if (!enabled) {
                continue;
            }

            const auto timestamp = timestampOf(record);
            const auto prefix = record.type == SensorType::CAMERA ? "camera" : "lidar";
            rclcpp::SerializedMessage serialized;
            serialize(record, serialized);
            const auto& raw = serialized.get_rcl_serialized_message();
            const auto rawFileName = std::string(prefix) + "_" +
                std::to_string(timestamp.nanoseconds()) + ".bin";
            auto storedFileName = rawFileName;
            bool compressed = false;

            if (job.compressionEnabled) {
                const auto bound = ZSTD_compressBound(raw.buffer_length);
                std::vector<std::uint8_t> compressedData(bound);
                const auto compressedSize = ZSTD_compress(
                    compressedData.data(), compressedData.size(), raw.buffer,
                    raw.buffer_length, job.compressionLevel);
                if (!ZSTD_isError(compressedSize)) {
                    storedFileName += ".zst";
                    compressed = writeAtomically(job.directory / storedFileName,
                                                 compressedData.data(), compressedSize);
                } else {
                    RCLCPP_ERROR(logger_, "Zstandard compression failed for %s: %s",
                                 rawFileName.c_str(), ZSTD_getErrorName(compressedSize));
                }
            }

            if (!compressed || job.keepRaw) {
                if (!writeAtomically(job.directory / rawFileName, raw.buffer, raw.buffer_length)) {
                    RCLCPP_ERROR(logger_, "Unable to write raw record file: %s", rawFileName.c_str());
                    continue;
                }
                if (compressed && !job.keepRaw) {
                    storedFileName = rawFileName;
                    compressed = false;
                }
            }
            manifest << prefix << "," << timestamp.nanoseconds() << ","
                     << storedFileName << "," << (compressed ? "zstd" : "raw") << "\n";
        }
    }

    rclcpp::Logger logger_;
    std::size_t maxPendingJobs_;
    std::deque<Job> jobs_;
    std::mutex mutex_;
    std::condition_variable condition_;
    bool stopping_{false};
    std::thread worker_;
};
