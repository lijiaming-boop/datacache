#pragma once

#include "data.hpp"
#include "disk_space_manager.hpp"
#include "pair_index.hpp"

#include <algorithm>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>
#endif

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <zstd.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

class RawStorageWorker {
public:
    explicit RawStorageWorker(rclcpp::Logger logger, std::size_t maxPendingJobs = 20,
                              std::shared_ptr<DiskSpaceManager> diskManager = nullptr)
        : logger_(std::move(logger)), maxPendingJobs_(maxPendingJobs),
          diskManager_(std::move(diskManager)), compressionContext_(ZSTD_createCCtx()),
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
        if (compressionContext_) {
            ZSTD_freeCCtx(compressionContext_);
        }
    }

    // finalJob: 该事件目录的最后一批写入(post 窗口数据, 或无 post 窗口时的唯一
    // 一批)。完成后写入 .complete 标记, 上传模块以此识别可回传的事件目录。
    bool enqueue(const std::filesystem::path& directory, std::vector<SensorData> records,
                 bool recordCamera, bool recordLidar, bool compressionEnabled, int compressionLevel,
                 bool keepRaw, bool conversionEnabled, std::string imageFormat, int imageQuality,
                 std::string pointCloudFormat, std::vector<PairRecord> pairs = {},
                 bool reserved = false, bool finalJob = false) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_ || (!reserved && jobs_.size() + reservedJobs_ >= maxPendingJobs_)) {
            if (reserved && reservedJobs_ > 0) {
                --reservedJobs_;
            }
            RCLCPP_ERROR(logger_, "Raw storage queue is full or stopping; dropping %zu records",
                         records.size());
            return false;
        }

        jobs_.push_back(Job{directory, std::move(records), recordCamera, recordLidar,
                            compressionEnabled, compressionLevel, keepRaw, conversionEnabled,
                            std::move(imageFormat), imageQuality, std::move(pointCloudFormat),
                            std::move(pairs), finalJob});
        if (reserved && reservedJobs_ > 0) {
            --reservedJobs_;
        }
        condition_.notify_one();
        return true;
    }

    bool reserve() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_ || jobs_.size() + reservedJobs_ >= maxPendingJobs_) {
            return false;
        }
        ++reservedJobs_;
        return true;
    }

    void releaseReservation() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (reservedJobs_ > 0) {
            --reservedJobs_;
        }
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
        bool conversionEnabled;
        std::string imageFormat;
        int imageQuality;
        std::string pointCloudFormat;
        std::vector<PairRecord> pairs;
        bool finalJob{false};
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

#ifndef _WIN32
    // 断电安全: rename 只保证原子可见, 不保证数据块先于目录项落盘。
    // 数据文件写完先 fsync 再 rename, rename 后同步父目录项, 掉电时才不会
    // 出现"标记完整但内容为空"的文件。Windows 开发环境下退化为纯 rename。
    static void syncFile(const std::filesystem::path& file) {
        const int descriptor = ::open(file.string().c_str(), O_WRONLY);
        if (descriptor < 0) {
            return;
        }
        ::fsync(descriptor);
        ::close(descriptor);
    }

    static void syncDirectory(const std::filesystem::path& directory) {
        const int descriptor = ::open(directory.string().c_str(), O_RDONLY | O_DIRECTORY);
        if (descriptor < 0) {
            return;
        }
        ::fsync(descriptor);
        ::close(descriptor);
    }
#endif

    static bool writeAtomically(const std::filesystem::path& target, const void* data,
                                std::size_t size) {
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

#ifndef _WIN32
        syncFile(temporary);
#endif
        std::error_code error;
        std::filesystem::rename(temporary, target, error);
        if (error) {
            std::filesystem::remove(temporary);
            return false;
        }
#ifndef _WIN32
        syncDirectory(target.parent_path());
#endif
        return true;
    }

    static std::string imageExtension(const std::string& format) {
        return format == "png" ? ".png" : ".jpg";
    }

    bool convertImage(const CameraData& camera, const std::filesystem::path& directory,
                      const std::string& fileName, const Job& job) const {
        const auto& message = *camera.image;
        if (message.height == 0 || message.width == 0 || message.step == 0 ||
            message.step * message.height > message.data.size()) {
            RCLCPP_WARN(logger_, "Invalid image buffer; skipping format conversion");
            return false;
        }
        cv::Mat image;
        const auto& encoding = message.encoding;
        if (encoding == "bgr8") {
            image = cv::Mat(static_cast<int>(message.height), static_cast<int>(message.width),
                            CV_8UC3, const_cast<unsigned char*>(message.data.data()), message.step);
        } else if (encoding == "rgb8") {
            const cv::Mat rgb(static_cast<int>(message.height), static_cast<int>(message.width),
                              CV_8UC3, const_cast<unsigned char*>(message.data.data()),
                              message.step);
            cv::cvtColor(rgb, image, cv::COLOR_RGB2BGR);
        } else if (encoding == "mono8") {
            image = cv::Mat(static_cast<int>(message.height), static_cast<int>(message.width),
                            CV_8UC1, const_cast<unsigned char*>(message.data.data()), message.step);
        } else if (encoding == "bgra8") {
            const cv::Mat bgra(static_cast<int>(message.height), static_cast<int>(message.width),
                               CV_8UC4, const_cast<unsigned char*>(message.data.data()),
                               message.step);
            cv::cvtColor(bgra, image, cv::COLOR_BGRA2BGR);
        } else if (encoding == "rgba8") {
            const cv::Mat rgba(static_cast<int>(message.height), static_cast<int>(message.width),
                               CV_8UC4, const_cast<unsigned char*>(message.data.data()),
                               message.step);
            cv::cvtColor(rgba, image, cv::COLOR_RGBA2BGR);
        } else {
            RCLCPP_WARN(logger_, "Unsupported image encoding for conversion: %s", encoding.c_str());
            return false;
        }

        std::vector<unsigned char> encoded;
        const auto extension = imageExtension(job.imageFormat);
        std::vector<int> parameters;
        if (extension == ".jpg") {
            parameters = {cv::IMWRITE_JPEG_QUALITY, std::clamp(job.imageQuality, 1, 100)};
        }
        if (!cv::imencode(extension, image, encoded, parameters)) {
            return false;
        }
        return writeAtomically(directory / (fileName + extension), encoded.data(), encoded.size());
    }

    bool convertPointCloud(const LidarData& lidar, const std::filesystem::path& directory,
                           const std::string& fileName, const Job& job) const {
        if (job.pointCloudFormat != "pcd") {
            RCLCPP_WARN(logger_, "Unsupported point cloud format: %s",
                        job.pointCloudFormat.c_str());
            return false;
        }

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*lidar.cloud, cloud);
        const auto target = directory / (fileName + ".pcd");
        const auto temporary = target.string() + ".tmp";
        if (pcl::io::savePCDFileBinaryCompressed(temporary, cloud) != 0) {
            std::error_code error;
            std::filesystem::remove(temporary, error);
            return false;
        }

        std::error_code error;
        std::filesystem::rename(temporary, target, error);
        if (error) {
            std::filesystem::remove(temporary);
            return false;
        }
#ifndef _WIN32
        syncFile(target);
        syncDirectory(directory);
#endif
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
        if (diskManager_ && !diskManager_->prepareForWrite()) {
            RCLCPP_ERROR(logger_, "Insufficient disk space; dropping %zu records destined for %s",
                         job.records.size(), job.directory.string().c_str());
            return;
        }

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
            manifest << "sensor,timestamp,file,encoding,converted_file\n";
        }

        writePairs(job.directory, job.pairs);

        for (const auto& record : job.records) {
            const bool enabled =
                record.type == SensorType::CAMERA ? job.recordCamera : job.recordLidar;
            if (!enabled) {
                continue;
            }

            const auto timestamp = timestampOf(record);
            const auto prefix = record.type == SensorType::CAMERA ? "camera" : "lidar";
            rclcpp::SerializedMessage serialized;
            serialize(record, serialized);
            const auto& raw = serialized.get_rcl_serialized_message();
            const auto rawFileName =
                std::string(prefix) + "_" + std::to_string(timestamp.nanoseconds()) + ".bin";
            auto storedFileName = rawFileName;
            bool compressed = false;
            std::string convertedFileName;

            if (job.compressionEnabled) {
                const auto bound = ZSTD_compressBound(raw.buffer_length);
                std::vector<std::uint8_t> compressedData(bound);
                // 复用压缩上下文 + 帧尾开启 XXH64 校验和, 静默位腐在解压时即可
                // 被发现(默认 ZSTD_compress 不写校验和)
                ZSTD_CCtx_setParameter(compressionContext_, ZSTD_c_compressionLevel,
                                       job.compressionLevel);
                ZSTD_CCtx_setParameter(compressionContext_, ZSTD_c_checksumFlag, 1);
                const auto compressedSize =
                    ZSTD_compress2(compressionContext_, compressedData.data(),
                                   compressedData.size(), raw.buffer, raw.buffer_length);
                if (!ZSTD_isError(compressedSize)) {
                    if (writeAtomically(job.directory / (rawFileName + ".zst"),
                                        compressedData.data(), compressedSize)) {
                        storedFileName += ".zst";
                        compressed = true;
                    }
                } else {
                    RCLCPP_ERROR(logger_, "Zstandard compression failed for %s: %s",
                                 rawFileName.c_str(), ZSTD_getErrorName(compressedSize));
                }
            }

            if (!compressed || job.keepRaw) {
                if (!writeAtomically(job.directory / rawFileName, raw.buffer, raw.buffer_length)) {
                    RCLCPP_ERROR(logger_, "Unable to write raw record file: %s",
                                 rawFileName.c_str());
                    continue;
                }
            }

            if (job.conversionEnabled) {
                const auto convertedDirectory =
                    job.directory / (record.type == SensorType::CAMERA ? "images" : "pointclouds");
                std::error_code error;
                std::filesystem::create_directories(convertedDirectory, error);
                if (!error) {
                    const auto baseName =
                        std::string(prefix) + "_" + std::to_string(timestamp.nanoseconds());
                    const bool converted =
                        record.type == SensorType::CAMERA
                            ? convertImage(std::get<CameraData>(record.data), convertedDirectory,
                                           baseName, job)
                            : convertPointCloud(std::get<LidarData>(record.data),
                                                convertedDirectory, baseName, job);
                    if (converted) {
                        convertedFileName = (convertedDirectory.filename() /
                                             (baseName + (record.type == SensorType::CAMERA
                                                              ? imageExtension(job.imageFormat)
                                                              : ".pcd")))
                                                .string();
                    }
                }
            }
            manifest << prefix << "," << timestamp.nanoseconds() << "," << storedFileName << ","
                     << (compressed ? "zstd" : "raw") << "," << convertedFileName << "\n";
        }

        // .complete 必须在清单文件越过用户态缓冲之后写入: 否则进程在该窗口内
        // 崩溃会留下"标记完整但 manifest 尾行缺失"的目录, 上传器会把它当完整
        // 目录发走
        manifest.flush();
        if (!manifest) {
            RCLCPP_ERROR(logger_, "Unable to flush raw record manifest in %s",
                         job.directory.string().c_str());
            return;
        }
        manifest.close();

        if (job.finalJob) {
#ifndef _WIN32
            // 追加写的清单同样要越过页缓存, .complete 的可见性才有意义
            syncFile(job.directory / "manifest.csv");
            if (!job.pairs.empty()) {
                syncFile(job.directory / "pairs.csv");
            }
#endif
            writeCompletionMarker(job.directory, job.records.size());
        }

        if (diskManager_) {
            diskManager_->enforceRetention();
        }
    }

    // .complete 是事件目录写完的信号(原子写入), UploadWorker 据此启动回传
    static void writeCompletionMarker(const std::filesystem::path& directory,
                                      std::size_t recordCount) {
        std::ostringstream content;
        content << "records=" << recordCount << "\n";
        const auto text = content.str();
        writeAtomically(directory / ".complete", text.data(), text.size());
    }

    static void writePairs(const std::filesystem::path& directory,
                           const std::vector<PairRecord>& pairs) {
        if (pairs.empty())
            return;
        std::ofstream output(directory / "pairs.csv", std::ios::app);
        if (!output)
            return;
        if (output.tellp() == 0) {
            output << "pair_id,status,camera_timestamp,lidar_timestamp,time_diff_ns,reason\n";
        }
        for (const auto& pair : pairs) {
            output << pair.pairId << ',' << pair.status << ','
                   << (pair.hasCamera ? std::to_string(pair.cameraTimestamp.nanoseconds()) : "")
                   << ','
                   << (pair.hasLidar ? std::to_string(pair.lidarTimestamp.nanoseconds()) : "")
                   << ',' << pair.difference.nanoseconds() << ',' << pair.reason << '\n';
        }
    }

    rclcpp::Logger logger_;
    std::size_t maxPendingJobs_;
    std::shared_ptr<DiskSpaceManager> diskManager_;
    // 压缩只在 worker_ 线程的 writeJob 里使用, 无需加锁; 声明在 worker_ 之前
    // 保证线程启动前已就绪
    mutable ZSTD_CCtx* compressionContext_;
    std::size_t reservedJobs_{0};
    std::deque<Job> jobs_;
    std::mutex mutex_;
    std::condition_variable condition_;
    bool stopping_{false};
    std::thread worker_;
};
