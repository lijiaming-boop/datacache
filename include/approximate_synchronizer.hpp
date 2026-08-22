#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class ApproximateSynchronizer {
public:
    using Image = sensor_msgs::msg::Image;
    using PointCloud = sensor_msgs::msg::PointCloud2;
    using MatchCallback = std::function<void(const Image::SharedPtr&, const PointCloud::SharedPtr&,
                                              rclcpp::Duration)>;
    using DropCallback = std::function<void(const std::string&, const rclcpp::Time&,
                                             std::uint64_t, const std::string&)>;

    ApproximateSynchronizer(std::size_t queueSize, rclcpp::Duration tolerance,
                            MatchCallback callback, DropCallback dropCallback = {})
        : queueSize_(std::max<std::size_t>(1, queueSize)),
          tolerance_(tolerance), callback_(std::move(callback)),
          dropCallback_(std::move(dropCallback)) {}

    void addImage(const Image::SharedPtr& image) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (images_.size() >= queueSize_) {
            const auto droppedTimestamp = stamp(images_.front());
            images_.pop_front();
            reportDrop("camera", droppedTimestamp, "synchronizer queue full");
        }
        images_.push_back(image);
        tryMatch();
    }

    void addPointCloud(const PointCloud::SharedPtr& cloud) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (clouds_.size() >= queueSize_) {
            const auto droppedTimestamp = stamp(clouds_.front());
            clouds_.pop_front();
            reportDrop("lidar", droppedTimestamp, "synchronizer queue full");
        }
        clouds_.push_back(cloud);
        tryMatch();
    }

    // Finalize messages waiting at an event boundary. They remain in the raw
    // sensor buffer, but are explicitly represented as single-sided records.
    void flushUnmatched(const std::string& reason = "event window closed") {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!images_.empty()) {
            const auto timestamp = stamp(images_.front());
            images_.pop_front();
            reportDrop("camera", timestamp, reason);
        }
        while (!clouds_.empty()) {
            const auto timestamp = stamp(clouds_.front());
            clouds_.pop_front();
            reportDrop("lidar", timestamp, reason);
        }
    }

private:
    template<typename Message>
    static rclcpp::Time stamp(const std::shared_ptr<Message>& message) {
        return rclcpp::Time(message->header.stamp);
    }

    void reportDrop(const std::string& sensor, const rclcpp::Time& timestamp,
                    const std::string& reason) {
        auto& count = sensor == "camera" ? droppedImages_ : droppedClouds_;
        ++count;
        if (dropCallback_) {
            dropCallback_(sensor, timestamp, count, reason);
        }
    }

    void tryMatch() {
        while (!images_.empty() && !clouds_.empty()) {
            auto bestImage = images_.end();
            auto bestCloud = clouds_.end();
            int64_t bestDifference = tolerance_.nanoseconds() + 1;

            for (auto image = images_.begin(); image != images_.end(); ++image) {
                for (auto cloud = clouds_.begin(); cloud != clouds_.end(); ++cloud) {
                    const auto difference = std::llabs(
                        (stamp(*image) - stamp(*cloud)).nanoseconds());
                    if (difference < bestDifference) {
                        bestDifference = difference;
                        bestImage = image;
                        bestCloud = cloud;
                    }
                }
            }

            if (bestImage == images_.end()) {
                return;
            }

            if (bestDifference <= tolerance_.nanoseconds()) {
                const auto image = *bestImage;
                const auto cloud = *bestCloud;
                images_.erase(bestImage);
                clouds_.erase(bestCloud);
                callback_(image, cloud, rclcpp::Duration::from_nanoseconds(bestDifference));
                continue;
            }

            // The oldest message cannot be matched by a future message if it is
            // already older than the newest message on the other side.
            if (stamp(images_.front()) < stamp(clouds_.front())) {
                const auto droppedTimestamp = stamp(images_.front());
                images_.pop_front();
                reportDrop("camera", droppedTimestamp, "outside synchronization tolerance");
            } else {
                const auto droppedTimestamp = stamp(clouds_.front());
                clouds_.pop_front();
                reportDrop("lidar", droppedTimestamp, "outside synchronization tolerance");
            }
        }
    }

    std::size_t queueSize_;
    rclcpp::Duration tolerance_;
    MatchCallback callback_;
    DropCallback dropCallback_;
    std::deque<Image::SharedPtr> images_;
    std::deque<PointCloud::SharedPtr> clouds_;
    std::mutex mutex_;
    std::uint64_t droppedImages_{0};
    std::uint64_t droppedClouds_{0};
};
