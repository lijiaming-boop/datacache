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
#include <utility>
#include <vector>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class ApproximateSynchronizer {
public:
    using Image = sensor_msgs::msg::Image;
    using PointCloud = sensor_msgs::msg::PointCloud2;
    using MatchCallback = std::function<void(const Image::SharedPtr&, const PointCloud::SharedPtr&,
                                             rclcpp::Duration)>;
    using DropCallback = std::function<void(const std::string&, const rclcpp::Time&, std::uint64_t,
                                            const std::string&)>;

    ApproximateSynchronizer(std::size_t queueSize, rclcpp::Duration tolerance,
                            MatchCallback callback, DropCallback dropCallback = {})
        : queueSize_(std::max<std::size_t>(1, queueSize)), tolerance_(tolerance),
          callback_(std::move(callback)), dropCallback_(std::move(dropCallback)) {}

    void addImage(const Image::SharedPtr& image) {
        PendingResults results;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (images_.size() >= queueSize_) {
                recordDrop(results.drops, "camera", stamp(images_.front()),
                           "synchronizer queue full");
                images_.pop_front();
            }
            images_.push_back(image);
            tryMatchLocked(results);
        }
        dispatch(results);
    }

    void addPointCloud(const PointCloud::SharedPtr& cloud) {
        PendingResults results;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (clouds_.size() >= queueSize_) {
                recordDrop(results.drops, "lidar", stamp(clouds_.front()),
                           "synchronizer queue full");
                clouds_.pop_front();
            }
            clouds_.push_back(cloud);
            tryMatchLocked(results);
        }
        dispatch(results);
    }

    // Finalize messages waiting at an event boundary. They remain in the raw
    // sensor buffer, but are explicitly represented as single-sided records.
    void flushUnmatched(const std::string& reason = "event window closed") {
        PendingResults results;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            while (!images_.empty()) {
                recordDrop(results.drops, "camera", stamp(images_.front()), reason);
                images_.pop_front();
            }
            while (!clouds_.empty()) {
                recordDrop(results.drops, "lidar", stamp(clouds_.front()), reason);
                clouds_.pop_front();
            }
        }
        dispatch(results);
    }

private:
    template <typename Message> static rclcpp::Time stamp(const std::shared_ptr<Message>& message) {
        return rclcpp::Time(message->header.stamp);
    }

    struct MatchResult {
        Image::SharedPtr image;
        PointCloud::SharedPtr cloud;
        rclcpp::Duration difference;
    };

    struct DropResult {
        std::string sensor;
        rclcpp::Time timestamp;
        std::uint64_t count;
        std::string reason;
    };

    struct PendingResults {
        std::vector<MatchResult> matches;
        std::vector<DropResult> drops;
    };

    void recordDrop(std::vector<DropResult>& drops, const std::string& sensor,
                    const rclcpp::Time& timestamp, const std::string& reason) {
        auto& count = sensor == "camera" ? droppedImages_ : droppedClouds_;
        ++count;
        drops.push_back(DropResult{sensor, timestamp, count, reason});
    }

    // Front-based greedy pairing on the two timestamp-ordered queues. If the
    // oldest message on one side already trails the other side's front by more
    // than the tolerance, every future opposite-side message can only be
    // further away, so that message can never match and is dropped. This makes
    // each arrival O(1) amortized instead of the previous full O(N*M) scan.
    void tryMatchLocked(PendingResults& results) {
        while (!images_.empty() && !clouds_.empty()) {
            const auto difference = (stamp(images_.front()) - stamp(clouds_.front())).nanoseconds();
            if (std::llabs(difference) <= tolerance_.nanoseconds()) {
                results.matches.push_back(
                    MatchResult{images_.front(), clouds_.front(),
                                rclcpp::Duration::from_nanoseconds(std::llabs(difference))});
                images_.pop_front();
                clouds_.pop_front();
                continue;
            }
            if (difference < 0) {
                recordDrop(results.drops, "camera", stamp(images_.front()),
                           "outside synchronization tolerance");
                images_.pop_front();
            } else {
                recordDrop(results.drops, "lidar", stamp(clouds_.front()),
                           "outside synchronization tolerance");
                clouds_.pop_front();
            }
        }
    }

    // Callbacks touch PairIndex and log; they must run outside mutex_.
    void dispatch(const PendingResults& results) const {
        for (const auto& match : results.matches) {
            callback_(match.image, match.cloud, match.difference);
        }
        if (dropCallback_) {
            for (const auto& drop : results.drops) {
                dropCallback_(drop.sensor, drop.timestamp, drop.count, drop.reason);
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
