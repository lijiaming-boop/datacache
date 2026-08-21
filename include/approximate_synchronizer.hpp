#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class ApproximateSynchronizer {
public:
    using Image = sensor_msgs::msg::Image;
    using PointCloud = sensor_msgs::msg::PointCloud2;
    using MatchCallback = std::function<void(const Image::SharedPtr&, const PointCloud::SharedPtr&)>;

    ApproximateSynchronizer(std::size_t queueSize, rclcpp::Duration tolerance,
                            MatchCallback callback)
        : queueSize_(std::max<std::size_t>(1, queueSize)),
          tolerance_(tolerance), callback_(std::move(callback)) {}

    void addImage(const Image::SharedPtr& image) {
        std::lock_guard<std::mutex> lock(mutex_);
        images_.push_back(image);
        trimQueue(images_);
        tryMatch();
    }

    void addPointCloud(const PointCloud::SharedPtr& cloud) {
        std::lock_guard<std::mutex> lock(mutex_);
        clouds_.push_back(cloud);
        trimQueue(clouds_);
        tryMatch();
    }

private:
    template<typename Message>
    static rclcpp::Time stamp(const std::shared_ptr<Message>& message) {
        return rclcpp::Time(message->header.stamp);
    }

    template<typename Message>
    void trimQueue(std::deque<std::shared_ptr<Message>>& queue) {
        while (queue.size() > queueSize_) {
            queue.pop_front();
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
                callback_(image, cloud);
                continue;
            }

            // The oldest message cannot be matched by a future message if it is
            // already older than the newest message on the other side.
            if (stamp(images_.front()) < stamp(clouds_.front())) {
                images_.pop_front();
            } else {
                clouds_.pop_front();
            }
        }
    }

    std::size_t queueSize_;
    rclcpp::Duration tolerance_;
    MatchCallback callback_;
    std::deque<Image::SharedPtr> images_;
    std::deque<PointCloud::SharedPtr> clouds_;
    std::mutex mutex_;
};
