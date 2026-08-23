#pragma once

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/time.hpp>
#include <variant>

struct CameraData {
    rclcpp::Time timestamp;
    sensor_msgs::msg::Image::SharedPtr image;
};

struct LidarData {
    rclcpp::Time timestamp;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
};

enum class SensorType { CAMERA, LIDAR };

using SensorDataVariant = std::variant<CameraData, LidarData>;

struct SensorData {
    SensorType type;
    SensorDataVariant data;
};
