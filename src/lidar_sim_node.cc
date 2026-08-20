#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class LidarSimNode : public rclcpp::Node {
public:
    LidarSimNode() : Node("lidar_sim_node") {
        declare_parameter<std::string>("pcd_path", "pcd/sample.pcd");
        declare_parameter<int>("hz", 10);

        const auto pcdPath = get_parameter("pcd_path").as_string();
        const int hz = get_parameter("hz").as_int();

        cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdPath, *cloud_) == -1) {
            throw std::runtime_error("Failed to load PCD file: " + pcdPath);
        }

        pcl::toROSMsg(*cloud_, cloudMsg_);
        cloudMsg_.header.frame_id = "lidar_frame";

        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);

        const auto period = std::chrono::milliseconds(1000 / hz);
        timer_ = create_wall_timer(period, [this]() { publishCloud(); });

        RCLCPP_INFO(get_logger(), "LidarSimNode started [pcd=%s, points=%zu, hz=%d]",
                    pcdPath.c_str(), cloud_->size(), hz);
    }

private:
    void publishCloud() {
        cloudMsg_.header.stamp = now();
        publisher_->publish(cloudMsg_);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    sensor_msgs::msg::PointCloud2 cloudMsg_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSimNode>());
    rclcpp::shutdown();
    return 0;
}
