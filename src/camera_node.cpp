#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node") {
        declare_parameter<int>("fps", 30);
        declare_parameter<int>("device", 0);
        declare_parameter<int>("width", 640);
        declare_parameter<int>("height", 480);

        const int fps = get_parameter("fps").as_int();
        const int device = get_parameter("device").as_int();
        const int width = get_parameter("width").as_int();
        const int height = get_parameter("height").as_int();

        capture_.open(device);
        if (!capture_.isOpened()) {
            throw std::runtime_error("Failed to open camera device " + std::to_string(device));
        }

        capture_.set(cv::CAP_PROP_FRAME_WIDTH, width);
        capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height);

        publisher_ = create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);

        const auto period = std::chrono::milliseconds(1000 / fps);
        timer_ = create_wall_timer(period, [this]() { captureAndPublish(); });

        RCLCPP_INFO(get_logger(), "CameraNode started [device=%d, fps=%d, %dx%d]",
                    device, fps, width, height);
    }

    ~CameraNode() override {
        if (capture_.isOpened()) {
            capture_.release();
        }
    }

private:
    void captureAndPublish() {
        cv::Mat frame;
        if (!capture_.read(frame) || frame.empty()) {
            RCLCPP_WARN(get_logger(), "Failed to capture frame");
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
                       .toImageMsg();
        msg->header.stamp = now();
        msg->header.frame_id = "camera_frame";

        publisher_->publish(*msg);
    }

    cv::VideoCapture capture_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
