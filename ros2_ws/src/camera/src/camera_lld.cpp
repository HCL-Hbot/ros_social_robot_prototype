#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class CameraLLD : public rclcpp::Node {
public:
    CameraLLD() : Node("camera_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera_image", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&CameraLLD::publish_image, this));
        cap_.open(0); // Open the default camera (camera index 0)
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open camera");
        }
    }

private:
    void publish_image() {
        cv::Mat frame;
        cap_ >> frame; // Capture a frame from the camera

        if (!frame.empty()) {
            // Convert OpenCV image (cv::Mat) to ROS 2 image message
            std::shared_ptr<sensor_msgs::msg::Image> msg = cv_bridge::CvImage(
                std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

            publisher_->publish(*msg);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraLLD>());
    rclcpp::shutdown();
    return 0;
}
