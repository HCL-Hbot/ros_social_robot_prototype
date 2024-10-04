#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class CameraSubscriber : public rclcpp::Node {
public:
    CameraSubscriber() : Node("camera_subscriber") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_image", 10, std::bind(&CameraSubscriber::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert the ROS 2 image message to an OpenCV image
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Display the image
        cv::imshow("Received Image", frame);
        cv::waitKey(1); // Wait for a key event to allow image display
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraSubscriber>());
    rclcpp::shutdown();
    return 0;
}
