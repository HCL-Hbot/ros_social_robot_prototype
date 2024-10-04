#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "camera_hld.hpp"


CameraHLD::CameraHLD(const std::string& node_name) : 
  rclcpp::Node(node_name),
  subscription_(create_subscription<sensor_msgs::msg::Image>(
            "raw_image", 10, std::bind(&CameraHLD::imageCallback, this, std::placeholders::_1)))
{
}

/*virtual*/ CameraHLD::~CameraHLD()
{
}

void CameraHLD::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Convert the ROS 2 image message to an OpenCV image
  cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
 

  // Display the image
  cv::imshow("Received Image", frame);
  cv::waitKey(1); // Wait for a key event to allow image display
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraHLD>("camera_hld"));
    rclcpp::shutdown();
    return 0;
}