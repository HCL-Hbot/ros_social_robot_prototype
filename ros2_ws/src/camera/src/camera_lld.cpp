#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "camera_lld.hpp"


CameraLLD::CameraLLD(const std::string& node_name) :
  rclcpp::Node(node_name),
  publisher_(this->create_publisher<sensor_msgs::msg::Image>("raw_image", 10)),
  camera_thread_(std::thread(std::bind(&CameraLLD::captureAndPublish,this)))
{
}

/*virtual*/ CameraLLD::~CameraLLD()
{
  if (camera_thread_.joinable())
  {
    camera_thread_.join();
  }
}

void CameraLLD::captureAndPublish()
{
    cv::VideoCapture cap(0);  // Open de camera
    if (!cap.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
      return;
    }

    while (rclcpp::ok())  // Continue draaien zolang de node actief is
    {
      cv::Mat frame;
      cap >> frame;  // Lees een frame van de camera
      if (!frame.empty()) {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*msg);  // Publiceer het beeld
      }
    }
}
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraLLD>("camera_lld"));
    rclcpp::shutdown();
    return 0;
}
