
#include "camera_lld.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

constexpr const char* DEFAULT_NODE_NAME = "camera_lld_node";
constexpr const char* DEFAULT_TOPIC_NAME_SUB = "raw_image";
constexpr const char* DEFAULT_CAMERA = "/dev/video0";
constexpr const char* CAMERA_PARAMETER = "camera_device_location";

namespace camera_lld {

CameraLLD::CameraLLD() 
  : rclcpp::Node(DEFAULT_NODE_NAME),
  raw_image_pub_(this->create_publisher<sensor_msgs::msg::Image>(DEFAULT_TOPIC_NAME_SUB, 10)),
  camera_device_location_(DEFAULT_CAMERA) {

  camera_device_location_ = this->declare_parameter<std::string>(CAMERA_PARAMETER, DEFAULT_CAMERA);
  RCLCPP_INFO(this->get_logger(), "Node '%s' will use camera '%s' ",this->get_name(),camera_device_location_.c_str());
  camera_thread_= std::thread(std::bind(&CameraLLD::captureAndPublish,this));
}

/*virtual*/ CameraLLD::~CameraLLD() {
  if (camera_thread_.joinable())
  {
    camera_thread_.join();
  }
}

void CameraLLD::captureAndPublish() {
  cv::VideoCapture cap(camera_device_location_);

  if (!cap.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
    rclcpp::shutdown();
  }

  std_msgs::msg::Header header;

  while (rclcpp::ok()) {
    cv::Mat frame;
    cap >> frame;  // read frame

    if (!frame.empty()) {
      header.stamp = this->get_clock()->now();
      auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
      raw_image_pub_->publish(*msg);  // publish frame
    }
  }
}

}  // namespace camera_lld
