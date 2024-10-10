#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "camera_hld.hpp"

CameraHLD::CameraHLD() : 
  rclcpp::Node("camera_hld"),
  raw_image_sub_(create_subscription<sensor_msgs::msg::Image>(
            "raw_image", 10, std::bind(&CameraHLD::imageCallback, this, std::placeholders::_1)))
{
  tf_frame_id_ = this->declare_parameter<std::string>("tf_frame_id", "");
  if(tf_frame_id_== "")
  {
    RCLCPP_WARN(this->get_logger(), "Parameter 'tf_frame_id' is not configured for the node '%s'. Node will be aborted.",this->get_name());
    rclcpp::shutdown();
  }

  face_info_pub_ = this->create_publisher<camera_hld::msg::FaceInfo>("face_info_topic", 10);
  RCLCPP_INFO(this->get_logger(), "Starting CameraHLD with node name '%s' and frame_id '%s' ", this->get_name(), tf_frame_id_.c_str());
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

  auto message = camera_hld::msg::FaceInfo();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = tf_frame_id_;
  message.bounding_box_x = 10;
  message.bounding_box_y = 20;
  message.bounding_box_width = 100;
  message.bounding_box_height = 150;

  message.orientation.x = 0;
  message.orientation.y = 0;
  message.orientation.z = 0;
  message.orientation.w = 0;
  
  face_info_pub_->publish(message);
  RCLCPP_INFO(this->get_logger(), "Published FaceInfo");
}