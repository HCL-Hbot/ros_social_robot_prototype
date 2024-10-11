#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "camera_hld.hpp"

constexpr const char* DEFAULT_NODE_NAME = "camera_hld_node";
constexpr const char* DEFAULT_TOPIC_NAME_SUB = "raw_image";
constexpr const char* DEFAULT_TOPIC_NAME_PUB = "face_info";

constexpr const char* TF_CAMERA_FRAME_ID_PARAMETER = "tf_frame_id";

constexpr const char* DEFAULT_TF_CAMERA_FRAME_ID = "camera";

CameraHLD::CameraHLD() : 
  rclcpp::Node(DEFAULT_NODE_NAME),
  raw_image_sub_(create_subscription<sensor_msgs::msg::Image>(
            DEFAULT_TOPIC_NAME_SUB, 10, std::bind(&CameraHLD::imageCallback, this, std::placeholders::_1)))
{
  tf_frame_id_ = this->declare_parameter<std::string>(TF_CAMERA_FRAME_ID_PARAMETER, DEFAULT_TF_CAMERA_FRAME_ID);
  if(tf_frame_id_== DEFAULT_TF_CAMERA_FRAME_ID)
  {
    RCLCPP_INFO(this->get_logger(), "Default value for the parameter '%s' will be used for node '%s'",TF_CAMERA_FRAME_ID_PARAMETER, this->get_name());
  }

  face_info_pub_ = this->create_publisher<camera_hld::msg::FaceInfo>(DEFAULT_TOPIC_NAME_PUB, 10);
  RCLCPP_INFO(this->get_logger(), "Starting CameraHLD with node name: '%s' and '%s': '%s' ", this->get_name(), TF_CAMERA_FRAME_ID_PARAMETER, tf_frame_id_.c_str());
}

/*virtual*/ CameraHLD::~CameraHLD()
{
}

void CameraHLD::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Convert the ROS 2 image message to an OpenCV image
  cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
 
  // Display the image
  cv::imshow("Received Image frame id: " + tf_frame_id_, frame);
  cv::waitKey(1); // Wait for a key event to allow image display

  auto message = camera_hld::msg::FaceInfo();
  message.header.stamp = msg->header.stamp;
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
}