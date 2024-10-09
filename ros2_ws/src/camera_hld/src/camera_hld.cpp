#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "camera_hld.hpp"

CameraHLD::CameraHLD(const std::string& node_name) : 
  rclcpp::Node(node_name),
  raw_image_sub_(create_subscription<sensor_msgs::msg::Image>(
            "raw_image", 10, std::bind(&CameraHLD::imageCallback, this, std::placeholders::_1)))
{
  face_info_pub_ = this->create_publisher<camera_hld::msg::FaceInfo>("face_info_topic", 10);
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
  message.header.frame_id = "camera_voor_aanzicht_frame"; //TODO uit configurtie halen.
  message.bounding_box_x = 10;
  message.bounding_box_y = 20;
  message.bounding_box_width = 100;
  message.bounding_box_height = 150;

  message.orientation.x = 0;
  message.orientation.y = 0;
  message.orientation.z = 0;
  message.orientation.w = 0;
  
  // Vul andere velden van de message in zoals nodig.
  face_info_pub_->publish(message);
  RCLCPP_INFO(this->get_logger(), "Published FaceInfo");
}