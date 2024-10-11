#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "camera_lld.hpp"

constexpr const char* DEFAULT_NODE_NAME = "camera_lld_node";
constexpr const char* DEFAULT_TOPIC_NAME = "raw_image";

CameraLLD::CameraLLD() :
  rclcpp::Node(DEFAULT_NODE_NAME),
  raw_image_pub_(this->create_publisher<sensor_msgs::msg::Image>(DEFAULT_TOPIC_NAME, 10))
  //camera_thread_(std::thread(std::bind(&CameraLLD::captureAndPublish,this)))
{
  if(init())
  {
    camera_thread_= std::thread(std::bind(&CameraLLD::captureAndPublish,this));
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "All parameters not configured for the node '%s'. Node will be aborted.",this->get_name());
    rclcpp::shutdown();
  }
}

/*virtual*/ CameraLLD::~CameraLLD()
{
  if (camera_thread_.joinable())
  {
    camera_thread_.join();
  }
}

bool CameraLLD::init()
{
  bool init_result = true;
  return init_result;
}

void CameraLLD::captureAndPublish()
{
    cv::VideoCapture cap(0);  // Open any camera
    if (!cap.isOpened())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
      return;
    }

    std_msgs::msg::Header header;
    
    while (rclcpp::ok()) 
    {
      cv::Mat frame;
      cap >> frame; //read frame
      if (!frame.empty())
      {
        header.stamp = this->get_clock()->now();
        auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        raw_image_pub_->publish(*msg);  // publish frame
      }
    }
}