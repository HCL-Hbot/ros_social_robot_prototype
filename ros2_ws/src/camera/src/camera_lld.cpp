#include <cv_bridge/cv_bridge.hpp>
#include "camera_lld.hpp"


CameraLLD::CameraLLD(const std::string& node_name) :
  rclcpp::Node(node_name),
  publisher_(this->create_publisher<sensor_msgs::msg::Image>("raw_image", 10)),
  timer_(this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&CameraLLD::publishImage, this))) 
{
  cap_.open(0); // Open the default camera (camera index 0)
  if (!cap_.isOpened()) 
  {
    RCLCPP_ERROR(this->get_logger(), "Could not open camera");
  }
}

/*virtual*/ CameraLLD::~CameraLLD()
{
}

void CameraLLD::publishImage()
{
  cv::Mat frame;
  cap_ >> frame; // Capture a frame from the camera

  if (!frame.empty())
  {
    // Convert OpenCV image (cv::Mat) to ROS 2 image message
    std::shared_ptr<sensor_msgs::msg::Image> msg = cv_bridge::CvImage(
        std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    
    publisher_->publish(*msg);
  }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraLLD>("camera_lld"));
    rclcpp::shutdown();
    return 0;
}
