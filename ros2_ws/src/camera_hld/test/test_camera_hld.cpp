#include <gtest/gtest.h>
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <camera_hld/msg/face_info.hpp>
// #include <cv_bridge/cv_bridge.hpp>
// #include <opencv2/opencv.hpp>
// #include "camera_hld/camera_hld.hpp" 
#include "camera_hld.hpp"

class CameraHLDTest : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr test_node_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> image_pub_;
  std::shared_ptr<rclcpp::Subscription<camera_hld::msg::FaceInfo>> face_info_sub_;
  
  camera_hld::msg::FaceInfo received_msg_;
  bool msg_received_;

  void SetUp() override
  {
    // Initialize ROS 2
    rclcpp::init(0, nullptr);

    // Creëer een node voor de test
    test_node_ = std::make_shared<rclcpp::Node>("camera_hld_test_node");

    // Publisher instellen voor het 'raw_image' topic
    image_pub_ = test_node_->create_publisher<sensor_msgs::msg::Image>("raw_image", 10);

    // Subscriber instellen voor het 'face_info' topic
    face_info_sub_ = test_node_->create_subscription<camera_hld::msg::FaceInfo>(
      "face_info", 10,
      [this](const camera_hld::msg::FaceInfo::SharedPtr msg) {
        received_msg_ = *msg;
        msg_received_ = true;
      });

    msg_received_ = false;
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  // Helper functie om een testafbeelding te maken
  sensor_msgs::msg::Image::SharedPtr createTestImage()
  {
    cv::Mat test_image = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
    cv_bridge::CvImage cv_image;
    cv_image.image = test_image;
    cv_image.encoding = "bgr8";
    return cv_image.toImageMsg();
  }
};

// Test de initialisatie van de node
TEST_F(CameraHLDTest, NodeInitializationTest)
{
  auto camera_hld_node = std::make_shared<CameraHLD>();
  EXPECT_EQ(camera_hld_node->get_name(), "camera_hld_node");
}

// Test de functionaliteit van de pub-sub
TEST_F(CameraHLDTest, TestImageProcessingAndFaceInfoPublished)
{
  auto camera_hld_node = std::make_shared<CameraHLD>();

  // Maak een testafbeelding en publiceer deze op het raw_image topic
  auto test_image = createTestImage();
  image_pub_->publish(*test_image);

  // Spin om de callbacks te verwerken
  rclcpp::spin_some(camera_hld_node);
  rclcpp::spin_some(test_node_);

  // Controleer of het bericht ontvangen is
  EXPECT_TRUE(msg_received_);

  // Controleer de waarden van het ontvangen bericht
  EXPECT_EQ(received_msg_.bounding_box_x, 10);
  EXPECT_EQ(received_msg_.bounding_box_y, 20);
  EXPECT_EQ(received_msg_.bounding_box_width, 100);
  EXPECT_EQ(received_msg_.bounding_box_height, 150);
  EXPECT_EQ(received_msg_.orientation.x, 0);
  EXPECT_EQ(received_msg_.orientation.y, 0);
  EXPECT_EQ(received_msg_.orientation.z, 0);
  EXPECT_EQ(received_msg_.orientation.w, 0);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
