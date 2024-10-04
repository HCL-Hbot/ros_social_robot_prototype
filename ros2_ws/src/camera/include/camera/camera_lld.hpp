/**
 * @file camera_lld.hpp
 * @brief Low level driver of a camera
 * 
 * This class will read image from camera and propegate this to the high level driver
 *
 * @author Agit
 * @date 2024-10-04
 * @version 1.0
 * 
 * @license see license file in this package
 * Copyright (c) 2024 Agit - HCL
 */

#ifndef CAMERA_INCLUDE_CAMERA_CAMERA_LLD_HPP_
#define CAMERA_INCLUDE_CAMERA_CAMERA_LLD_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

class CameraLLD : public rclcpp::Node
{
public:
  CameraLLD(const std::string& node_name);
  virtual ~CameraLLD();
private:
  void publishImage();
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
};

#endif // CAMERA_INCLUDE_CAMERA_CAMERA_LLD_HPP_