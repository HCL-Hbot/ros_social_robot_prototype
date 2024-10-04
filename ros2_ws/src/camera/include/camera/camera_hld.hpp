/**
 * @file camera_hld.hpp
 * @brief high level driver of a camera
 * 
 * This class will read recognize a face from a raw image
 *
 * @author Agit
 * @date 2024-10-04
 * @version 1.0
 * 
 * @license see license file in this package
 * Copyright (c) 2024 Agit - HCL
 */

#ifndef CAMERA_INCLUDE_CAMERA_CAMERA_HLD_HPP_
#define CAMERA_INCLUDE_CAMERA_CAMERA_HLD_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraHLD : public rclcpp::Node
{
public:
  CameraHLD(const std::string& node_name);
  virtual ~CameraHLD();
private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

#endif // CAMERA_INCLUDE_CAMERA_CAMERA_HLD_HPP_