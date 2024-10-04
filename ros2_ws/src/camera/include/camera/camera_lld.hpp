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

/**
 * @brief Low level driver for a camera 
 * 
 * @todo Maybe add configuration to choose from a usb port? 
 *       Currently it will search for any camera. Would be problem if you have multiple camera's attached.
 */
class CameraLLD : public rclcpp::Node
{
public:
  /**
  * @brief Construct a new Camera LLD object
  * 
  * @param node_name Name of the low level driver node.
  */
  CameraLLD(const std::string& node_name);

  /**
   * @brief Destroy the Camera LLD object
   */
  virtual ~CameraLLD();
  
private:

  /**
   * @brief capture a frame from the camera and publish it to a topic.
   */
  void captureAndPublish();
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  std::thread camera_thread_;
};

#endif // CAMERA_INCLUDE_CAMERA_CAMERA_LLD_HPP_