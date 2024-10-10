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

#ifndef CAMERA_HLD_INCLUDE_CAMERA_HLD_CAMERA_HLD_HPP_
#define CAMERA_HLD_INCLUDE_CAMERA_HLD_CAMERA_HLD_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "camera_hld/msg/face_info.hpp"

/**
 * @brief High level driver for a camera
 * This class will detect a face from a image and publish it coordintes to a topic.
 * 
 * The following parameters are configurable via 
 * @param node_name The name for this node, default value is "camera_hld"
 * @param frame_id The frame id name for this publishing node (ex. "camera_vooraanzicht_linker_oog")
 * 
 * @example The following example shows how to 
 *          "ros2 run camera_hld camera_hld_node --ros-args --remap __node:=new_node_name -p frame_id:=camera_link"
 * 
 * @see See the SDD documentation how to configure the above parameters (via "/parameters_events")
 */
class CameraHLD : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Camera HLD object
   */
  CameraHLD();

  /**
   * @brief Destroy the Camera HLD object
   */
  virtual ~CameraHLD();

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_sub_;
  rclcpp::Publisher<camera_hld::msg::FaceInfo>::SharedPtr face_info_pub_;
  std::string tf_frame_id_;
};

#endif // CAMERA_HLD_INCLUDE_CAMERA_HLD_CAMERA_HLD_HPP_