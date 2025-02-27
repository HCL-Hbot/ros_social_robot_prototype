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

#include "geometry_msgs/msg/point_stamped.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <face_detection.hpp>
#include <iris_mesh.hpp>

namespace camera_hld {
/**
 * @brief High level driver for a camera
 * This class will detect a face from a image and publish it coordintes to a topic.
 * 
 * The following parameters are configurable via 
 * @param node_name The name for this node, default value is "camera_hld"
 * 
 * @example The following example shows how to 
 *          "ros2 run camera_hld camera_hld_node --ros-args --remap __node:=new_node_name"
 * 
 * @see See the SDD documentation how to configure the above parameters (via "/parameters_events")
 */
class CameraHLD : public rclcpp::Node {
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
  
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg);

  cv::Mat convertImageMsgToCvMat(const sensor_msgs::msg::Image::SharedPtr image_msg);

  void publishFacePosition(const cv::Mat& frame);
  
  cv::Point getCenterOfFace(const cv::Rect& face_roi);

  float getDistanceToFace(const cv::Rect& face_roi);
  
  geometry_msgs::msg::PointStamped createFacePositionMsg(const cv::Point center_of_face, float distance_to_face);

  /**
   * @brief Calculate the eye regions on the face by using two 2D-facial landmarks of the eye_center.
   *        It uses the distance between the eye's as reference for how big the regions should be.
   *        For most people it works, but it is a very crude and easy way!
   */
  const std::array<cv::Rect, 2> calculateEyeRoi(const cv::Point &left_eye_landmark, const cv::Point &right_eye_landmark);

  float getBiggestIrisDiameterInPixel(const std::array<cv::Rect, 2> &eye_rois, const cv::Mat& frame);

  float getIrisDiameterInPixel(const std::array<cv::Point3f, CLFML::IrisMesh::NUM_OF_IRIS_MESH_POINTS> iris_mesh_landmarks);

  /**
   * @brief Check if the given ROI is within the bounds of the image.
   * 
   * @param roi The region of interest to check.
   * @param image The image in which the ROI should be checked.
   * @return true if the ROI is within the bounds of the image, false otherwise.
   */
  bool is_roi_within_bounds(const cv::Rect &roi, const cv::Mat &image);


  void publishDebugImage(const cv::Mat& frame);
  
  CLFML::FaceDetection::FaceDetector face_detector_;
  CLFML::IrisMesh::IrisMesh iris_mesh_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr face_position_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

  std::string tf_frame_id_;
};

}  // namespace camera_hld

#endif // CAMERA_HLD_INCLUDE_CAMERA_HLD_CAMERA_HLD_HPP_