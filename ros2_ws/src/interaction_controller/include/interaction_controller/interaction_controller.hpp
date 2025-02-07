#ifndef INTERACTION_CONTROLLER_INCLUDE_INTERACTION_CONTROLLER_INTERACTION_CONTROLLER_HPP_
#define INTERACTION_CONTROLLER_INCLUDE_INTERACTION_CONTROLLER_INTERACTION_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "eye_display_hld/msg/eye_control.hpp"

class InteractionController : public rclcpp::Node
{
public:
  InteractionController();
  virtual ~InteractionController();
private:
    void facePositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    eye_display_hld::msg::EyeControl convertFacePositionToEyeControl(const geometry_msgs::msg::PointStamped::SharedPtr& face_position);
    
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr face_position_;
    rclcpp::Publisher<eye_display_hld::msg::EyeControl>::SharedPtr eye_control_pub_;

};


#endif // INTERACTION_CONTROLLER_INCLUDE_INTERACTION_CONTROLLER_INTERACTION_CONTROLLER_HPP_