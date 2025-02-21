#ifndef INTERACTION_CONTROLLER_INCLUDE_INTERACTION_CONTROLLER_INTERACTION_CONTROLLER_HPP_
#define INTERACTION_CONTROLLER_INCLUDE_INTERACTION_CONTROLLER_INTERACTION_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "eye_display_hld/msg/eye_control.hpp"
#include "interaction_controller/msg/presence_detection.hpp"
#include "eye_display_hld/msg/screen_expression.hpp"

class InteractionController : public rclcpp::Node
{
public:
  InteractionController();
  virtual ~InteractionController();
private:
    void facePositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr face_position);

    eye_display_hld::msg::EyeControl convertFacePositionToEyeControl(const geometry_msgs::msg::PointStamped::SharedPtr& face_position);

    void radarPresenceCallback(const interaction_controller::msg::PresenceDetection::SharedPtr presence_msg);  
    
    bool isPresenceStateChanged(const interaction_controller::msg::PresenceDetection::SharedPtr& presence_msg);

    eye_display_hld::msg::ScreenExpression::SharedPtr convertPresenceDetectionToScreenExpression(const interaction_controller::msg::PresenceDetection::SharedPtr& presence_msg);

    void updateLastPresenceDetection(const interaction_controller::msg::PresenceDetection::SharedPtr& presence_msg);
    
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr face_position_sub_;
    rclcpp::Publisher<eye_display_hld::msg::EyeControl>::SharedPtr eye_control_pub_;
    rclcpp::Subscription<interaction_controller::msg::PresenceDetection>::SharedPtr radar_presence_sub_;
    rclcpp::Publisher<eye_display_hld::msg::ScreenExpression>::SharedPtr screen_expression_pub_;
    interaction_controller::msg::PresenceDetection last_precence_msg_;
};


#endif // INTERACTION_CONTROLLER_INCLUDE_INTERACTION_CONTROLLER_INTERACTION_CONTROLLER_HPP_