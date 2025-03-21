#ifndef INTERACTION_CONTROLLER_INCLUDE_INTERACTION_CONTROLLER_INTERACTION_CONTROLLER_HPP_
#define INTERACTION_CONTROLLER_INCLUDE_INTERACTION_CONTROLLER_INTERACTION_CONTROLLER_HPP_

#include "eye_display_hld/msg/eye_control.hpp"
#include "interaction_controller/msg/presence_detection.hpp"
#include "eye_display_hld/msg/screen_expression.hpp"
#include "audio_hld/msg/sound_command.hpp"
#include "audio_hld/action/play_sound.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <functional>
#include <future>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace interaction_controller {
    
class InteractionController : public rclcpp::Node
{
public:
  using PlaySound = audio_hld::action::PlaySound;
  using GoalHandlePlaySound = rclcpp_action::ClientGoalHandle<PlaySound>;

  InteractionController();
  virtual ~InteractionController();
private:
    void facePositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr face_position);

    eye_display_hld::msg::EyeControl convertFacePositionToEyeControl(const geometry_msgs::msg::PointStamped::SharedPtr& face_position);

    void radarPresenceCallback(const interaction_controller::msg::PresenceDetection::SharedPtr presence_msg);  
    
    bool isPresenceStateChanged(const interaction_controller::msg::PresenceDetection::SharedPtr& presence_msg);

    eye_display_hld::msg::ScreenExpression::SharedPtr convertPresenceDetectionToScreenExpression(const interaction_controller::msg::PresenceDetection::SharedPtr& presence_msg);

    void updateLastPresenceDetection(const interaction_controller::msg::PresenceDetection::SharedPtr& presence_msg);

    void send_goal();
    
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr face_position_sub_;
    rclcpp::Publisher<eye_display_hld::msg::EyeControl>::SharedPtr eye_control_pub_;
    rclcpp::Subscription<interaction_controller::msg::PresenceDetection>::SharedPtr radar_presence_sub_;
    rclcpp::Publisher<eye_display_hld::msg::ScreenExpression>::SharedPtr screen_expression_pub_;
    interaction_controller::msg::PresenceDetection last_precence_msg_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp_action::Client<PlaySound>::SharedPtr client_ptr_;

};

}  // namespace interaction_controller

#endif // INTERACTION_CONTROLLER_INCLUDE_INTERACTION_CONTROLLER_INTERACTION_CONTROLLER_HPP_