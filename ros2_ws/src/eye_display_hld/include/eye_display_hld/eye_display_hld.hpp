/**
 * @file eye_display_hld.hpp
 * @author Agit (a.tunc1@student.han.nl)
 * @brief high level driver of eye display
 * 
 * This class will implement the bevaviour of the eyes
 * 
 * @version 0.1
 * @date 2025-01-31
 * 
 * @license see license file in this package
 * @copyright Copyright (c) 2025 Agit - HCL
 * 
 */


#ifndef EYE_DISPLAY_HLD_INCLUDE_EYE_DISPLAY_HLD_EYE_DISPLAY_HLD_HPP_
#define EYE_DISPLAY_HLD_INCLUDE_EYE_DISPLAY_HLD_EYE_DISPLAY_HLD_HPP_

#include <rclcpp/rclcpp.hpp>
#include "eye_display_hld/msg/eye_control.hpp"
#include "eye_display_hld/msg/screen_expression.hpp"
#include "eye_display_lld/msg/pupil_control.hpp"
#include "eye_display_lld/msg/eyes_direction.hpp"
#include "eye_display_lld/msg/eye_lid_control.hpp"
class EyeDisplayHLD : public rclcpp::Node
{
public:

    enum class Eye : uint8_t
    {
        LEFT,
        RIGHT,
        BOTH
    };

    EyeDisplayHLD();
    
    virtual ~EyeDisplayHLD();

private:
    void bothEyesCallback(const eye_display_hld::msg::EyeControl::SharedPtr msg);
    void sendToLowLevelDriver(Eye eye, const eye_display_hld::msg::EyeControl::SharedPtr msg);
    void screenExpressionCallback(const eye_display_hld::msg::ScreenExpression::SharedPtr msg);
    rclcpp::Subscription<eye_display_hld::msg::EyeControl>::SharedPtr both_eyes_subscriber_;
    rclcpp::Subscription<eye_display_hld::msg::ScreenExpression>::SharedPtr screen_expression_;
    rclcpp::Publisher<eye_display_lld::msg::PupilControl>::SharedPtr pupil_control_publisher_; //also for both eyes
    rclcpp::Publisher<eye_display_lld::msg::EyesDirection>::SharedPtr eyes_direction_publisher_;
    rclcpp::Publisher<eye_display_lld::msg::EyeLidControl>::SharedPtr eye_lid_publisher_;

};

#endif // EYE_DISPLAY_HLD_INCLUDE_EYE_DISPLAY_HLD_EYE_DISPLAY_HLD_HPP_