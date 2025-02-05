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

class EyeDisplayHLD : public rclcpp::Node
{
public:

    enum class Eye : uint8_t
    {
        LEFT,
        RIGHT,
        BOTH
    };

    // enum class Expression : uint8_t
    // {
    //     NONE,
    //     STARING,
    //     TRACKING,
    //     GRIN,
    //     SAD,
    //     LOOK_RANDOM,
    //     NEUTRAL
    // };

    EyeDisplayHLD();
    
    virtual ~EyeDisplayHLD();

    void setExpression(const eye_display_hld::msg::ScreenExpression::SharedPtr expression);

private:
    void leftEyeCallback(const eye_display_hld::msg::EyeControl::SharedPtr msg);
    void rightEyeCallback(const eye_display_hld::msg::EyeControl::SharedPtr msg);
    void bothEyesCallback(const eye_display_hld::msg::EyeControl::SharedPtr msg);
    
    void sendToLowLevelDriver(Eye eye, const eye_display_hld::msg::EyeControl::SharedPtr msg);

    eye_display_hld::msg::ScreenExpression current_expression_;
    rclcpp::Subscription<eye_display_hld::msg::EyeControl>::SharedPtr left_eye_subscriber_;
    rclcpp::Subscription<eye_display_hld::msg::EyeControl>::SharedPtr right_eye_subscriber_;
    rclcpp::Subscription<eye_display_hld::msg::EyeControl>::SharedPtr both_eyes_subscriber_;
};

#endif // EYE_DISPLAY_HLD_INCLUDE_EYE_DISPLAY_HLD_EYE_DISPLAY_HLD_HPP_