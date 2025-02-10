#include "eye_display_hld.hpp"

constexpr const char* DEFAULT_NODE_NAME = "eyes_hld_node";
constexpr const char* DEFAULT_TOPIC_NAME_SUB_BOTH_EYE = "eye_control";
constexpr const char* DEFAULT_TOPIC_NAME_PUB_PUPIL_CONTROL = "pupil_control";
constexpr const char* DEFAULT_TOPIC_NAME_PUB_EYES_DIRECTION = "eyes_direction_control";
//constexpr const char* DEFAULT_TOPIC_NAME_PUB_LEFT_EYE = "left_eye_lld";
//constexpr const char* DEFAULT_TOPIC_NAME_PUB_RIGHT_EYE = "right_eye_lld";

EyeDisplayHLD::EyeDisplayHLD() : 
    rclcpp::Node(DEFAULT_NODE_NAME),
    both_eyes_subscriber_(create_subscription<eye_display_hld::msg::EyeControl>(
            DEFAULT_TOPIC_NAME_SUB_BOTH_EYE, 10, std::bind(&EyeDisplayHLD::bothEyesCallback, this, std::placeholders::_1))),
    pupil_control_publisher_(this->create_publisher<eye_display_lld::msg::PupilControl>(DEFAULT_TOPIC_NAME_PUB_PUPIL_CONTROL, 10)),
    eyes_direction_publisher_(this->create_publisher<eye_display_lld::msg::EyesDirection>(DEFAULT_TOPIC_NAME_PUB_EYES_DIRECTION, 10))
            
{
    RCLCPP_INFO(this->get_logger(), "EyesHLD node has been started.");
}

EyeDisplayHLD::~EyeDisplayHLD()
{
}


void EyeDisplayHLD::bothEyesCallback(const eye_display_hld::msg::EyeControl::SharedPtr msg)
{
    sendToLowLevelDriver(Eye::BOTH, msg);
}

void EyeDisplayHLD::sendToLowLevelDriver(Eye eye, const eye_display_hld::msg::EyeControl::SharedPtr msg)
{
    eye_display_lld::msg::PupilControl pupil_control_msg;
    float pupil_conversion = (25.0f/100.0f); //Verhouding van 0 tot 100. Bij een afstand 100 cm 25% pupil dialation
    pupil_control_msg.dilation_percentage = msg->target_distance_cm * pupil_conversion;
    
    eye_display_lld::msg::EyesDirection eyes_direction_msg;
    eyes_direction_msg.yaw = msg->yaw;
    eyes_direction_msg.pitch = msg->pitch;
    
    eyes_direction_publisher_->publish(eyes_direction_msg);
    pupil_control_publisher_->publish(pupil_control_msg);
}