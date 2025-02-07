#include "eye_display_hld.hpp"

constexpr const char* DEFAULT_NODE_NAME = "eyes_hld_node";
constexpr const char* DEFAULT_TOPIC_NAME_SUB_BOTH_EYE = "eye_control";

constexpr const char* DEFAULT_TOPIC_NAME_PUB_LEFT_EYE = "left_eye_lld";
constexpr const char* DEFAULT_TOPIC_NAME_PUB_RIGHT_EYE = "right_eye_lld";

EyeDisplayHLD::EyeDisplayHLD() : 
    rclcpp::Node(DEFAULT_NODE_NAME),
    both_eyes_subscriber_(create_subscription<eye_display_hld::msg::EyeControl>(
            DEFAULT_TOPIC_NAME_SUB_BOTH_EYE, 10, std::bind(&EyeDisplayHLD::bothEyesCallback, this, std::placeholders::_1)))
            
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
}
