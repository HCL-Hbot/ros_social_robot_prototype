#include "eye_display_hld.hpp"

constexpr const char* DEFAULT_NODE_NAME = "eyes_hld_node";
constexpr const char* DEFAULT_TOPIC_NAME_SUB_LEFT_EYE = "left_eye_hld";
constexpr const char* DEFAULT_TOPIC_NAME_SUB_RIGHT_EYE = "right_eye_hld";
constexpr const char* DEFAULT_TOPIC_NAME_SUB_BOTH_EYE = "both_eye_hld";

constexpr const char* DEFAULT_TOPIC_NAME_PUB_LEFT_EYE = "left_eye_lld";
constexpr const char* DEFAULT_TOPIC_NAME_PUB_RIGHT_EYE = "right_eye_lld";

EyeDisplayHLD::EyeDisplayHLD() : 
    rclcpp::Node(DEFAULT_NODE_NAME),
    //current_expression_(eye_display_hld::msg::ScreenExpression::EYE_SLEEP),            
    left_eye_subscriber_(create_subscription<eye_display_hld::msg::EyeControl>(
            DEFAULT_TOPIC_NAME_SUB_LEFT_EYE, 10, std::bind(&EyeDisplayHLD::leftEyeCallback, this, std::placeholders::_1))),
    right_eye_subscriber_(create_subscription<eye_display_hld::msg::EyeControl>(
            DEFAULT_TOPIC_NAME_SUB_RIGHT_EYE, 10, std::bind(&EyeDisplayHLD::rightEyeCallback, this, std::placeholders::_1))),
    both_eyes_subscriber_(create_subscription<eye_display_hld::msg::EyeControl>(
            DEFAULT_TOPIC_NAME_SUB_BOTH_EYE, 10, std::bind(&EyeDisplayHLD::bothEyesCallback, this, std::placeholders::_1)))
            
{
    current_expression_.action = eye_display_hld::msg::ScreenExpression::EYE_SLEEP;
    RCLCPP_INFO(this->get_logger(), "EyesHLD node has been started.");
}

EyeDisplayHLD::~EyeDisplayHLD()
{
}

void EyeDisplayHLD::setExpression(const eye_display_hld::msg::ScreenExpression::SharedPtr expression)
{
     eye_display_hld::msg::ScreenExpression new_expression;
     new_expression.action = eye_display_hld::msg::ScreenExpression::EYE_SLEEP;
//   eye_display_hld::msg::ScreenExpression::SharedPtr new_expression = eye_display_hld::msg::ScreenExpression::EYE_AWAKE;
}

void EyeDisplayHLD::leftEyeCallback(const eye_display_hld::msg::EyeControl::SharedPtr msg)
{
    sendToLowLevelDriver(Eye::LEFT, msg);
}

void EyeDisplayHLD::rightEyeCallback(const eye_display_hld::msg::EyeControl::SharedPtr msg)
{
    sendToLowLevelDriver(Eye::RIGHT, msg);
}

void EyeDisplayHLD::bothEyesCallback(const eye_display_hld::msg::EyeControl::SharedPtr msg)
{
    sendToLowLevelDriver(Eye::BOTH, msg);
}

void EyeDisplayHLD::sendToLowLevelDriver(Eye eye, const eye_display_hld::msg::EyeControl::SharedPtr msg)
{
}
