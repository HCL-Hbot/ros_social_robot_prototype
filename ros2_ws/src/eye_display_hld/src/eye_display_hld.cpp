#include "eye_display_hld.hpp"

constexpr const char* DEFAULT_NODE_NAME = "eyes_hld_node";
constexpr const char* DEFAULT_TOPIC_NAME_SUB_BOTH_EYE = "eye_control";
constexpr const char* DEFAULT_TOPIC_NAME_PUB_PUPIL_CONTROL = "pupil_control";
constexpr const char* DEFAULT_TOPIC_NAME_PUB_EYES_DIRECTION = "eyes_direction_control";
constexpr const char* DEFAULT_TOPIC_NAME_SUB_SCREEN_EXPRESSION = "screen_expression";
constexpr const char* DEFAULT_TOPIC_NAME_PUB_EYE_LID_CONTROL = "eye_lid_control";
//constexpr const char* DEFAULT_TOPIC_NAME_PUB_LEFT_EYE = "left_eye_lld";
//constexpr const char* DEFAULT_TOPIC_NAME_PUB_RIGHT_EYE = "right_eye_lld";

EyeDisplayHLD::EyeDisplayHLD() : 
    rclcpp::Node(DEFAULT_NODE_NAME),
    both_eyes_subscriber_(create_subscription<eye_display_hld::msg::EyeControl>(
            DEFAULT_TOPIC_NAME_SUB_BOTH_EYE, 10, std::bind(&EyeDisplayHLD::bothEyesCallback, this, std::placeholders::_1))),
    screen_expression_subscriber_(create_subscription<eye_display_hld::msg::ScreenExpression>(
            DEFAULT_TOPIC_NAME_SUB_SCREEN_EXPRESSION, 10, std::bind(&EyeDisplayHLD::screenExpressionCallback, this, std::placeholders::_1))),      
    pupil_control_publisher_(this->create_publisher<eye_display_lld::msg::PupilControl>(DEFAULT_TOPIC_NAME_PUB_PUPIL_CONTROL, 10)),
    eyes_direction_publisher_(this->create_publisher<eye_display_lld::msg::EyesDirection>(DEFAULT_TOPIC_NAME_PUB_EYES_DIRECTION, 10)),
    eye_lid_publisher_(this->create_publisher<eye_display_lld::msg::EyeLidControl>(DEFAULT_TOPIC_NAME_PUB_EYE_LID_CONTROL, 10))
            
{
    RCLCPP_INFO(this->get_logger(), "EyesHLD node has been started.");
}

EyeDisplayHLD::~EyeDisplayHLD()
{
}


void EyeDisplayHLD::bothEyesCallback(const eye_display_hld::msg::EyeControl::SharedPtr eye_control_msg)
{
    sendToLowLevelDriver(Eye::BOTH, eye_control_msg);
}

void EyeDisplayHLD::sendToLowLevelDriver(Eye eye, const eye_display_hld::msg::EyeControl::SharedPtr& eye_control_msg)
{
    eye_display_lld::msg::PupilControl pupil_control_msg = eye_display_lld::msg::PupilControl();
    pupil_control_msg.dilation_percentage = getPupilDialation(eye_control_msg->target_distance_cm);

    eye_display_lld::msg::EyesDirection eyes_direction_msg = getEyeDirectionMsg(eye_control_msg->yaw, eye_control_msg->pitch);

    eyes_direction_publisher_->publish(eyes_direction_msg);
    pupil_control_publisher_->publish(pupil_control_msg);
}


float EyeDisplayHLD::getPupilDialation(uint16_t target_distance_cm)
{
    float pupil_conversion = (25.0f/100.0f); //Verhouding van 0 tot 100. Bij een afstand 100 cm 25% pupil dialation
    return target_distance_cm * pupil_conversion;
}

eye_display_lld::msg::EyesDirection EyeDisplayHLD::getEyeDirectionMsg(float yaw, float pitch)
{
    eye_display_lld::msg::EyesDirection eyes_direction_msg;
    eyes_direction_msg.yaw = yaw;
    eyes_direction_msg.pitch = pitch;
    return eyes_direction_msg;
}

void EyeDisplayHLD::screenExpressionCallback(const eye_display_hld::msg::ScreenExpression::SharedPtr screen_expression_msg)
{
    eye_display_lld::msg::EyeLidControl eye_lid_msg;
    if(screen_expression_msg->action == eye_display_hld::msg::ScreenExpression::EYE_AWAKE)
    {
        eye_lid_msg.eye_id = eye_display_lld::msg::EyeLidControl::BOTH_EYES;
        eye_lid_msg.top_lid_position = 0;
        eye_lid_msg.bottom_lid_position = 0;
        eye_lid_publisher_->publish(eye_lid_msg);
    }
    else if (screen_expression_msg->action == eye_display_hld::msg::ScreenExpression::EYE_SLEEP)
    {
        eye_lid_msg.eye_id = eye_display_lld::msg::EyeLidControl::BOTH_EYES;
        eye_lid_msg.top_lid_position = 50;
        eye_lid_msg.bottom_lid_position = 50;
        eye_lid_publisher_->publish(eye_lid_msg);
    }
}
