#include "eye_display_hld.hpp"
#include <algorithm> // For std::clamp

constexpr const char* DEFAULT_NODE_NAME = "eyes_hld_node";
constexpr const char* DEFAULT_TOPIC_NAME_SUB_BOTH_EYE = "eye_control";
constexpr const char* DEFAULT_TOPIC_NAME_PUB_PUPIL_CONTROL = "pupil_control";
constexpr const char* DEFAULT_TOPIC_NAME_PUB_EYES_DIRECTION = "eyes_direction_control";
constexpr const char* DEFAULT_TOPIC_NAME_SUB_SCREEN_EXPRESSION = "screen_expression";
constexpr const char* DEFAULT_TOPIC_NAME_PUB_EYE_LID_CONTROL = "eye_lid_control";
//constexpr const char* DEFAULT_TOPIC_NAME_PUB_LEFT_EYE = "left_eye_lld";
//constexpr const char* DEFAULT_TOPIC_NAME_PUB_RIGHT_EYE = "right_eye_lld";


constexpr double MIN_DISTANCE = 30.0f;   // Minimum distance (max percentage)
constexpr double MAX_DISTANCE = 100.0f;  // Maximum distance (min percentage)
constexpr double MIN_EYE_PERCENTAGE = 50.0f; // Percentage at maxDistance and above
constexpr double MAX_EYE_PERCENTAGE = 90.0f; // Percentage at minDistance and below

// Function to calculate the percentage based on distance
constexpr static double calculatePercentage(double distance, double minDistance, double maxDistance, double minPercentage, double maxPercentage) {
    if (distance >= maxDistance) return minPercentage; // Above maxDistance -> minPercentage
    if (distance <= minDistance) return maxPercentage; // Below minDistance -> maxPercentage

    // Linear interpolation
    double slope = (minPercentage - maxPercentage) / (maxDistance - minDistance);
    double intercept = maxPercentage - (slope * minDistance);

    return slope * distance + intercept;
}

namespace eye_display_hld {

EyeDisplayHLD::EyeDisplayHLD() 
: rclcpp::Node(DEFAULT_NODE_NAME),
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


double EyeDisplayHLD::getPupilDialation(uint16_t target_distance_cm) 
{
    //double pupil_conversion = (25.0/100.0); //Verhouding van 0 tot 100. Bij een afstand 100 cm 25% pupil dialation
    //return target_distance_cm * pupil_conversion;
    double pupil_dialation = calculatePercentage(target_distance_cm, MIN_DISTANCE, MAX_DISTANCE, MIN_EYE_PERCENTAGE, MAX_EYE_PERCENTAGE);
    return pupil_dialation;
}

eye_display_lld::msg::EyesDirection EyeDisplayHLD::getEyeDirectionMsg(double yaw, double pitch)
{
    eye_display_lld::msg::EyesDirection eyes_direction_msg;
    eyes_direction_msg.yaw = yaw * 180.0 / M_PI;
    eyes_direction_msg.pitch = pitch * 180.0 / M_PI;
    return eyes_direction_msg;
}

void EyeDisplayHLD::screenExpressionCallback(const eye_display_hld::msg::ScreenExpression::SharedPtr screen_expression_msg)
{
    eye_display_lld::msg::EyeLidControl eye_lid_msg;
    if(screen_expression_msg->action == eye_display_hld::msg::ScreenExpression::EYE_AWAKE) {
        eye_lid_msg.eye_id = eye_display_lld::msg::EyeLidControl::BOTH_EYES;
        eye_lid_msg.top_lid_position = 0;
        eye_lid_msg.bottom_lid_position = 0;
        eye_lid_publisher_->publish(eye_lid_msg);
    }
    else if (screen_expression_msg->action == eye_display_hld::msg::ScreenExpression::EYE_SLEEP) {
        eye_lid_msg.eye_id = eye_display_lld::msg::EyeLidControl::BOTH_EYES;
        eye_lid_msg.top_lid_position = 50;
        eye_lid_msg.bottom_lid_position = 50;
        eye_lid_publisher_->publish(eye_lid_msg);
    }
}

} // namespace eye_display_hld
