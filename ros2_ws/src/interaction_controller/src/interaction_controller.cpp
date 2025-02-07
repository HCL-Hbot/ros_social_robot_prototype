#include "interaction_controller.hpp"

constexpr const char* DEFAULT_NODE_NAME = "interaction_controller_node";
constexpr const char* DEFAULT_TOPIC_NAME_SUB_FACE = "face_info";
constexpr const char* DEFAULT_TOPIC_NAME_PUB_EYE_CONTROL = "eye_control";

InteractionController::InteractionController() :
    rclcpp::Node(DEFAULT_NODE_NAME),
    face_position_(create_subscription<geometry_msgs::msg::PointStamped>(
        DEFAULT_TOPIC_NAME_SUB_FACE, 10, std::bind(&InteractionController::facePositionCallback, this, std::placeholders::_1))),
    eye_control_pub_(create_publisher<eye_display_hld::msg::EyeControl>(DEFAULT_TOPIC_NAME_PUB_EYE_CONTROL, 10))
{
}

InteractionController::~InteractionController()
{
}

void InteractionController::facePositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    eye_display_hld::msg::EyeControl eye_control_msg = convertFacePositionToEyeControl(msg);
    eye_control_pub_->publish(eye_control_msg);
}

eye_display_hld::msg::EyeControl InteractionController::convertFacePositionToEyeControl(const geometry_msgs::msg::PointStamped::SharedPtr& face_position)
{
    eye_display_hld::msg::EyeControl eye_control_msg = eye_display_hld::msg::EyeControl();

    //Face position is in the camera frame, so we need to convert it to the eye frame.
    //Coordinates of face position are defined in ROS Right-Hand Rule

    //Todo convert face position to eye position with tf frames
    // TODO better implementations
    //------------------------------------------------------------------------------------------------
    
    // Quick implementation!    
    //for now assume the face position frame is the same as the eye frame for testing purposes
    //Origin is (0,0,0) at the center of the camera frame
    //x = forward (distance to camera) , y = left/right, z = up/down
  
    // Calculate yaw (horizontal angle) in degrees
    eye_control_msg.yaw = std::atan2(face_position->point.y, face_position->point.x) * 180.0 / M_PI;
    // Calculate pitch (vertical angle) in degrees
    eye_control_msg.pitch = std::atan2(face_position->point.z, face_position->point.x) * 180.0 / M_PI;
    // Calculate distance to face in cm
    eye_control_msg.target_distance_cm = face_position->point.x;

    //Of moet ik de distance zo berekenen? Probleem hierbij is x is cm, y is pixels en z is pixels
    //eye_control_msg.target_distance_cm = std::sqrt(std::pow(face_position->point.x, 2) + std::pow(face_position->point.y, 2) + std::pow(face_position->point.z, 2));
    //------------------------------------------------------------------------------------------------

    return eye_control_msg;
}
