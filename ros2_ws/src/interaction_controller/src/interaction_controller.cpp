#include "interaction_controller.hpp"


constexpr const char* DEFAULT_NODE_NAME = "interaction_controller_node";
constexpr const char* DEFAULT_TOPIC_NAME_SUB_FACE = "face_info";
constexpr const char* DEFAULT_TOPIC_NAME_PUB_EYE_CONTROL = "eye_control";
constexpr const char* DEFAULT_TOPIC_NAME_SUB_RADAR_PRESENCE_HL = "radar_presence";
constexpr const char* DEFAULT_TOPIC_NAME_PUB_SCREEN_EXPRESSION = "screen_expression";
constexpr const char* BETWEEN_EYES_FRAME = "robot_eyes_between";
constexpr const uint8_t ROBOT_FACE_RADIUS_CM = 10;
constexpr const double ROBOT_FACE_RADIUS_M = static_cast<double>(ROBOT_FACE_RADIUS_CM) / 100.0f;

namespace interaction_controller {
    
InteractionController::InteractionController() 
: rclcpp::Node(DEFAULT_NODE_NAME),
  face_position_sub_(create_subscription<geometry_msgs::msg::PointStamped>(
        DEFAULT_TOPIC_NAME_SUB_FACE, 10, std::bind(&InteractionController::facePositionCallback, this, std::placeholders::_1))),
  eye_control_pub_(create_publisher<eye_display_hld::msg::EyeControl>(DEFAULT_TOPIC_NAME_PUB_EYE_CONTROL, 10)),
  radar_presence_sub_(create_subscription<interaction_controller::msg::PresenceDetection>(
        DEFAULT_TOPIC_NAME_SUB_RADAR_PRESENCE_HL, 10, std::bind(&InteractionController::radarPresenceCallback, this, std::placeholders::_1))),
  screen_expression_pub_(this->create_publisher<eye_display_hld::msg::ScreenExpression>(DEFAULT_TOPIC_NAME_PUB_SCREEN_EXPRESSION, 10)),
  last_precence_msg_(),
  previous_face_position_(),
  greet_(false)
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    last_precence_msg_.presence_state = interaction_controller::msg::PresenceDetection::TARGET_OUT_OF_RANGE;
    last_precence_msg_.target_state = interaction_controller::msg::PresenceDetection::TARGET_STANDING;


    this->client_ptr_ = rclcpp_action::create_client<PlaySound>(this,"/play_sound");

    RCLCPP_INFO(this->get_logger(), "InteractionController started");
}

InteractionController::~InteractionController()
{
}

void InteractionController::facePositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr face_position) {
    try {
        eye_display_hld::msg::EyeControl eye_control_msg = convertFacePositionToEyeControl(face_position);
        eye_control_pub_->publish(eye_control_msg);
        if(isFacePositionInFrontOfRobotStationary(face_position)) {
            RCLCPP_INFO(this->get_logger(), "Face is in front of robot and stationary");
            if(greet_ && last_precence_msg_.presence_state == interaction_controller::msg::PresenceDetection::TARGET_IN_RANGE) {
                greet_ = false;
                RCLCPP_INFO(this->get_logger(), "Greeting the person");
                greet();
            }
        }
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF Transformation failed: %s", ex.what());
    }
    previous_face_position_ = *face_position;
}

eye_display_hld::msg::EyeControl InteractionController::convertFacePositionToEyeControl(const geometry_msgs::msg::PointStamped::SharedPtr& face_position) {
    // Search for the transformation from camera_1_front (face_position->header.frame_id) to robot_eyes
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped = tf_buffer_->lookupTransform(BETWEEN_EYES_FRAME, face_position->header.frame_id, face_position->header.stamp, rclcpp::Duration::from_seconds(0.0)); // 0.0 means the latest available transform, we don't need a timeout. Because relation between camera and eyes is a static transform and is always available.

    // Transform the point to the reference frame
    geometry_msgs::msg::PointStamped face_position_relative_to_eyes;
    tf2::doTransform(*face_position, face_position_relative_to_eyes, transform_stamped);
    
    RCLCPP_INFO(this->get_logger(), "Transformed point: x=%.2f, y=%.2f, z=%.2f",
                face_position_relative_to_eyes.point.x, face_position_relative_to_eyes.point.y, face_position_relative_to_eyes.point.z);

    
    eye_display_hld::msg::EyeControl eye_control_msg = eye_display_hld::msg::EyeControl();

    // Calculate yaw (horizontal angle) in radians
    eye_control_msg.yaw = std::atan2(face_position_relative_to_eyes.point.y, face_position_relative_to_eyes.point.x);
    // Calculate horizontal distance z-axis to xy-plane
    double r = std::sqrt(std::pow(face_position_relative_to_eyes.point.x, 2) + std::pow(face_position_relative_to_eyes.point.y, 2));
    // Calculate pitch (vertical angle) in radians
    eye_control_msg.pitch = std::atan2(face_position_relative_to_eyes.point.z, r);
    
    // Calculate distance to face in cm
    eye_control_msg.target_distance_cm = static_cast<uint16_t>(
    std::sqrt(std::pow(face_position_relative_to_eyes.point.x, 2) +
                std::pow(face_position_relative_to_eyes.point.y, 2) +
                std::pow(face_position_relative_to_eyes.point.z, 2)) * 100.0f);

    return eye_control_msg;
}

void InteractionController::radarPresenceCallback(const interaction_controller::msg::PresenceDetection::SharedPtr presence_msg) {
    bool is_presence_state_changed = isPresenceStateChanged(presence_msg);
    if(is_presence_state_changed) {
        auto screen_expression_msg = convertPresenceDetectionToScreenExpression(presence_msg);
        if(screen_expression_msg != nullptr) {
            updateLastPresenceDetection(presence_msg);
            screen_expression_pub_->publish(*screen_expression_msg);
            if(screen_expression_msg->action == eye_display_hld::msg::ScreenExpression::EYE_AWAKE) {
                greet_= true;
            } else if (screen_expression_msg->action == eye_display_hld::msg::ScreenExpression::EYE_SLEEP && !greet_) {
                farewell();
                greet_ = true;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Unknown screen expression state, no action will be taken.");
        }
    }
}

bool InteractionController::isPresenceStateChanged(const interaction_controller::msg::PresenceDetection::SharedPtr& presence_msg) {
    return last_precence_msg_.presence_state != presence_msg->presence_state;
}

void InteractionController::updateLastPresenceDetection(const interaction_controller::msg::PresenceDetection::SharedPtr& presence_msg) {
    last_precence_msg_ = *presence_msg;
}

eye_display_hld::msg::ScreenExpression::SharedPtr InteractionController::convertPresenceDetectionToScreenExpression(const interaction_controller::msg::PresenceDetection::SharedPtr& presence_msg) {
    auto screen_expression_msg = std::make_shared<eye_display_hld::msg::ScreenExpression>();

    if(presence_msg->presence_state == interaction_controller::msg::PresenceDetection::TARGET_IN_RANGE) {
        screen_expression_msg->action = eye_display_hld::msg::ScreenExpression::EYE_AWAKE;
    } else if(presence_msg->presence_state == interaction_controller::msg::PresenceDetection::TARGET_OUT_OF_RANGE) {
        screen_expression_msg->action = eye_display_hld::msg::ScreenExpression::EYE_SLEEP;
    } else {
        screen_expression_msg =  nullptr;
    }
    return screen_expression_msg;
}


bool InteractionController::isFacePositionInFrontOfRobotStationary(const geometry_msgs::msg::PointStamped::SharedPtr& face_position) {
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped = tf_buffer_->lookupTransform(BETWEEN_EYES_FRAME, face_position->header.frame_id, face_position->header.stamp, rclcpp::Duration::from_seconds(0.0)); // 0.0 means the latest available transform, we don't need a timeout. Because relation between camera and eyes is a static transform and is always available.

    geometry_msgs::msg::PointStamped face_position_relative_to_eyes;
    tf2::doTransform(*face_position, face_position_relative_to_eyes, transform_stamped);
    
    if(std::abs(face_position_relative_to_eyes.point.z) <= ROBOT_FACE_RADIUS_M && std::abs(face_position_relative_to_eyes.point.y) <= ROBOT_FACE_RADIUS_M) {

        const double x_diff = std::abs(face_position->point.x - previous_face_position_.point.x);
        const double y_diff = std::abs(face_position->point.y - previous_face_position_.point.y);
        const double z_diff = std::abs(face_position->point.z - previous_face_position_.point.z);
        RCLCPP_INFO(this->get_logger(), "x_diff: %.5f, y_diff: %.5f, z_diff: %.5f", x_diff, y_diff, z_diff);
        return x_diff <= 0.01 && y_diff <= 0.01 && z_diff <= 0.01;;
    }

    return false;
}

void InteractionController::greet() {
    auto goal_msg = audio_hld::action::PlaySound::Goal();
    goal_msg.sound_command.command = audio_hld::msg::SoundCommand::GREET;
    goal_msg.sound_command.repeat_count = 1;
    goal_msg.sound_command.repeat_delay_ms = 0;
    this->client_ptr_->async_send_goal(goal_msg);
}

void InteractionController::farewell() {
    auto goal_msg = audio_hld::action::PlaySound::Goal();
    goal_msg.sound_command.command = audio_hld::msg::SoundCommand::FAREWELL;
    goal_msg.sound_command.repeat_count = 1;
    goal_msg.sound_command.repeat_delay_ms = 0;
    this->client_ptr_->async_send_goal(goal_msg);
}

}  // namespace interaction_controller

