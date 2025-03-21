#include "interaction_controller.hpp"


constexpr const char* DEFAULT_NODE_NAME = "interaction_controller_node";
constexpr const char* DEFAULT_TOPIC_NAME_SUB_FACE = "face_info";
constexpr const char* DEFAULT_TOPIC_NAME_PUB_EYE_CONTROL = "eye_control";
constexpr const char* DEFAULT_TOPIC_NAME_SUB_RADAR_PRESENCE_HL = "radar_presence";
constexpr const char* DEFAULT_TOPIC_NAME_PUB_SCREEN_EXPRESSION = "screen_expression";
constexpr const char* BETWEEN_EYES_FRAME = "robot_eyes_between";

namespace interaction_controller {
    
InteractionController::InteractionController() 
: rclcpp::Node(DEFAULT_NODE_NAME),
  face_position_sub_(create_subscription<geometry_msgs::msg::PointStamped>(
        DEFAULT_TOPIC_NAME_SUB_FACE, 10, std::bind(&InteractionController::facePositionCallback, this, std::placeholders::_1))),
  eye_control_pub_(create_publisher<eye_display_hld::msg::EyeControl>(DEFAULT_TOPIC_NAME_PUB_EYE_CONTROL, 10)),
  radar_presence_sub_(create_subscription<interaction_controller::msg::PresenceDetection>(
        DEFAULT_TOPIC_NAME_SUB_RADAR_PRESENCE_HL, 10, std::bind(&InteractionController::radarPresenceCallback, this, std::placeholders::_1))),
  screen_expression_pub_(this->create_publisher<eye_display_hld::msg::ScreenExpression>(DEFAULT_TOPIC_NAME_PUB_SCREEN_EXPRESSION, 10)),
  last_precence_msg_()
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    last_precence_msg_.presence_state = interaction_controller::msg::PresenceDetection::TARGET_OUT_OF_RANGE;
    last_precence_msg_.target_state = interaction_controller::msg::PresenceDetection::TARGET_STANDING;

    RCLCPP_INFO(this->get_logger(), "InteractionController started");

    //Testing audio component from the interaction controller:
    this->client_ptr_ = rclcpp_action::create_client<PlaySound>(this,"/play_sound");
    std::thread{std::bind(&InteractionController::send_goal, this)}.detach();

}

InteractionController::~InteractionController()
{
}

void InteractionController::facePositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr face_position)
{
    try {
        eye_display_hld::msg::EyeControl eye_control_msg = convertFacePositionToEyeControl(face_position);
        eye_control_pub_->publish(eye_control_msg);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF Transformation failed: %s", ex.what());
    }
}

eye_display_hld::msg::EyeControl InteractionController::convertFacePositionToEyeControl(const geometry_msgs::msg::PointStamped::SharedPtr& face_position)
{
    // Search for the transformation from camera_1_front (face_position->header.frame_id) to robot_eyes
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped = tf_buffer_->lookupTransform(BETWEEN_EYES_FRAME, face_position->header.frame_id, face_position->header.stamp, rclcpp::Duration::from_seconds(0.0)); // 0.0 means the latest available transform, we don't need a timeout. Because relation between camera and eyes is a static transform and is always available.

    // Transform the point to the reference frame
    geometry_msgs::msg::PointStamped transformed_point;
    tf2::doTransform(*face_position, transformed_point, transform_stamped);
    
    RCLCPP_INFO(this->get_logger(), "Transformed point: x=%.2f, y=%.2f, z=%.2f",
                transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);

    
    eye_display_hld::msg::EyeControl eye_control_msg = eye_display_hld::msg::EyeControl();

    // Calculate yaw (horizontal angle) in radians
    eye_control_msg.yaw = std::atan2(transformed_point.point.y, transformed_point.point.x);
    // Calculate horizontal distance z-axis to xy-plane
    double r = std::sqrt(std::pow(transformed_point.point.x, 2) + std::pow(transformed_point.point.y, 2));
    // Calculate pitch (vertical angle) in radians
    eye_control_msg.pitch = std::atan2(transformed_point.point.z, r);
    
    // Calculate distance to face in cm
    eye_control_msg.target_distance_cm = static_cast<uint16_t>(
    std::sqrt(std::pow(transformed_point.point.x, 2) +
                std::pow(transformed_point.point.y, 2) +
                std::pow(transformed_point.point.z, 2)) * 100.0f);

    return eye_control_msg;
}

void InteractionController::radarPresenceCallback(const interaction_controller::msg::PresenceDetection::SharedPtr presence_msg)
{
    bool is_presence_state_changed = isPresenceStateChanged(presence_msg);
    if(is_presence_state_changed) {
        auto screen_expression_msg = convertPresenceDetectionToScreenExpression(presence_msg);
        if(screen_expression_msg != nullptr) {
            updateLastPresenceDetection(presence_msg);
            screen_expression_pub_->publish(*screen_expression_msg);
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Unknown screen expression state, no action will be taken.");
        }
    }
}

bool InteractionController::isPresenceStateChanged(const interaction_controller::msg::PresenceDetection::SharedPtr& presence_msg)
{
    return last_precence_msg_.presence_state != presence_msg->presence_state;
}

void InteractionController::updateLastPresenceDetection(const interaction_controller::msg::PresenceDetection::SharedPtr& presence_msg)
{
    last_precence_msg_ = *presence_msg;
}

eye_display_hld::msg::ScreenExpression::SharedPtr InteractionController::convertPresenceDetectionToScreenExpression(const interaction_controller::msg::PresenceDetection::SharedPtr& presence_msg)
{
    auto screen_expression_msg = std::make_shared<eye_display_hld::msg::ScreenExpression>();

    if(presence_msg->presence_state == interaction_controller::msg::PresenceDetection::TARGET_IN_RANGE) {
        screen_expression_msg->action = eye_display_hld::msg::ScreenExpression::EYE_AWAKE;
    }
    else if(presence_msg->presence_state == interaction_controller::msg::PresenceDetection::TARGET_OUT_OF_RANGE) {
        screen_expression_msg->action = eye_display_hld::msg::ScreenExpression::EYE_SLEEP;
    }
    else {
        screen_expression_msg =  nullptr;
    }
    return screen_expression_msg;
}

void InteractionController::send_goal()
{
    std::this_thread::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto goal_msg = audio_hld::action::PlaySound::Goal();
    goal_msg.sound_command.command = audio_hld::msg::SoundCommand::FAREWELL;
    goal_msg.sound_command.repeat_count = 2;
    goal_msg.sound_command.repeat_delay_ms = 1000;
    
    auto send_goal_options = rclcpp_action::Client<PlaySound>::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](const GoalHandlePlaySound::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](
        GoalHandlePlaySound::SharedPtr,
        const std::shared_ptr<const PlaySound::Feedback> feedback)
      {
        RCLCPP_INFO(this->get_logger(), "Next sound command is being executed: %d", feedback->executing_count);
      };

    send_goal_options.result_callback = [this](const GoalHandlePlaySound::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            break;
            case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(this->get_logger(), "Goal was aborted");
            break;
            case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Goal was canceled");
            break;
            default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
        RCLCPP_INFO(this->get_logger(), "Result message: %s", result.result->message.c_str());
        RCLCPP_INFO(this->get_logger(), "Executed count: %d", result.result->executed_count);
        rclcpp::shutdown();
    };
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}
}  // namespace interaction_controller

