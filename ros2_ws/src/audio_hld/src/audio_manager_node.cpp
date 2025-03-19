#include "audio_manager_node.hpp"
#include "audio_hld/msg/sound_command.hpp"
#include <chrono>
#include <thread>

constexpr const char* DEFAULT_NODE_NAME = "audio_manager_node";
constexpr const char* AUDIO_PLAYER_FREE_TOPIC = "audio_device_is_free";

namespace audio_hld {

AudioManagerNode::AudioManagerNode() 
: Node(DEFAULT_NODE_NAME),
  audio_device_is_free_(true),
  active_goal_(nullptr),
  current_response_(nullptr),
  action_server_(rclcpp_action::create_server<PlaySound>(this,"/play_sound",
    std::bind(&AudioManagerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&AudioManagerNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&AudioManagerNode::handle_accepted, this, std::placeholders::_1))),
  audio_client_(this->create_client<audio_lld::srv::PlayAudioFile>("/play_audio_file")),
  is_audio_player_free_subscriber_(this->create_subscription<std_msgs::msg::Bool>(AUDIO_PLAYER_FREE_TOPIC,10,std::bind(&AudioManagerNode::audio_player_free_callback, this, std::placeholders::_1)))
{
    RCLCPP_INFO(this->get_logger(), "AudioManagerNode started with Action Server.");
}

rclcpp_action::GoalResponse AudioManagerNode::handle_goal([[maybe_unused]] const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const PlaySound::Goal> goal) {
    
    RCLCPP_INFO(this->get_logger(), "Received sound request: %d (Repeat %d times)", goal->sound_command.command, goal->sound_command.repeat_count);

    // Als er een bestaand doel actief is, annuleer deze
    if (active_goal_ && active_goal_->is_active()) {
        RCLCPP_WARN(this->get_logger(), "Aborting previous action and executing the recevieved action.");
        active_goal_->abort(current_response_);
    }
        
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AudioManagerNode::handle_cancel([[maybe_unused]] const std::shared_ptr<GoalHandlePlaySound> goal_handle) {
    RCLCPP_WARN(this->get_logger(), "Playback canceled.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void AudioManagerNode::handle_accepted(const std::shared_ptr<GoalHandlePlaySound> goal_handle) {
    
    active_goal_ = goal_handle;
    current_response_.reset(new PlaySound::Result);
    current_response_->success = false;
    current_response_->executed_count = 0;
    current_response_->message = "Playback not executed.";

    using namespace std::placeholders;
    std::thread{std::bind(&AudioManagerNode::execute_sound, this, _1), active_goal_}.detach();
    
    //active_goal_ = goal_handle;

    //std::thread{std::bind(&AudioManagerNode::execute_sound, this, std::placeholders::_1), goal_handle}.detach();
    //std::thread{std::bind(&AudioManagerNode::execute_sound, this, active_goal_)}.detach();
}

void AudioManagerNode::audio_player_free_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        audio_device_is_free_ = true;
        RCLCPP_INFO(this->get_logger(), "Audio player is free.");
    } else {
        audio_device_is_free_ = false;
        RCLCPP_INFO(this->get_logger(), "Audio player is busy.");
    }
}

void AudioManagerNode::execute_sound(const std::shared_ptr<GoalHandlePlaySound> goal_handle) {

    //auto result = std::make_shared<PlaySound::Result>();
    auto& result = current_response_;
    //default response
    //result->success = false;
    //result->executed_count = 0;

    uint8_t command = goal_handle->get_goal()->sound_command.command;

    std::string file_path;
    switch (command) {
        case audio_hld::msg::SoundCommand::GREET:
            file_path = "/home/hcl/Downloads/hello.mp3";
            break;
        case audio_hld::msg::SoundCommand::FAREWELL:
            file_path = "/home/hcl/Downloads/i_like_you.mp3";
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Invalid command.");
            result->message = "Invalid command.";
            goal_handle->abort(result);
            return;
    }

    for (uint8_t i = 0; i < goal_handle->get_goal()->sound_command.repeat_count; i++) {
        if (goal_handle->is_canceling()) {
            result->message = "Playback was canceled.";
            result->executed_count = i;
            goal_handle->canceled(result);
            return;
        }

        auto request = std::make_shared<audio_lld::srv::PlayAudioFile::Request>();
        request->file_path = file_path;
        request->volume = 80;

        if (!audio_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "audio_lld service not available.");
            result->message = "audio_lld service unavailable.";
            result->executed_count = i;
            goal_handle->abort(result);
            return;
        }

        // // Send the request and wait for response
        // auto future = audio_client_->async_send_request(request);
        // if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        //     auto response = future.get();
        //     if (!response->success) {
        //         RCLCPP_ERROR(this->get_logger(), "Audio playback failed: %s", response->message.c_str());
        //         result->message = "Audio playback failed.";
        //         result->executed_count = i;
        //         goal_handle->abort(result);
        //         return;
        //     }
        //     RCLCPP_INFO(this->get_logger(), "Audio playback started successfully.");
        // } else {
        //     RCLCPP_ERROR(this->get_logger(), "Timeout waiting for audio playback response.");
        //     result->message = "Audio playback service timed out.";
        //     result->executed_count = i;
        //     goal_handle->abort(result);
        //     return;
        // }

        
        auto response = audio_client_->async_send_request(request);

        RCLCPP_INFO(this->get_logger(), "Playing: %s (Repeat %d/%d)", file_path.c_str(), i + 1, goal_handle->get_goal()->sound_command.repeat_count);
        std::this_thread::sleep_for(std::chrono::duration<float>(goal_handle->get_goal()->sound_command.repeat_delay_sec));
    }

    result->success = true;
    result->message = "Command executed successfully.";
    result->executed_count = goal_handle->get_goal()->sound_command.repeat_count;
    goal_handle->succeed(result);
}

}  // namespace audio_hld
