#include "audio_manager_node.hpp"
#include "audio_hld/msg/sound_command.hpp"
#include <chrono>
#include <thread>

constexpr const char* DEFAULT_NODE_NAME = "audio_manager_node";
constexpr const char* AUDIO_PLAYER_FREE_TOPIC = "audio_device_is_free";
constexpr const char* PLAY_AUDIO_FILE_SERVICE = "play_audio_file";
constexpr const char* STOP_AUDIO_FILE_SERVICE = "stop_audio_file";

namespace audio_hld {

AudioManagerNode::AudioManagerNode() 
: Node(DEFAULT_NODE_NAME),
  audio_device_is_free_(true),
  goal_aborted_by_new_request_(false),
  active_goal_(nullptr),
  current_response_(nullptr),
  action_server_(rclcpp_action::create_server<PlaySound>(this,"/play_sound",
    std::bind(&AudioManagerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&AudioManagerNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&AudioManagerNode::handle_accepted, this, std::placeholders::_1))),
  audio_play_client_(this->create_client<audio_lld::srv::PlayAudioFile>(PLAY_AUDIO_FILE_SERVICE)),
  audio_stop_client_(this->create_client<std_srvs::srv::Trigger>(STOP_AUDIO_FILE_SERVICE)),
  is_audio_player_free_subscriber_(this->create_subscription<std_msgs::msg::Bool>(AUDIO_PLAYER_FREE_TOPIC,10,std::bind(&AudioManagerNode::audio_player_free_callback, this, std::placeholders::_1)))
{
    RCLCPP_INFO(this->get_logger(), "AudioManagerNode started with Action Server.");
}

bool AudioManagerNode::abort_current_goal()
{
    if (active_goal_ && active_goal_->is_active()) {
        //active_goal_->canceled(current_response_);
        active_goal_->abort(current_response_);
        active_goal_.reset();
        return true;
    }
    return false;
}

rclcpp_action::GoalResponse AudioManagerNode::handle_goal([[maybe_unused]] const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const PlaySound::Goal> goal) {
    
    RCLCPP_INFO(this->get_logger(), "Received sound request: %d (Repeat %d times)", goal->sound_command.command, goal->sound_command.repeat_count);

    if(active_goal_ && active_goal_->is_active())
    {
        goal_aborted_by_new_request_ = true;
        while(active_goal_->is_active()){}//Wait until the current goal is aborted
        goal_aborted_by_new_request_ = false;
        RCLCPP_WARN(this->get_logger(), "Previous action aborted and executing the new received action.");
    }      
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AudioManagerNode::handle_cancel([[maybe_unused]] const std::shared_ptr<GoalHandlePlaySound> goal_handle) {
    RCLCPP_WARN(this->get_logger(), "Playback canceled.!!");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void AudioManagerNode::handle_accepted(const std::shared_ptr<GoalHandlePlaySound> goal_handle) {
    
    active_goal_ = goal_handle;
    current_response_.reset(new PlaySound::Result);
    current_response_->success = false;
    current_response_->executed_count = 0;
    current_response_->message = "Playback not executed.";

    std::thread{std::bind(&AudioManagerNode::execute_sound, this, active_goal_)}.detach();
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

    auto feedback = std::make_shared<PlaySound::Feedback>();
    feedback->executing_count = 0;

    current_response_->success = false;
    current_response_->executed_count = 0;

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
            current_response_->message = "Invalid command.";
            goal_handle->abort(current_response_);
            return;
    }

    for (uint8_t i = 0; i < goal_handle->get_goal()->sound_command.repeat_count; i++) {
    
        auto request = std::make_shared<audio_lld::srv::PlayAudioFile::Request>();
        request->file_path = file_path;
        request->volume = 80;

        bool status = audio_play_client_->service_is_ready();
        if (!status) {
            RCLCPP_ERROR(this->get_logger(), "audio_lld service not available.");
            current_response_->message = "audio_lld service unavailable!!.";
            goal_handle->abort(current_response_);
            return;
        }

        // Send the request and wait for response wih a timeout of 100ms
        auto future = audio_play_client_->async_send_request(request);
        if (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
            auto response = future.get();
            if (!response->success) {
                RCLCPP_ERROR(this->get_logger(), "Audio playback failed: %s", response->message.c_str());
                current_response_->message = "Audio playback failed.";
                current_response_->executed_count = i;
                goal_handle->abort(current_response_);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Audio playback started successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting for audio playback response.");
            current_response_->message = "Audio playback service timed out.";
            current_response_->executed_count = i;
            goal_handle->abort(current_response_);
            return;
        }

        feedback->executing_count = i + 1;
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(this->get_logger(), "Playing: %s (Repeat %d/%d)", file_path.c_str(), feedback->executing_count, goal_handle->get_goal()->sound_command.repeat_count);

        // Wait until audio device is free, or goal is aborted by a new request, or goal is canceled by the client.
        while(!audio_device_is_free_ && !goal_aborted_by_new_request_ && !goal_handle->is_canceling()){ }

        if (goal_handle->is_canceling()) {
            RCLCPP_INFO(this->get_logger(), "Playback canceled by client!");
            audio_stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
            current_response_->message = "Playback was canceled.";
            goal_handle->canceled(current_response_);
            return;
        }
        else if(goal_aborted_by_new_request_) //Goal is aborted by a new request
        {
            RCLCPP_INFO(this->get_logger(), "Playback aborted by new request!");
            audio_stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
            current_response_->message = "Playback was aborted by new request.";
            goal_handle->abort(current_response_);
            goal_aborted_by_new_request_ = false;
            return;
        }
        else if(audio_device_is_free_)
        {
            current_response_->executed_count = feedback->executing_count;
        }

        const uint16_t wait_time_ms = goal_handle->get_goal()->sound_command.repeat_delay_ms;
        auto end_time = this->now() + rclcpp::Duration(std::chrono::milliseconds(wait_time_ms));
        while (this->now() < end_time && current_response_->executed_count >= goal_handle->get_goal()->sound_command.repeat_count) {
            if(goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Playback canceled by client!!");
                audio_stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
                current_response_->message = "Playback was canceled.";
                goal_handle->canceled(current_response_);
                return;
            } else if(goal_aborted_by_new_request_) { //Goal is aborted by a new request
                RCLCPP_INFO(this->get_logger(), "Playback aborted by new request!!");
                audio_stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
                current_response_->message = "Playback was aborted by new request.";
                goal_handle->abort(current_response_);
                return;
            }
            else if(!audio_device_is_free_) {// it seems audio device is busy withouth any new request from the high level driver
                RCLCPP_ERROR(this->get_logger(), "Playback aborted because audio device is busy! Not initiated by audio manager!.");
                audio_stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
                current_response_->message = "Playback aborted because audio device is busy! Not initiated by audio manager!.";
                goal_handle->abort(current_response_);
                return;
            }

        }
    }

    current_response_->success = true;
    current_response_->message = "Command executed successfully.";
    goal_handle->succeed(current_response_);
}

}  // namespace audio_hld
