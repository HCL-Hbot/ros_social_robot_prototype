#include "audio_manager_node.hpp"
#include "audio_hld/msg/sound_command.hpp"
#include <chrono>
#include <thread>

namespace audio_hld {

AudioManagerNode::AudioManagerNode() : Node("audio_manager_node") {
    using namespace std::placeholders;

    // Create the action server
    action_server_ = rclcpp_action::create_server<PlaySound>(
        this,
        "/play_sound",
        std::bind(&AudioManagerNode::handle_goal, this, _1, _2),
        std::bind(&AudioManagerNode::handle_cancel, this, _1),
        std::bind(&AudioManagerNode::handle_accepted, this, _1)
    );

    // Audio client to call `audio_lld`
    audio_client_ = this->create_client<audio_lld::srv::PlayAudioFile>("/play_audio_file");

    RCLCPP_INFO(this->get_logger(), "AudioManagerNode started with Action Server.");
}

rclcpp_action::GoalResponse AudioManagerNode::handle_goal([[maybe_unused]] const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const PlaySound::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received sound request: %d (Repeat %d times)", goal->sound_command.command, goal->sound_command.repeat_count);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AudioManagerNode::handle_cancel([[maybe_unused]] const std::shared_ptr<GoalHandlePlaySound> goal_handle) {
    RCLCPP_WARN(this->get_logger(), "Playback canceled.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void AudioManagerNode::handle_accepted(const std::shared_ptr<GoalHandlePlaySound> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&AudioManagerNode::execute_sound, this, _1), goal_handle}.detach();
    //std::thread{std::bind(&AudioManagerNode::execute_sound, this, goal_handle)}.detach();
}

void AudioManagerNode::execute_sound(const std::shared_ptr<GoalHandlePlaySound> goal_handle) {

    auto result = std::make_shared<PlaySound::Result>();
    //default response
    result->success = false;
    result->executed_count = 0;

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
