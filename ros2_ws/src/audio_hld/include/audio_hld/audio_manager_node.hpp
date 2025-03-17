#ifndef AUDIO_HLD_AUDIO_MANAGER_NODE_HPP_
#define AUDIO_HLD_AUDIO_MANAGER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "audio_hld/action/play_sound.hpp"
#include "audio_lld/srv/play_audio_file.hpp"
#include <string>
#include <memory>
#include <atomic>

namespace audio_hld {

class AudioManagerNode : public rclcpp::Node {
public:
    using PlaySound = audio_hld::action::PlaySound;
    using GoalHandlePlaySound = rclcpp_action::ServerGoalHandle<PlaySound>;

    AudioManagerNode();

private:
    rclcpp_action::Server<PlaySound>::SharedPtr action_server_;
    rclcpp::Client<audio_lld::srv::PlayAudioFile>::SharedPtr audio_client_;

    void execute_sound(const std::shared_ptr<GoalHandlePlaySound> goal_handle);
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const PlaySound::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePlaySound> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandlePlaySound> goal_handle);
};

}  // namespace audio_hld

#endif  // AUDIO_HLD_AUDIO_MANAGER_NODE_HPP_
