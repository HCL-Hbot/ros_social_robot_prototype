#ifndef AUDIO_HLD_AUDIO_MANAGER_NODE_HPP_
#define AUDIO_HLD_AUDIO_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "audio_hld/action/play_sound.hpp"
#include "audio_lld/srv/play_audio_file.hpp"


namespace audio_hld {

class AudioManagerNode : public rclcpp::Node {
public:
    using PlaySound = audio_hld::action::PlaySound;
    using GoalHandlePlaySound = rclcpp_action::ServerGoalHandle<PlaySound>;

    AudioManagerNode();

private:
    bool audio_device_is_free_;
    bool goal_aborted_by_new_request_;
    
    std::shared_ptr<GoalHandlePlaySound> active_goal_;  // Current active goal
    std::shared_ptr<PlaySound::Result> current_response_;

    rclcpp_action::Server<PlaySound>::SharedPtr action_server_;
    rclcpp::Client<audio_lld::srv::PlayAudioFile>::SharedPtr audio_play_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr audio_stop_client_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_audio_player_free_subscriber_;

    void executeSound(const std::shared_ptr<GoalHandlePlaySound> goal_handle);
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const PlaySound::Goal> goal);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandlePlaySound> goal_handle);
    void handleAccepted(const std::shared_ptr<GoalHandlePlaySound> goal_handle);
    void audioPlayerFreeCallback(const std_msgs::msg::Bool::SharedPtr msg);
};

}  // namespace audio_hld

#endif  // AUDIO_HLD_AUDIO_MANAGER_NODE_HPP_
