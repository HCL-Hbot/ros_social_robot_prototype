#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include "audio_lld/speaker.hpp"

class AudioLLDNode : public rclcpp::Node {
public:
    AudioLLDNode() : Node("audio_lld_node") {
        speaker_ = std::make_shared<audio_lld::Speaker>();

        play_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/audio_playback/play", 10, std::bind(&AudioLLDNode::playAudio, this, std::placeholders::_1));

        stop_service_ = this->create_service<std_srvs::srv::Empty>(
            "/audio_playback/stop", std::bind(&AudioLLDNode::stopAudio, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Audio LLD Node gestart.");
    }

private:
    std::shared_ptr<audio_lld::Speaker> speaker_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr play_subscriber_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;

    void playAudio(const std_msgs::msg::String::SharedPtr msg) {
        if (!speaker_->play(msg->data)) {
            RCLCPP_ERROR(this->get_logger(), "Fout bij afspelen van audio: %s", msg->data.c_str());
        }
    }

    void stopAudio(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                   std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        speaker_->stop();
        RCLCPP_INFO(this->get_logger(), "Audio playback gestopt.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AudioLLDNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
