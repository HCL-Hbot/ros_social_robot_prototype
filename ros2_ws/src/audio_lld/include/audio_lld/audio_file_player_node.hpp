#ifndef AUDIO_FILE_PLAYER_HPP
#define AUDIO_FILE_PLAYER_HPP

#include <rclcpp/rclcpp.hpp>
#include <gst/gst.h>
#include "audio_lld/srv/play_audio_file.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace audio_lld {

class AudioFilePlayerNode : public rclcpp::Node {
public:
    AudioFilePlayerNode();
    ~AudioFilePlayerNode();

private:
    GstElement *pipeline_;

    rclcpp::Service<audio_lld::srv::PlayAudioFile>::SharedPtr play_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

    void play_audio_file_callback(const std::shared_ptr<audio_lld::srv::PlayAudioFile::Request> request,
                                  std::shared_ptr<audio_lld::srv::PlayAudioFile::Response> response);
    
    void pause_audio_file_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void resume_audio_file_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void stop_audio_file_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    bool handle_gst_state_change(GstState new_state, const std::string &action, std::string &message);
};

}  // namespace audio_lld

#endif // AUDIO_FILE_PLAYER_HPP
