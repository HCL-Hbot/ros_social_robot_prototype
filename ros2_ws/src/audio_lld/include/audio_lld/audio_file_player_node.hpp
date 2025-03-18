#ifndef AUDIO_FILE_PLAYER_HPP
#define AUDIO_FILE_PLAYER_HPP

#include <rclcpp/rclcpp.hpp>
#include <gst/gst.h>
#include <thread>
#include "audio_lld/srv/play_audio_file.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"

namespace audio_lld {

class AudioFilePlayerNode : public rclcpp::Node {
public:
    AudioFilePlayerNode();
    
    ~AudioFilePlayerNode();
private:
    GstElement *pipeline_;
    GstElement *source_;
    GstElement *decoder_;
    GstElement *convert_;
    GstElement *resample_;
    GstElement *capsfilter_;
    GstElement *volume_element_;
    GstElement *sink_;
    bool is_audio_player_free_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr audio_device_is_free_publisher_;
    rclcpp::Service<audio_lld::srv::PlayAudioFile>::SharedPtr play_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

    std::thread bus_thread_;

    bool initAudioPlayer();

    void playAudioFileCallback(const std::shared_ptr<audio_lld::srv::PlayAudioFile::Request> request,
                                  std::shared_ptr<audio_lld::srv::PlayAudioFile::Response> response);
    
    void pauseAudioFileCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void resumeAudiFileCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void stopAudioFileCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response);


    void publishAudioDeviceIsFree(bool is_free);

    bool isValidAlsaDevice(const std::string& device) const;

    void monitorBus();
};

}  // namespace audio_lld

#endif // AUDIO_FILE_PLAYER_HPP
