#include "audio_file_player_node.hpp"
#include "std_msgs/msg/bool.hpp"

#include <iostream>
#include <sstream>
#include <vector>
#include <regex>
#include <memory>

constexpr const char* DEFAULT_NODE_NAME = "audio_file_player";
constexpr const char* AUDIO_DEVICE_PARAMETER = "audio_device";
constexpr const char* SAMPLE_RATE_PARAMETER = "sample_rate";

namespace audio_lld {

AudioFilePlayerNode::AudioFilePlayerNode() 
: Node(DEFAULT_NODE_NAME), 
  pipeline_(nullptr),
  source_(nullptr),
  decoder_(nullptr),
  convert_(nullptr),
  resample_(nullptr), 
  capsfilter_(nullptr),
  volume_element_(nullptr),
  sink_(nullptr),
  is_audio_player_free_(false),
  audio_device_is_free_publisher_(create_publisher<std_msgs::msg::Bool>("audio_device_is_free", 10)),
  play_service_(create_service<audio_lld::srv::PlayAudioFile>("play_audio_file", std::bind(&AudioFilePlayerNode::play_audio_file_callback, this, std::placeholders::_1, std::placeholders::_2))),
  pause_service_(create_service<std_srvs::srv::Trigger>("pause_audio_file", std::bind(&AudioFilePlayerNode::pause_audio_file_callback, this, std::placeholders::_1, std::placeholders::_2))),
  resume_service_(create_service<std_srvs::srv::Trigger>("resume_audio_file", std::bind(&AudioFilePlayerNode::resume_audio_file_callback, this, std::placeholders::_1, std::placeholders::_2))),
  stop_service_(create_service<std_srvs::srv::Trigger>("stop_audio_file", std::bind(&AudioFilePlayerNode::stop_audio_file_callback, this, std::placeholders::_1, std::placeholders::_2)))
{
    gst_init(nullptr, nullptr);

    if (!initAudioPlayer()) {
        RCLCPP_ERROR(this->get_logger(), "Audio-player initialisation failed!");
        rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Audio File Player Node started. Waiting for service requests...");
    publish_audio_device_is_free(true);

    // Start a thread to monitor the GStreamer bus
    bus_thread_ = std::thread(&AudioFilePlayerNode::monitor_bus, this);
}

AudioFilePlayerNode::~AudioFilePlayerNode()
{
    publish_audio_device_is_free(false); //Because there will be no audio device anymore ;)
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (bus_thread_.joinable()) {
        bus_thread_.join();
    }
    if(pipeline_) {
        gst_object_unref(pipeline_);
    }
    gst_deinit();   
}

bool AudioFilePlayerNode::initAudioPlayer() {
    // Declare parameters for audio device selection and sample rate

    this->declare_parameter<std::string>(AUDIO_DEVICE_PARAMETER, "");
    this->declare_parameter<int>(SAMPLE_RATE_PARAMETER, 48000);

    std::string audio_device;
    int sample_rate;
    bool using_autoaudiosink = false;
    this->get_parameter(AUDIO_DEVICE_PARAMETER, audio_device);
    this->get_parameter(SAMPLE_RATE_PARAMETER, sample_rate);


    pipeline_ = gst_pipeline_new("audio_pipeline");
    source_ = gst_element_factory_make("filesrc", "source");
    decoder_ = gst_element_factory_make("decodebin", "decoder");
    convert_ = gst_element_factory_make("audioconvert", "convert");
    resample_ = gst_element_factory_make("audioresample", "resample");
    capsfilter_ = gst_element_factory_make("capsfilter", "capsfilter");
    volume_element_ = gst_element_factory_make("volume", "volume");

    if (!audio_device.empty()) {
        // If user specifies ALSA format "hw:X,Y", use alsasink
        if (audio_device.find("hw:") == 0 && isValidAlsaDevice(audio_device)) {
            sink_ = gst_element_factory_make("alsasink", "sink");
            g_object_set(G_OBJECT(sink_), "device", audio_device.c_str(), NULL);
            RCLCPP_INFO(this->get_logger(), "Using ALSA sink with device: %s", audio_device.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid ALSA format or device! Using default autoaudiosink.");
            sink_ = gst_element_factory_make("autoaudiosink", "sink");
            using_autoaudiosink = true;
        }
    } else {
        // Default to autoaudiosink
        sink_ = gst_element_factory_make("autoaudiosink", "sink");
        using_autoaudiosink = true;
    }

    if (!pipeline_ || !source_ || !decoder_ || !convert_ || !resample_ || !capsfilter_ || !volume_element_ || !sink_) {
        RCLCPP_ERROR(this->get_logger(), "GStreamer error: Failed to create elements.");
        return false;
    }

    if(using_autoaudiosink) {
        RCLCPP_INFO(this->get_logger(), "Configured sample rate will be ignored. Using default audio sink.");
    }
    else
    {
        // Configure sample rate via capsfilter
        GstCaps *caps = gst_caps_new_simple("audio/x-raw", "rate", G_TYPE_INT, sample_rate, NULL);
        g_object_set(G_OBJECT(capsfilter_), "caps", caps, NULL);
        gst_caps_unref(caps);
        RCLCPP_INFO(this->get_logger(), "Audio playback will use sample rate: %d Hz", sample_rate);
    }

    // Path connection for decodebin    
    g_signal_connect(decoder_, "pad-added", G_CALLBACK(+[]( [[maybe_unused]] GstElement *src, GstPad *new_pad, gpointer data) {
        GstElement *convert = static_cast<GstElement *>(data);
        GstPad *sink_pad = gst_element_get_static_pad(convert, "sink");

        if (!gst_pad_is_linked(sink_pad)) {
            gst_pad_link(new_pad, sink_pad);
        }
        gst_object_unref(sink_pad);
    }), convert_);

    gst_bin_add_many(GST_BIN(pipeline_), source_, decoder_, convert_, resample_, capsfilter_, volume_element_, sink_, NULL);

    if (!gst_element_link(source_, decoder_) || !gst_element_link_many(convert_, resample_, capsfilter_, volume_element_, sink_, NULL)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to link GStreamer pipeline.");
        return false;
    }

    return true;
}

void AudioFilePlayerNode::play_audio_file_callback(
    const std::shared_ptr<audio_lld::srv::PlayAudioFile::Request> request,
    std::shared_ptr<audio_lld::srv::PlayAudioFile::Response> response) {

    if (!pipeline_) {
        response->success = false;
        response->message = "Pipeline is not initialized.";
        return;
    }
    else
    {
        GstState state;
        gst_element_get_state(pipeline_, &state, nullptr, GST_CLOCK_TIME_NONE);
        if(state == GST_STATE_PLAYING) {
            response->success = false;
            response->message = "Cannot play: A audio is already playing.";
            return;
        }
        else if(state == GST_STATE_PAUSED) {
            response->success = false;
            response->message = "Cannot play: A audio is paused.";
            return;
        }
    }

    std::string file_path = request->file_path;
    uint8_t volume = (request->volume > 100) ? 100 : request->volume;

    g_object_set(G_OBJECT(source_), "location", file_path.c_str(), NULL);
    g_object_set(G_OBJECT(volume_element_), "volume", volume / 100.0f, NULL);

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        response->success = false;
        response->message = "Failed to start playback.";
        return;
    }

    response->success = true;
    response->message = "Playback started.";
    publish_audio_device_is_free(false);
}

void AudioFilePlayerNode::pause_audio_file_callback(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    GstState state;
    gst_element_get_state(pipeline_, &state, nullptr, GST_CLOCK_TIME_NONE);

    if (state == GST_STATE_NULL) {
        response->success = false;
        response->message = "Cannot resume: The audio player is empty.";
        return;
    }

    if (state != GST_STATE_PLAYING) {
        response->success = false;
        response->message = "Cannot pause: Audio is not currently playing.";
        return;
    }

    if (gst_element_set_state(pipeline_, GST_STATE_PAUSED) == GST_STATE_CHANGE_FAILURE) {
        response->success = false;
        response->message = "Failed to pause playback.";
        return;
    }

    response->success = true;
    response->message = "Playback paused.";
}

void AudioFilePlayerNode::resume_audio_file_callback(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    GstState state;
    gst_element_get_state(pipeline_, &state, nullptr, GST_CLOCK_TIME_NONE);

    if (state == GST_STATE_NULL) {
        response->success = false;
        response->message = "Cannot resume: The audio player is empty.";
        return;
    }

    if (state != GST_STATE_PAUSED) {
        response->success = false;
        response->message = "Cannot resume: Audio is not paused.";
        return;
    }

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        response->success = false;
        response->message = "Failed to resume playback.";
        return;
    }

    response->success = true;
    response->message = "Playback resumed.";
}

void AudioFilePlayerNode::stop_audio_file_callback(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    if (gst_element_set_state(pipeline_, GST_STATE_NULL) == GST_STATE_CHANGE_FAILURE) {
        response->success = false;
        response->message = "Failed to stop playback.";
        return;
    }

    response->success = true;
    response->message = "Playback stopped.";
    publish_audio_device_is_free(true);
}

void AudioFilePlayerNode::publish_audio_device_is_free(bool is_free) {
    if(is_free != is_audio_player_free_) {
        is_audio_player_free_ = is_free;
        auto msg = std_msgs::msg::Bool();
        msg.data = is_free;
        audio_device_is_free_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Audio device free status: %s", is_free ? "TRUE" : "FALSE");
    }
}

bool AudioFilePlayerNode::isValidAlsaDevice(const std::string& device) const {
    // Extract card and device numbers
    std::regex hw_regex(R"(hw:(\d+),(\d+))");
    std::smatch match;
    if (!std::regex_match(device, match, hw_regex)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid ALSA device format: %s. Expected format: hw:X,Y", device.c_str());
        return false;
    }

    std::string card_number = match[1].str();
    std::string device_number = match[2].str();

    // Use `grep` with regular expression to check for card X and device Y
    std::string cmd = "aplay -l | grep -E 'card " + card_number + ": .*device " + device_number + ":'";
    int result = system(cmd.c_str());

    if (result == 0) {
        RCLCPP_INFO(this->get_logger(), "ALSA device %s is valid!", device.c_str());
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "ALSA device %s not found!", device.c_str());
        return false;
    }
}

void AudioFilePlayerNode::monitor_bus() {
    GstBus *bus = gst_element_get_bus(pipeline_);
    GstMessage *msg;

    while (rclcpp::ok()) {
        msg = gst_bus_timed_pop_filtered(bus, GST_MSECOND * 500, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_STATE_CHANGED));

        if (!msg) {
            continue;
        }

        switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_EOS: { 
                RCLCPP_INFO(this->get_logger(), "Audio playback finished.");
                gst_element_set_state(pipeline_, GST_STATE_NULL);
                publish_audio_device_is_free(true);
                break;
            }

            case GST_MESSAGE_ERROR: { 
                GError *err;
                gchar *debug;
                gst_message_parse_error(msg, &err, &debug);
                RCLCPP_ERROR(this->get_logger(), "GStreamer Error: %s (Debug: %s)", err->message, debug ? debug : "No debug info available");
                g_error_free(err);
                g_free(debug);

                gst_element_set_state(pipeline_, GST_STATE_NULL);
                publish_audio_device_is_free(true);
                break;
            }

            default:
                break;
        }
        gst_message_unref(msg);
    }

    gst_object_unref(bus);
}


} // namespace audio_lld