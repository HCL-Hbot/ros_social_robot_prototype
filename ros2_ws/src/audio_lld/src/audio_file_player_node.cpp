#include "audio_file_player_node.hpp"

constexpr const char* DEFAULT_NODE_NAME = "audio_file_player";
constexpr const char* AUDIO_DEVICE_PARAMETER = "audio_device";
constexpr const char* SAMPLE_RATE_PARAMETER = "sample_rate";

namespace audio_lld {

AudioFilePlayerNode::AudioFilePlayerNode() 
: Node(DEFAULT_NODE_NAME), 
  pipeline_(nullptr),
  play_service_(create_service<audio_lld::srv::PlayAudioFile>("play_audio_file", std::bind(&AudioFilePlayerNode::play_audio_file_callback, this, std::placeholders::_1, std::placeholders::_2))),
  pause_service_(create_service<std_srvs::srv::Trigger>("pause_audio_file", std::bind(&AudioFilePlayerNode::pause_audio_file_callback, this, std::placeholders::_1, std::placeholders::_2))),
  resume_service_(create_service<std_srvs::srv::Trigger>("resume_audio_file", std::bind(&AudioFilePlayerNode::resume_audio_file_callback, this, std::placeholders::_1, std::placeholders::_2))),
  stop_service_(create_service<std_srvs::srv::Trigger>("stop_audio_file", std::bind(&AudioFilePlayerNode::stop_audio_file_callback, this, std::placeholders::_1, std::placeholders::_2)))
{
    // Declare parameters for audio device selection and sample rate
    this->declare_parameter<std::string>(AUDIO_DEVICE_PARAMETER, "");
    this->declare_parameter<int>(SAMPLE_RATE_PARAMETER, 48000);  // Default: 48 kHz

    gst_init(nullptr, nullptr);

    RCLCPP_INFO(this->get_logger(), "Audio File Player Node started. Waiting for service requests...");
}

AudioFilePlayerNode::~AudioFilePlayerNode()
{
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }
    gst_deinit();
}

void AudioFilePlayerNode::play_audio_file_callback(
    const std::shared_ptr<audio_lld::srv::PlayAudioFile::Request> request,
    std::shared_ptr<audio_lld::srv::PlayAudioFile::Response> response) {

    std::string file_path = request->file_path;
    uint8_t volume = (request->volume > 100) ? 100 : request->volume;
    std::string audio_device;
    int sample_rate;

    this->get_parameter(AUDIO_DEVICE_PARAMETER, audio_device);
    this->get_parameter(SAMPLE_RATE_PARAMETER, sample_rate);
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }

    pipeline_ = gst_pipeline_new("audio_pipeline");
    GstElement *source = gst_element_factory_make("filesrc", "source");
    GstElement *decoder = gst_element_factory_make("decodebin", "decoder");
    GstElement *convert = gst_element_factory_make("audioconvert", "convert");
    GstElement *resample = gst_element_factory_make("audioresample", "resample");
    GstElement *capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
    GstElement *volume_element = gst_element_factory_make("volume", "volume");

    // Determine the correct audio sink
    GstElement *sink = nullptr;
    if (!audio_device.empty()) {
        // If user specifies ALSA format "hw:X,Y", use alsasink
        if (audio_device.find("hw:") == 0) {
            sink = gst_element_factory_make("alsasink", "sink");
            g_object_set(G_OBJECT(sink), "device", audio_device.c_str(), NULL);
            RCLCPP_INFO(this->get_logger(), "Using ALSA sink with device: %s", audio_device.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid ALSA format! Using default autoaudiosink.");
            sink = gst_element_factory_make("autoaudiosink", "sink");
        }
    } else {
        // Default to autoaudiosink
        sink = gst_element_factory_make("autoaudiosink", "sink");
    }

    if (!pipeline_ || !source || !decoder || !convert || !resample || !capsfilter || !volume_element || !sink) {
        response->success = false;
        response->message = "GStreamer error: Failed to create elements.";
        return;
    }

    g_object_set(G_OBJECT(source), "location", file_path.c_str(), NULL);
    g_object_set(G_OBJECT(volume_element), "volume", volume / 100.0f, NULL);

    // Set sample rate
    GstCaps *caps = gst_caps_new_simple("audio/x-raw", "rate", G_TYPE_INT, sample_rate,NULL);
    g_object_set(G_OBJECT(capsfilter), "caps", caps, NULL);
    gst_caps_unref(caps);

    g_signal_connect(decoder, "pad-added", G_CALLBACK(+[](GstElement *src, GstPad *new_pad, gpointer data) {
        GstElement *convert = static_cast<GstElement *>(data);
        GstPad *sink_pad = gst_element_get_static_pad(convert, "sink");

        if (!gst_pad_is_linked(sink_pad)) {
            gst_pad_link(new_pad, sink_pad);
        }
        gst_object_unref(sink_pad);
    }), convert);

    gst_bin_add_many(GST_BIN(pipeline_), source, decoder, convert, resample, capsfilter, volume_element, sink, NULL);

    if (!gst_element_link(source, decoder) || !gst_element_link_many(convert, resample, capsfilter, volume_element, sink, NULL)) {
        response->success = false;
        response->message = "Failed to link GStreamer pipeline.";
        return;
    }

    response->success = handle_gst_state_change(GST_STATE_PLAYING, "Playing", response->message);
}

bool AudioFilePlayerNode::handle_gst_state_change(GstState new_state, const std::string &action, std::string &message) {
    if (!pipeline_) {
        message = action + " failed: No active audio stream.";
        return false;
    }

    GstState current;
    gst_element_get_state(pipeline_, &current, nullptr, GST_CLOCK_TIME_NONE);

    if (current == new_state) {
        message = action + ": already in the correct state.";
        return true;
    }

    if (gst_element_set_state(pipeline_, new_state) == GST_STATE_CHANGE_FAILURE) {
        message = action + " failed.";
        return false;
    }

    message = action + " successful.";
    return true;
}

void AudioFilePlayerNode::pause_audio_file_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    response->success = handle_gst_state_change(GST_STATE_PAUSED, "Pausing", response->message);
}

void AudioFilePlayerNode::resume_audio_file_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    response->success = handle_gst_state_change(GST_STATE_PLAYING, "Resuming", response->message);
}

void AudioFilePlayerNode::stop_audio_file_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    response->success = handle_gst_state_change(GST_STATE_NULL, "Stopping", response->message);
}

} // namespace audio_lld