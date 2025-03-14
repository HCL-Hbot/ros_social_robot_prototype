#include "audio_file_player_node.hpp"

AudioFilePlayerNode::AudioFilePlayerNode() : Node("audio_file_player"), pipeline_(nullptr) {
    play_service_ = this->create_service<audio_lld::srv::PlayAudioFile>(
        "play_audio_file", std::bind(&AudioFilePlayerNode::play_audio_file_callback, this, std::placeholders::_1, std::placeholders::_2));

    pause_service_ = this->create_service<std_srvs::srv::Trigger>(
        "pause_audio_file", std::bind(&AudioFilePlayerNode::pause_audio_file_callback, this, std::placeholders::_1, std::placeholders::_2));

    resume_service_ = this->create_service<std_srvs::srv::Trigger>(
        "resume_audio_file", std::bind(&AudioFilePlayerNode::resume_audio_file_callback, this, std::placeholders::_1, std::placeholders::_2));

    stop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "stop_audio_file", std::bind(&AudioFilePlayerNode::stop_audio_file_callback, this, std::placeholders::_1, std::placeholders::_2));

    gst_init(nullptr, nullptr);

    RCLCPP_INFO(this->get_logger(), "Audio File Player Node started. Waiting for service requests...");
}

AudioFilePlayerNode::~AudioFilePlayerNode() {
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
    uint8_t volume = request->volume;

    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }

    pipeline_ = gst_pipeline_new("audio_pipeline");
    GstElement *source = gst_element_factory_make("filesrc", "source");
    GstElement *decoder = gst_element_factory_make("decodebin", "decoder");
    GstElement *convert = gst_element_factory_make("audioconvert", "convert");
    GstElement *resample = gst_element_factory_make("audioresample", "resample");
    GstElement *volume_element = gst_element_factory_make("volume", "volume");
    GstElement *sink = gst_element_factory_make("autoaudiosink", "sink");

    if (!pipeline_ || !source || !decoder || !convert || !resample || !volume_element || !sink) {
        response->success = false;
        response->message = "GStreamer error: Failed to create elements.";
        return;
    }

    g_object_set(G_OBJECT(source), "location", file_path.c_str(), NULL);
    g_object_set(G_OBJECT(volume_element), "volume", volume / 100.0, NULL);

    g_signal_connect(decoder, "pad-added", G_CALLBACK(+[](GstElement *src, GstPad *new_pad, gpointer data) {
        GstElement *convert = static_cast<GstElement *>(data);
        GstPad *sink_pad = gst_element_get_static_pad(convert, "sink");

        if (!gst_pad_is_linked(sink_pad)) {
            gst_pad_link(new_pad, sink_pad);
        }
        gst_object_unref(sink_pad);
    }), convert);

    gst_bin_add_many(GST_BIN(pipeline_), source, decoder, convert, resample, volume_element, sink, NULL);

    if (!gst_element_link(source, decoder) || !gst_element_link_many(convert, resample, volume_element, sink, NULL)) {
        response->success = false;
        response->message = "Failed to link GStreamer pipeline.";
        return;
    }

    std::string message;
    response->success = handle_gst_state_change(GST_STATE_PLAYING, "Playing", message);
    response->message = message;
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

    response->success = handle_gst_state_change(GST_STATE_PAUSED, "Audio player paused", response->message);
}

void AudioFilePlayerNode::resume_audio_file_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    response->success = handle_gst_state_change(GST_STATE_PLAYING, "Audio player resumed", response->message);
}

void AudioFilePlayerNode::stop_audio_file_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    response->success = handle_gst_state_change(GST_STATE_NULL, "Stopping", response->message);
}
