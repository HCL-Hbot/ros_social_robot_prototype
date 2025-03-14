#include "audio_file_player_node.hpp"

AudioFilePlayerNode::AudioFilePlayerNode() : Node("audio_file_player"), pipeline_(nullptr), current_state_(GST_STATE_NULL) {
    play_service_ = this->create_service<audio_lld::srv::PlayAudioFile>(
        "play_audio_file", std::bind(&AudioFilePlayerNode::play_audio_file_callback, this, std::placeholders::_1, std::placeholders::_2));

    pause_service_ = this->create_service<std_srvs::srv::Trigger>(
        "pause_audio_file", std::bind(&AudioFilePlayerNode::pause_audio_file_callback, this, std::placeholders::_1, std::placeholders::_2));

    resume_service_ = this->create_service<std_srvs::srv::Trigger>(
        "resume_audio_file", std::bind(&AudioFilePlayerNode::resume_audio_file_callback, this, std::placeholders::_1, std::placeholders::_2));

    stop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "stop_audio_file", std::bind(&AudioFilePlayerNode::stop_audio_file_callback, this, std::placeholders::_1, std::placeholders::_2));

    gst_init(nullptr, nullptr);

    RCLCPP_INFO(this->get_logger(), "Audio File Player Node gestart. Wacht op service-aanvragen...");
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

    RCLCPP_INFO(this->get_logger(), "Ontvangen verzoek: Bestand = %s, Volume = %d%%", file_path.c_str(), volume);

    // Stop vorige audio indien nodig
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
        RCLCPP_ERROR(this->get_logger(), "Fout bij aanmaken van GStreamer elementen!");
        response->success = false;
        response->message = "GStreamer fout: elementen konden niet worden aangemaakt.";
        return;
    }

    // **Stel het bestand en volume in**
    g_object_set(G_OBJECT(source), "location", file_path.c_str(), NULL);
    g_object_set(G_OBJECT(volume_element), "volume", volume / 100.0, NULL);

    // **Dynamische pad-koppeling voor de decoder** //De + bij de lambde doet een typecast naar een functiepointer (nodig voor C-API)
    g_signal_connect(decoder, "pad-added", G_CALLBACK(+[](GstElement *src, GstPad *new_pad, gpointer data) {
        GstElement *convert = static_cast<GstElement *>(data);
        GstPad *sink_pad = gst_element_get_static_pad(convert, "sink");

        if (!gst_pad_is_linked(sink_pad)) {
            if (gst_pad_link(new_pad, sink_pad) != GST_PAD_LINK_OK) {
                g_printerr("Fout bij koppelen van decodebin aan audioconvert!\n");
            } else {
                g_print("Decoder is succesvol gekoppeld!\n");
            }
        }
        gst_object_unref(sink_pad);
    }), convert);

    // Voeg elementen toe en link de bekende delen
    gst_bin_add_many(GST_BIN(pipeline_), source, decoder, convert, resample, volume_element, sink, NULL);

    if (!gst_element_link(source, decoder) || !gst_element_link_many(convert, resample, volume_element, sink, NULL)) {
        RCLCPP_ERROR(this->get_logger(), "Fout bij koppelen van audio pipeline!");
        response->success = false;
        response->message = "Kon GStreamer pipeline niet koppelen.";
        return;
    }

    // **Start de pipeline**
    GstStateChangeReturn state = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (state == GST_STATE_CHANGE_FAILURE) {
        RCLCPP_ERROR(this->get_logger(), "Error: Could not start audio playback!");
        response->success = false;
        response->message = "Afspelen gefaald!";
        return;
    }

    response->success = true;
    response->message = "Afspelen gestart.";
    current_state_ = GST_STATE_PLAYING;
    RCLCPP_INFO(this->get_logger(), "Audio gestart: %s", file_path.c_str());
}


void AudioFilePlayerNode::pause_audio_file_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    if (!pipeline_) {
        response->success = false;
        response->message = "Geen actieve audiostream om te pauzeren.";
        return;
    }

    // **Controleer de werkelijke status van de pipeline**
    gst_element_get_state(pipeline_, &current_state_, nullptr, GST_CLOCK_TIME_NONE);

    if (current_state_ == GST_STATE_PLAYING) {
        if(gst_element_set_state(pipeline_, GST_STATE_PAUSED) == GST_STATE_CHANGE_FAILURE) {
            response->success = false;
            response->message = "Kon audio niet pauzeren.";
            RCLCPP_ERROR(this->get_logger(), "Pauzeren mislukt: Kon audio niet pauzeren.");
        } else {
            response->success = true;
            response->message = "Audio gepauzeerd.";
            RCLCPP_INFO(this->get_logger(), "Audio gepauzeerd.");
        }
    } else if (current_state_ == GST_STATE_PAUSED) {
        response->success = true;
        response->message = "Audio is al gepauzeerd.";
    } 
    else {
        response->success = false;
        response->message = "Geen audio aan het spelen.";
        RCLCPP_WARN(this->get_logger(), "Pauzeren mislukt: Geen actieve audio.");
    }
}


void AudioFilePlayerNode::resume_audio_file_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    if (!pipeline_) {
        response->success = false;
        response->message = "Geen actieve audiostream om te hervatten.";
        return;
    }    
    
    gst_element_get_state(pipeline_, &current_state_, nullptr, GST_CLOCK_TIME_NONE);

    if (pipeline_ && current_state_ == GST_STATE_PAUSED) {
        if(gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            response->success = false;
            response->message = "Kon audio niet hervatten.";
        } else{
            response->success = true;
            response->message = "Audio hervat.";
        }
    } else if (current_state_ == GST_STATE_PLAYING) {
        response->success = true;
        response->message = "Audio is al aan het spelen.";
    } 
    else {
        response->success = false;
        response->message = "Audio is niet gepauzeerd.";
    }
}

void AudioFilePlayerNode::stop_audio_file_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        current_state_ = GST_STATE_NULL;
        response->success = true;
        response->message = "Audio gestopt.";
    } else {
        response->success = true;
        response->message = "Geen audio om te stoppen.";
    }
}
