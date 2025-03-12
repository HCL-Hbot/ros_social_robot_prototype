#include "audio_lld/speaker.hpp"
#include <iostream>

namespace audio_lld {

Speaker::Speaker() {
    gst_init(nullptr, nullptr);
    pipeline_ = nullptr;
}

Speaker::~Speaker() {
    stop();
}

bool Speaker::play(const std::string &filepath) {
    stop();  // Stop vorige audio als die nog speelt

    pipeline_ = gst_pipeline_new("audio_pipeline");
    GstElement *source = gst_element_factory_make("filesrc", "source");
    GstElement *decoder = gst_element_factory_make("decodebin", "decoder");
    GstElement *convert = gst_element_factory_make("audioconvert", "convert");
    GstElement *resample = gst_element_factory_make("audioresample", "resample");
    GstElement *sink = gst_element_factory_make("autoaudiosink", "sink");

    if (!pipeline_ || !source || !decoder || !convert || !resample || !sink) {
        std::cerr << "Error: Failed to create GStreamer elements!" << std::endl;
        return false;
    }

    // Stel het audiobestand in
    g_object_set(G_OBJECT(source), "location", filepath.c_str(), NULL);

    // Voeg de elementen toe aan de pipeline
    gst_bin_add_many(GST_BIN(pipeline_), source, decoder, convert, resample, sink, NULL);

    // Link de bron aan de decoder
    if (!gst_element_link(source, decoder)) {
        std::cerr << "Error: Could not link source to decoder!" << std::endl;
        return false;
    }

    // Connecteer de decoder dynamisch met de rest van de pipeline
    g_signal_connect(decoder, "pad-added", G_CALLBACK(+[](GstElement *src, GstPad *new_pad, gpointer data) {
        GstElement *convert = static_cast<GstElement *>(data);
        GstPad *sink_pad = gst_element_get_static_pad(convert, "sink");

        if (!gst_pad_is_linked(sink_pad)) {
            gst_pad_link(new_pad, sink_pad);
        }
        gst_object_unref(sink_pad);
    }), convert);

    // Link de overige elementen
    if (!gst_element_link_many(convert, resample, sink, NULL)) {
        std::cerr << "Error: Could not link elements!" << std::endl;
        return false;
    }

    // Start de pipeline
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    std::cout << "Playing audio: " << filepath << std::endl;
    return true;
}

void Speaker::stop() {
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        std::cout << "Audio playback stopped." << std::endl;
    }
}

} // namespace audio_lld
