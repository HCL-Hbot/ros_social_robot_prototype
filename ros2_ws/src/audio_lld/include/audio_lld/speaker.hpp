#ifndef AUDIO_LLD_SPEAKER_HPP
#define AUDIO_LLD_SPEAKER_HPP

#include <gst/gst.h>
#include <string>

namespace audio_lld {

class Speaker {
public:
    Speaker();
    ~Speaker();
    
    bool play(const std::string &filepath);
    void stop();

private:
    GstElement *pipeline_;
};

} // namespace audio_lld

#endif // AUDIO_LLD_SPEAKER_HPP
