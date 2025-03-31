#include "audio_hld/msg/sound_command.hpp"
#include <string>
#include <map>

namespace audio_hld
{
    // Mapping of sound command to file path
    static const std::map<uint8_t, const std::string> sound_command_to_file_path_map = {
        {audio_hld::msg::SoundCommand::GREET, "/home/hcl/Documents/ros_social_robot_prototype/ros2_ws/src/audio_hld/audio_files/groet.mp3"},
        {audio_hld::msg::SoundCommand::FAREWELL, "/home/hcl/Documents/ros_social_robot_prototype/ros2_ws/src/audio_hld/audio_files/doei.mp3"}
    };
}  // namespace audio_hld
