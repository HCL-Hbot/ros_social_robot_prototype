#include "audio_file_player_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<audio_lld::AudioFilePlayerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}