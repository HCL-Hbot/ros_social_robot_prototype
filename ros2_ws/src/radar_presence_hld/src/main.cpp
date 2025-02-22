#include "radar_presence_hld.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<radar_hld::RadarPresenceHLD>());
    rclcpp::shutdown();
    return 0;
}