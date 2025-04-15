#include "camera_hld.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera_hld::CameraHLD>());
    rclcpp::shutdown();
    return 0;
}