#include "camera_hld.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraHLD>("camera_hld"));
    rclcpp::shutdown();
    return 0;
}