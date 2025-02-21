#include "camera_lld.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera_lld::CameraLLD>());
    rclcpp::shutdown();
    return 0;
}