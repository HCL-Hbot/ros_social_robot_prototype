#include "interaction_controller.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InteractionController>());
    rclcpp::shutdown();
    return 0;
}