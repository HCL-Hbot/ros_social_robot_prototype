#include "interaction_controller.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<interaction_controller::InteractionController>());
    rclcpp::shutdown();
    return 0;
}