#include "eyes_hld.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EyesHLD>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}