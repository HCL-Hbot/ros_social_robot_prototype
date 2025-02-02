#include "eyes_hld.hpp"

EyesHLD::EyesHLD() : Node("eyes_hld_node")
{
    RCLCPP_INFO(this->get_logger(), "EyesHLD node has been started.");
}