#include "camera_hld.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Maak een tijdelijke node om de parameter op te halen
    auto temp_node = std::make_shared<rclcpp::Node>("temp_node");

    // Declareer en lees de 'node_name' parameter
    temp_node->declare_parameter<std::string>("node_name", "default_node_name");
    std::string node_name = temp_node->get_parameter("node_name").as_string();

    // Creëer de CameraHLD-node met de opgehaalde naam
    auto camera_node = std::make_shared<CameraHLD>(node_name);

    temp_node.reset();
    // Spin de node
    rclcpp::spin(camera_node);
    rclcpp::shutdown();
    return 0;
}
