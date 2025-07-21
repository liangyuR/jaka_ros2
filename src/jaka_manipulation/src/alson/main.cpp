#include "jaka_manipulation/alson_client.h"
#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("alson_client_node");
    const auto ip = node->get_parameter("camera_ip").as_string(); // NOLINT
    const auto port = node->get_parameter("camera_port").as_int();
    jaka_manipulation::AlsonClient::GetInstance(node, ip, port);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}