#include "jaka_manipulation/alson_client_node.h"
#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto alson_client_node = std::make_shared<jaka_manipulation::AlsonClientNode>();
    alson_client_node->set_parameter(rclcpp::Parameter("host", "127.0.0.1"));
    alson_client_node->set_parameter(rclcpp::Parameter("port", 59999));
    rclcpp::spin(alson_client_node);
    rclcpp::shutdown();
    return 0;
}