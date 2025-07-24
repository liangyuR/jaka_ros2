#include "alson_client/alson_client_node.h"
#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto alson_client_node = std::make_shared<alson_client::AlsonClientNode>();
  alson_client_node->set_parameter(rclcpp::Parameter("host", "192.168.0.188"));
  alson_client_node->set_parameter(rclcpp::Parameter("port", 54600));
  rclcpp::spin(alson_client_node);
  rclcpp::shutdown();
  return 0;
}