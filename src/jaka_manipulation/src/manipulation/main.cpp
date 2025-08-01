#include "jaka_manipulation/manipulation_node.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<jaka_manipulation::ManipulationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}