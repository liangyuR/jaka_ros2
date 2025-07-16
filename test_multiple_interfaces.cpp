#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_node");
    
    // 创建两个接口实例
    auto interface_A = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    auto interface_B = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    
    // A添加对象
    moveit_msgs::msg::CollisionObject obj;
    obj.id = "test_object";
    obj.header.frame_id = "world";
    // ... 设置对象属性
    
    interface_A->addCollisionObjects({obj});
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    
    // 检查两个接口的状态
    auto objects_A = interface_A->getKnownObjectNames();
    auto objects_B = interface_B->getKnownObjectNames();
    
    std::cout << "Interface A knows " << objects_A.size() << " objects" << std::endl;
    std::cout << "Interface B knows " << objects_B.size() << " objects" << std::endl;
    
    // 结果：A和B可能返回不同的结果！
    
    rclcpp::shutdown();
    return 0;
} 