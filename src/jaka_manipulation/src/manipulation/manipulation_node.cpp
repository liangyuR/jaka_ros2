#include "jaka_manipulation/manipulation_node.h"

namespace jaka_manipulation
{

ManipulationNode::ManipulationNode() : Node("manipulation_node")
{
    // 声明参数
    this->declare_parameter<std::string>("planning_group", "jaka_zu12");
    planning_group_ = this->get_parameter("planning_group").as_string();

    // 初始化MoveIt接口
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), planning_group_);
    
    // 获取单例SceneManager
    scene_manager_ = SceneManager::GetInstance();

    // 等待MoveIt准备就绪
    auto timer = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() { this->initialization_timer(); });
    timer_ = timer;

    RCLCPP_INFO(this->get_logger(), "Manipulation node initialized");
}

void ManipulationNode::initialization_timer() // NOLINT
{
    static int count = 0;
    count++;
    
    RCLCPP_INFO(this->get_logger(), "Initialization timer triggered, count: %d", count);
    
    // 检查 MoveIt 是否就绪
    try {
        // 尝试获取机器人状态
        auto robot_state = move_group_->getCurrentState();
        if (robot_state) {
            RCLCPP_INFO(this->get_logger(), "MoveIt is ready! Robot state obtained successfully.");
            timer_->cancel();
            // 加载默认场景
            scene_manager_->LoadDefaultScene();
            RCLCPP_INFO(this->get_logger(), "Initialization complete. Default scene loaded.");
            return;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "MoveIt not ready yet: %s", e.what());
    }
    
    // 如果超过10秒还没就绪，强制继续
    if (count >= 10) {
        RCLCPP_WARN(this->get_logger(), "MoveIt not ready after 10 seconds, proceeding anyway...");
        timer_->cancel();
        scene_manager_->LoadDefaultScene();
        RCLCPP_INFO(this->get_logger(), "Initialization complete. Default scene loaded.");
    }
}
} // namespace jaka_manipulation
