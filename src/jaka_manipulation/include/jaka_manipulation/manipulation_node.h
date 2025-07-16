#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <functional>
#include <chrono>
#include "jaka_manipulation/scene_manager.h"

namespace jaka_manipulation
{

class ManipulationNode : public rclcpp::Node
{
public:
    ManipulationNode();
    ~ManipulationNode() = default;

private:
    // 初始化相关
    void initialization_timer();
    
    // 成员变量
    std::string planning_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    SceneManager* scene_manager_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace jaka_manipulation