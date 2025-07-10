#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <functional>
#include <chrono>

class ManipulationNode : public rclcpp::Node
{
public:
    ManipulationNode() : Node("manipulation_node")
    {
        RCLCPP_INFO(get_logger(), "初始化MoveIt接口: %s", planning_group_.c_str());
        
        // 初始化MoveIt接口
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), planning_group_);
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        
        // 创建服务
        pick_service_ = this->create_service<std_srvs::srv::SetBool>(
            "pick_object", 
            std::bind(&ManipulationNode::pickCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
                     
        place_service_ = this->create_service<std_srvs::srv::SetBool>(
            "place_object",
            std::bind(&ManipulationNode::placeCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // 设置基本参数
        move_group_->setMaxVelocityScalingFactor(0.3);
        move_group_->setMaxAccelerationScalingFactor(0.3);
        
        RCLCPP_INFO(this->get_logger(), "机械臂操作节点初始化完成!");
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::string planning_group_;
    
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pick_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr place_service_;
    
    // 预定义位置
    std::vector<double> home_position_ = {0.0, 1.57, -1.57, 1.57, 1.57, 0.0};
    std::vector<double> pick_position_ = {1.0, 1.2, -1.2, 1.57, 1.57, 0.0};
    std::vector<double> place_position_ = {-1.0, 1.2, -1.2, 1.57, 1.57, 0.0};

    void pickCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                     std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "开始执行取物操作...");
        
        bool success = true;
        
        try {
            // 1. 移动到Home位置
            if (!moveToJointPosition(home_position_, "Home位置")) {
                success = false;
            }
            
            // 2. 移动到取物位置
            if (success && !moveToJointPosition(pick_position_, "取物位置")) {
                success = false;
            }
            
            // 3. 这里可以添加夹爪控制
            if (success) {
                RCLCPP_INFO(this->get_logger(), "夹爪闭合 - 抓取物体");
                rclcpp::sleep_for(std::chrono::milliseconds(1000));
            }
            
            // 4. 返回Home位置
            if (success && !moveToJointPosition(home_position_, "返回Home位置")) {
                success = false;
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "取物操作异常: %s", e.what());
            success = false;
        }
        
        response->success = success;
        response->message = success ? "取物操作完成" : "取物操作失败";
    }
    
    void placeCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "开始执行放物操作...");
        
        bool success = true;
        
        try {
            // 1. 移动到放物位置
            if (!moveToJointPosition(place_position_, "放物位置")) {
                success = false;
            }
            
            // 2. 夹爪松开
            if (success) {
                RCLCPP_INFO(this->get_logger(), "夹爪张开 - 释放物体");
                rclcpp::sleep_for(std::chrono::milliseconds(1000));
            }
            
            // 3. 返回Home位置
            if (success && !moveToJointPosition(home_position_, "返回Home位置")) {
                success = false;
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "放物操作异常: %s", e.what());
            success = false;
        }
        
        response->success = success;
        response->message = success ? "放物操作完成" : "放物操作失败";
    }
    
    bool moveToJointPosition(const std::vector<double>& joint_positions, const std::string& description)
    {
        RCLCPP_INFO(this->get_logger(), "移动到: %s", description.c_str());
        
        move_group_->setStartStateToCurrentState();
        move_group_->setJointValueTarget(joint_positions);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            move_group_->execute(plan);
            RCLCPP_INFO(this->get_logger(), "成功到达: %s", description.c_str());
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "规划失败: %s", description.c_str());
            return false;
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulationNode>();
    
    RCLCPP_INFO(node->get_logger(), "机械臂操作节点启动中...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}