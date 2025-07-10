#ifndef JAKA_MANIPULATION_AUTOCHARGE_MOVE_GROUP_H_
#define JAKA_MANIPULATION_AUTOCHARGE_MOVE_GROUP_H_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <memory>
#include <string>

namespace jaka_manipulation {

class AutochargeMoveGroup {
 public:
  explicit AutochargeMoveGroup(rclcpp::Node::SharedPtr node, 
                               const std::string& planning_group = "arm");
  ~AutochargeMoveGroup() = default;

  // 初始化MoveIt接口
  bool Initialize();

  // 移动到指定关节位置
  bool MoveToJointPosition(const std::vector<double>& joint_positions, 
                          const std::string& description);

  // 移动到指定笛卡尔位置
  bool MoveToPose(const geometry_msgs::msg::Pose& target_pose, 
                  const std::string& description);

  // 执行预定义动作
  bool MoveToHome();
  bool MoveToPickPosition();
  bool MoveToPlacePosition();

  // 设置运动参数
  void SetVelocityScalingFactor(double factor);
  void SetAccelerationScalingFactor(double factor);

  // 获取当前状态
  std::vector<double> GetCurrentJointValues();
  geometry_msgs::msg::Pose GetCurrentPose();

 private:
  rclcpp::Node::SharedPtr node_;
  std::string planning_group_;
  
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  
  // 预定义位置
  std::vector<double> home_position_;
  std::vector<double> pick_position_;
  std::vector<double> place_position_;

  // 内部辅助函数
  bool ExecutePlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan);
  void LogInfo(const std::string& message);
  void LogError(const std::string& message);
};

}  // namespace jaka_manipulation

#endif  // JAKA_MANIPULATION_AUTOCHARGE_MOVE_GROUP_H_
