#pragma once

#include <QObject>
#include <geometry_msgs/msg/pose.hpp>
#include <jaka_sdk/jktypes.h>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

// 前向声明
namespace auto_charge {
class SceneManager;
}

class TaskManager : public QObject {
  Q_OBJECT

public:
  explicit TaskManager(const std::string &node_name = "task_manager",
                       QObject *parent = nullptr);
  ~TaskManager() override;

  void InitScene();

  // 主要业务接口
  bool Charge();    // 充电流程
  bool ChargeEnd(); // 充电结束流程

private:
  // ROS节点
  std::shared_ptr<rclcpp::Node> ros_node_;

  // 场景管理
  void SetupScene_();      // 加载固定场景信息
  bool FindChargingBox_(); // 查找充电箱位置 & 将充电箱构建到 Scene 中
  bool UpdateChargingBoxPose_(); // 更新充电箱位置

  // 工具操作
  // 不涉及真实物理操作, 只在 sence 中 pick 和 place
  bool PickGun_();  // 末端吸附枪
  bool PlaceGun_(); // 末端释放枪

  // 运动规划
  bool MoveToPose_(const geometry_msgs::msg::Pose &pose);
  bool MoveToJointPosition_(const std::vector<double> &joint_positions);
  bool PlanAndExecute_(const geometry_msgs::msg::Pose &target_pose);

  // 安全检查
  bool CheckSafety_();              // 安全检查
  bool CheckChargingBoxDetected_(); // 检查充电箱是否被检测到

  // 状态管理
  void SetTaskState_(const std::string &state);
  bool IsTaskCompleted_() const;

  // 成员变量
  std::string planning_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>
      planning_scene_interface_;
  auto_charge::SceneManager *scene_manager_;

  // 任务状态
  std::string current_task_state_;
  bool task_completed_;
  bool charging_box_detected_;

  // 位置配置
  geometry_msgs::msg::Pose gun_storage_pose_;  // 枪放置位置
  geometry_msgs::msg::Pose charging_box_pose_; // 充电箱位置
  geometry_msgs::msg::Pose approach_pose_;     // 接近位置
  geometry_msgs::msg::Pose insert_pose_;       // 插入位置

  // 运动参数
  double approach_distance_;
  double planning_time_;
  int max_planning_attempts_;

  // 定时器
  rclcpp::TimerBase::SharedPtr safety_check_timer_;
  rclcpp::TimerBase::SharedPtr status_publish_timer_;

  // 初始化方法
  void InitializeMoveIt_();
  void LoadParameters_();
  void SetupTimers_();
  void SetupPublishers_();
};