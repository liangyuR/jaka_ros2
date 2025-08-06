#include "task/task_manager.h"
#include "auto_charge/scene_manager.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

TaskManager::TaskManager(const std::string &node_name, QObject *parent)
    : QObject(parent), task_completed_(false), charging_box_detected_(false),
      approach_distance_(0.1), planning_time_(5.0), max_planning_attempts_(3) {

  // 创建ROS节点
  ros_node_ = std::make_shared<rclcpp::Node>(node_name);

  LoadParameters_();
  InitializeMoveIt_();
  SetupTimers_();
  SetupPublishers_();

  RCLCPP_INFO(ros_node_->get_logger(), "TaskManager initialized");
}

TaskManager::~TaskManager() = default;

void TaskManager::InitScene() {
  RCLCPP_INFO(ros_node_->get_logger(), "InitScene");
  scene_manager_->ClearScene();
  scene_manager_->LoadDefaultScene();
}

void TaskManager::LoadParameters_() {
  // 加载规划组参数
  planning_group_ = ros_node_->declare_parameter("planning_group", "jaka_zu12");

  // 加载运动参数
  approach_distance_ = ros_node_->declare_parameter("approach_distance", 0.1);
  planning_time_ = ros_node_->declare_parameter("planning_time", 5.0);
  max_planning_attempts_ =
      ros_node_->declare_parameter("max_planning_attempts", 3);
}

void TaskManager::InitializeMoveIt_() {
  try {
    // 初始化 MoveGroup
    move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            ros_node_, planning_group_);

    // 初始化 PlanningSceneInterface
    planning_scene_interface_ =
        std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // 获取 SceneManager 实例
    scene_manager_ = auto_charge::SceneManager::GetInstance();

    // 设置规划参数
    move_group_->setPlanningTime(planning_time_);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);

    RCLCPP_INFO(ros_node_->get_logger(), "MoveIt initialized successfully");

  } catch (const std::exception &e) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Failed to initialize MoveIt: %s",
                 e.what());
  }
}

void TaskManager::SetupTimers_() {
  // 安全检查定时器
  safety_check_timer_ = ros_node_->create_wall_timer(
      std::chrono::milliseconds(100), [this]() { CheckSafety_(); });

  // 状态发布定时器
  status_publish_timer_ = ros_node_->create_wall_timer(
      std::chrono::milliseconds(500), [this]() { /* 发布任务状态 */ });
}

void TaskManager::SetupPublishers_() {
  // 可以添加状态发布器等
}

bool TaskManager::Charge() {
  RCLCPP_INFO(ros_node_->get_logger(), "Starting charging task");
  SetTaskState_("charging_started");

  try {
    // 1. 设置场景
    SetupScene_();

    // 2. 查找充电箱
    if (!FindChargingBox_()) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Failed to find charging box");
      return false;
    }

    // 3. 取枪
    if (!PickGun_()) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Failed to pick gun");
      return false;
    }

    // 4. 移动到接近位置
    if (!MoveToPose_(approach_pose_)) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Failed to move to approach pose");
      return false;
    }

    // 5. 插入充电口
    if (!MoveToPose_(insert_pose_)) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Failed to insert gun");
      return false;
    }

    SetTaskState_("charging_completed");
    RCLCPP_INFO(ros_node_->get_logger(),
                "Charging task completed successfully");
    return true;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Charging task failed: %s", e.what());
    return false;
  }
}

bool TaskManager::ChargeEnd() {
  RCLCPP_INFO(ros_node_->get_logger(), "Starting charge end task");
  SetTaskState_("charge_end_started");

  try {
    // 1. 拔出充电枪
    if (!MoveToPose_(approach_pose_)) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Failed to pull out gun");
      return false;
    }

    // 2. 放回枪
    if (!PlaceGun_()) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Failed to place gun");
      return false;
    }

    SetTaskState_("charge_end_completed");
    RCLCPP_INFO(ros_node_->get_logger(),
                "Charge end task completed successfully");
    return true;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Charge end task failed: %s",
                 e.what());
    return false;
  }
}

void TaskManager::SetupScene_() {
  RCLCPP_INFO(ros_node_->get_logger(), "Setting up scene");

  // 加载默认场景
  scene_manager_->LoadDefaultScene();

  // 可以添加特定的场景对象
  // 例如：工作台、安全区域等
}

bool TaskManager::FindChargingBox_() {
  RCLCPP_INFO(ros_node_->get_logger(), "Searching for charging box");

  // 这里应该实现充电箱检测逻辑
  // 可以通过相机、传感器等方式检测充电箱位置

  // 模拟检测结果
  charging_box_detected_ = true;

  if (charging_box_detected_) {
    // 更新充电箱位置到场景中
    UpdateChargingBoxPose_();

    // 计算接近和插入位置
    // CalculateChargingPoses_();

    return true;
  }

  return false;
}

bool TaskManager::UpdateChargingBoxPose_() {
  // 这里应该从传感器数据更新充电箱的实际位置
  // 暂时使用默认位置

  charging_box_pose_.position.x = 0.8;
  charging_box_pose_.position.y = 0.0;
  charging_box_pose_.position.z = 0.2;

  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  charging_box_pose_.orientation = tf2::toMsg(q);

  // 将充电箱添加到场景中
  auto_charge::SceneObject charging_box;
  charging_box.id = "charging_box";
  charging_box.type = "box";
  charging_box.dimensions = {0.2, 0.15, 0.1};
  charging_box.pose = charging_box_pose_;

  return scene_manager_->AddObject(charging_box);
}

// void TaskManager::CalculateChargingPoses_() {
//   // 计算接近位置（在充电箱前方）
//   approach_pose_ = charging_box_pose_;
//   approach_pose_.position.x -= approach_distance_;

//   // 计算插入位置（充电箱内部）
//   insert_pose_ = charging_box_pose_;
//   insert_pose_.position.x += 0.05; // 稍微插入一点
// }

bool TaskManager::PickGun_() {
  RCLCPP_INFO(ros_node_->get_logger(), "Picking gun from storage");

  // 1. 移动到枪存储位置
  if (!MoveToPose_(gun_storage_pose_)) {
    return false;
  }

  // 2. 执行吸附操作（通过 RobotController）
  // 这里应该调用 RobotController 的相关接口
  // robot_controller_->InsertGun();

  RCLCPP_INFO(ros_node_->get_logger(), "Gun picked successfully");
  return true;
}

bool TaskManager::PlaceGun_() {
  RCLCPP_INFO(ros_node_->get_logger(), "Placing gun back to storage");

  // 1. 移动到枪存储位置
  if (!MoveToPose_(gun_storage_pose_)) {
    return false;
  }

  // 2. 执行释放操作（通过 RobotController）
  // 这里应该调用 RobotController 的相关接口
  // robot_controller_->PullGun();

  RCLCPP_INFO(ros_node_->get_logger(), "Gun placed successfully");
  return true;
}

bool TaskManager::MoveToPose_(const geometry_msgs::msg::Pose &pose) {
  return PlanAndExecute_(pose);
}

bool TaskManager::PlanAndExecute_(const geometry_msgs::msg::Pose &target_pose) {
  move_group_->setPoseTarget(target_pose);

  for (int attempt = 0; attempt < max_planning_attempts_; ++attempt) {
    RCLCPP_INFO(ros_node_->get_logger(), "Planning attempt %d/%d", attempt + 1,
                max_planning_attempts_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_->plan(plan);

    if (result.val == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(ros_node_->get_logger(),
                  "Planning successful, executing trajectory");

      auto execute_result = move_group_->execute(plan);
      if (execute_result.val == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(ros_node_->get_logger(),
                    "Trajectory executed successfully");
        return true;
      } else {
        RCLCPP_WARN(ros_node_->get_logger(), "Trajectory execution failed: %s",
                    execute_result.val ==
                            moveit::core::MoveItErrorCode::PLANNING_FAILED
                        ? "PLANNING_FAILED"
                        : "EXECUTION_FAILED");
      }
    } else {
      RCLCPP_WARN(ros_node_->get_logger(), "Planning failed: %s",
                  result.val == moveit::core::MoveItErrorCode::PLANNING_FAILED
                      ? "PLANNING_FAILED"
                      : "INVALID_MOTION_PLAN");
    }
  }

  RCLCPP_ERROR(ros_node_->get_logger(),
               "Failed to plan and execute after %d attempts",
               max_planning_attempts_);
  return false;
}

bool TaskManager::CheckSafety_() {
  // 实现安全检查逻辑
  // 例如：检查机器人是否在安全范围内
  // 检查是否有紧急停止信号
  // 检查关节限制等

  return true;
}

bool TaskManager::CheckChargingBoxDetected_() { return charging_box_detected_; }

void TaskManager::SetTaskState_(const std::string &state) {
  current_task_state_ = state;
  RCLCPP_INFO(ros_node_->get_logger(), "Task state changed to: %s",
              state.c_str());
}

bool TaskManager::IsTaskCompleted_() const { return task_completed_; }
