#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "jaka_sdk/jkerr.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "jaka_msgs/msg/robot_msg.hpp"
#include "jaka_sdk/JAKAZuRobot.h"
#include "jaka_sdk/jktypes.h"

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace robot_controller {

class RobotController : public rclcpp::Node {
public:
  explicit RobotController(const std::string &node_name = "robot_controller");
  ~RobotController() override;

  // 初始化机器人连接
  bool InitializeRobot();

  // 启动控制器
  void StartController();

  // 停止控制器
  void StopController();

  // 连接管理
  bool ConnectToRobot(const std::string &ip, const std::string &model);
  void DisconnectFromRobot();
  void RestartController();

private:
  // 机器人实例
  JAKAZuRobot robot_;

  // 机器人参数
  std::string robot_ip_;
  std::string robot_model_;

  // 发布器
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<jaka_msgs::msg::RobotMsg>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connection_status_pub_;

  // 动作服务器
  rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr
      action_server_;

  // 服务
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr power_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr restart_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_error_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_sensor_service_;

  // 高级操作服务
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr insert_gun_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pull_gun_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr connect_gun_service_;

  // 定时器
  rclcpp::TimerBase::SharedPtr joint_states_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr connection_monitor_timer_;

  // 控制状态
  bool is_enabled_;
  bool is_connected_;
  bool has_error_;
  std::string error_message_;

  // 错误码映射
  std::map<int, std::string> error_map_;

  // 私有方法
  void SetupErrorMap();
  void SetupPublishers();
  void SetupServices();
  void SetupActionServer();
  void SetupTimers();

  // 状态发布
  void PublishJointStates();
  void PublishRobotStatus();
  void PublishConnectionStatus();

  // 状态监控
  void MonitorConnectionStatus();
  void UpdateRobotStatus(bool connected, bool enabled, bool has_error,
                         const std::string &error_msg = "");

  // 服务回调
  void HandlePowerService(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> &request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> &response);
  void HandleEnableService(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> &request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> &response);
  void HandleRestartService(
      const std::shared_ptr<std_srvs::srv::Empty::Request> &request,
      std::shared_ptr<std_srvs::srv::Empty::Response> &response);
  void HandleClearErrorService(
      const std::shared_ptr<std_srvs::srv::Empty::Request> &request,
      std::shared_ptr<std_srvs::srv::Empty::Response> &response);
  void HandleResetSensorService(
      const std::shared_ptr<std_srvs::srv::Empty::Request> &request,
      std::shared_ptr<std_srvs::srv::Empty::Response> &response);

  // 高级操作服务回调
  void HandleInsertGunService(
      const std::shared_ptr<std_srvs::srv::Empty::Request> &request,
      std::shared_ptr<std_srvs::srv::Empty::Response> &response);
  void HandlePullGunService(
      const std::shared_ptr<std_srvs::srv::Empty::Request> &request,
      std::shared_ptr<std_srvs::srv::Empty::Response> &response);
  void HandleConnectGunService(
      const std::shared_ptr<std_srvs::srv::Empty::Request> &request,
      std::shared_ptr<std_srvs::srv::Empty::Response> &response);

  // 高级操作实现
  bool InsertGun();
  bool PullGun();
  bool ConnectGun();

  // 动作服务器回调
  rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID &uuid,
      const std::shared_ptr<
          const control_msgs::action::FollowJointTrajectory::Goal> &goal);

  rclcpp_action::CancelResponse
  HandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                   control_msgs::action::FollowJointTrajectory>> &goal_handle);

  void HandleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<
          control_msgs::action::FollowJointTrajectory>> &goal_handle);

  // 轨迹执行
  void ExecuteTrajectory(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<
          control_msgs::action::FollowJointTrajectory>> &goal_handle);

  // 位置检查
  bool CheckJointPosition(const JointValue &target_pose,
                          double tolerance = 0.2);

  // 安全停止
  void EmergencyStop();
};

} // namespace robot_controller

#endif // ROBOT_CONTROLLER_H
