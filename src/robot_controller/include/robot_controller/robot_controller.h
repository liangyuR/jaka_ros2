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

#include "jaka_msgs/msg/robot_status.hpp"
#include "jaka_msgs/srv/robot_control.hpp"
#include "jaka_sdk/JAKAZuRobot.h"
#include "jaka_sdk/jktypes.h"

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace robot_controller {
// 机器人控制命令常量
constexpr uint8_t CMD_CONNECT = 1;
constexpr uint8_t CMD_DISCONNECT = 2;
constexpr uint8_t CMD_POWER_ON = 3;
constexpr uint8_t CMD_POWER_OFF = 4;
constexpr uint8_t CMD_ENABLE = 5;
constexpr uint8_t CMD_DISABLE = 6;
constexpr uint8_t CMD_RESTART = 7;
constexpr uint8_t CMD_CLEAR_ERROR = 8;
constexpr uint8_t CMD_RESET_SENSOR = 9;

// 机器人状态相关常量
constexpr int JOINT_COUNT = 6;
constexpr int IO_SIGNAL_COUNT = 256;
constexpr int TIO_SIGNAL_COUNT = 16;
constexpr int TIO_KEY_COUNT = 3;

class RobotController : public rclcpp::Node {
public:
  explicit RobotController(const std::string &node_name = "robot_controller");
  ~RobotController() override;

  // 启动控制器
  void StartController();

  // 停止控制器
  void StopController();

private:
  // 机器人实例
  JAKAZuRobot robot_;

  // 机器人参数
  std::string robot_ip_;
  std::string robot_model_{"zu12"};

  // 发布器
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<jaka_msgs::msg::RobotStatus>::SharedPtr robot_state_pub_;

  // 动作服务器
  rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr
      action_server_;

  // 统一控制服务
  rclcpp::Service<jaka_msgs::srv::RobotControl>::SharedPtr
      robot_control_service_;

  // 高级操作服务
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr insert_gun_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pull_gun_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr connect_gun_service_;

  // 定时器
  rclcpp::TimerBase::SharedPtr joint_states_timer_;
  rclcpp::TimerBase::SharedPtr detailed_robot_status_timer_;
  rclcpp::TimerBase::SharedPtr connection_monitor_timer_;

  // 控制状态
  bool is_enabled_;
  bool is_connected_;
  bool has_error_;
  std::string error_message_;

  std::map<int, std::string> error_map_;

  bool connect(const std::string &ip, const std::string &model);
  void disconnect();
  void restart();

  void setupErrorMap();
  void setupPublishers();
  void setupServices();
  void setupActionServer();
  void setupTimers();

  bool performRobotConnection();
  bool performRobotInitialization();

  // 状态发布
  void publishJointStates();
  void publishDetailedRobotStatus();

  void handleRobotControlService(
      const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
      std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response);

  // 私有命令处理方法
  void handleConnectCommand(
      const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
      std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response);
  void handleDisconnectCommand(
      const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
      std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response);
  void handlePowerOnCommand(
      const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
      std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response);
  void handlePowerOffCommand(
      const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
      std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response);
  void handleEnableCommand(
      const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
      std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response);
  void handleDisableCommand(
      const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
      std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response);
  void handleRestartCommand(
      const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
      std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response);
  void handleClearErrorCommand(
      const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
      std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response);
  void handleResetSensorCommand(
      const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
      std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response);

  // 高级操作服务回调
  void handleInsertGunService(
      const std::shared_ptr<std_srvs::srv::Empty::Request> &request,
      std::shared_ptr<std_srvs::srv::Empty::Response> &response);
  void handlePullGunService(
      const std::shared_ptr<std_srvs::srv::Empty::Request> &request,
      std::shared_ptr<std_srvs::srv::Empty::Response> &response);
  void handleConnectGunService(
      const std::shared_ptr<std_srvs::srv::Empty::Request> &request,
      std::shared_ptr<std_srvs::srv::Empty::Response> &response);

  // 高级操作实现
  bool insertGun();
  bool pullGun();
  bool connectGun();

  // 动作服务器回调
  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID &uuid,
      const std::shared_ptr<
          const control_msgs::action::FollowJointTrajectory::Goal> &goal);

  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                   control_msgs::action::FollowJointTrajectory>> &goal_handle);

  void handleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<
          control_msgs::action::FollowJointTrajectory>> &goal_handle);

  // 轨迹执行
  void executeTrajectory(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<
          control_msgs::action::FollowJointTrajectory>> &goal_handle);

  // 位置检查
  bool checkJointPosition(const JointValue &target_pose,
                          double tolerance = 0.2);

  // 安全停止
  void emergencyStop();
};

} // namespace robot_controller

#endif // ROBOT_CONTROLLER_H
