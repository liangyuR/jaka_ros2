#include "robot_controller/robot_controller.h"
#include <csignal>
#include <numbers>

namespace robot_controller {

using std::numbers::pi;

RobotController::RobotController(const std::string &node_name)
    : Node(node_name), is_enabled_(false), is_connected_(false),
      has_error_(false) {

  // 读取参数
  robot_ip_ = this->declare_parameter("robot_ip", "192.168.0.122");
  robot_model_ = this->declare_parameter("robot_model", "zu12");

  RCLCPP_INFO(this->get_logger(), "Robot IP: %s, Model: %s", robot_ip_.c_str(),
              robot_model_.c_str());

  // 设置错误映射
  setupErrorMap();

  // 设置发布器、服务、动作服务器
  setupPublishers();
  setupServices();
  setupActionServer();
  setupTimers();
}

RobotController::~RobotController() { StopController(); }

void RobotController::setupErrorMap() {
  error_map_ = {{2, "ERR_FUCTION_CALL_ERROR"},   {-1, "ERR_INVALID_HANDLER"},
                {-2, "ERR_INVALID_PARAMETER"},   {-3, "ERR_COMMUNICATION_ERR"},
                {-4, "ERR_KINE_INVERSE_ERR"},    {-5, "ERR_EMERGENCY_PRESSED"},
                {-6, "ERR_NOT_POWERED"},         {-7, "ERR_NOT_ENABLED"},
                {-8, "ERR_DISABLE_SERVOMODE"},   {-9, "ERR_NOT_OFF_ENABLE"},
                {-10, "ERR_PROGRAM_IS_RUNNING"}, {-11, "ERR_CANNOT_OPEN_FILE"},
                {-12, "ERR_MOTION_ABNORMAL"}};
}

void RobotController::setupPublishers() {
  joint_states_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  robot_state_pub_ = this->create_publisher<jaka_msgs::msg::RobotStatus>(
      "/robot_controller/detailed_status", 10);
}

void RobotController::setupServices() {
  robot_control_service_ = this->create_service<jaka_msgs::srv::RobotControl>(
      "/robot_controller/control",
      [this](
          const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
          std::shared_ptr<jaka_msgs::srv::RobotControl::Response> response) {
        this->handleRobotControlService(request, response);
      });
}

void RobotController::setupActionServer() {
  std::string action_name =
      "/jaka_" + robot_model_ + "_controller/follow_joint_trajectory";

  // clang-format off
  action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
      this,
      action_name,
      // Goal callback
      [this](
          const rclcpp_action::GoalUUID &uuid,
          const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> &goal
      ) {
        return this->handleGoal(uuid, goal);
      },
      // Cancel callback
      [this](
          const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> &goal_handle
      ) {
        return this->handleCancel(goal_handle);
      },
      // Accepted callback
      [this](
          const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> &goal_handle
      ) {
        this->handleAccepted(goal_handle);
      }
  );
  // clang-format on
}

void RobotController::setupTimers() {
  joint_states_timer_ =
      this->create_wall_timer(8ms, [this]() { this->publishJointStates(); });

  detailed_robot_status_timer_ = this->create_wall_timer(
      100ms, [this]() { this->publishDetailedRobotStatus(); });
}

void RobotController::handleRobotControlService(
    const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
    std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response) {

  try {
    switch (request->command) {
    case CMD_CONNECT:
      handleConnectCommand(request, response);
      break;
    case CMD_DISCONNECT:
      handleDisconnectCommand(request, response);
      break;
    case CMD_POWER_ON:
      handlePowerOnCommand(request, response);
      break;
    case CMD_POWER_OFF:
      handlePowerOffCommand(request, response);
      break;
    case CMD_ENABLE:
      handleEnableCommand(request, response);
      break;
    case CMD_DISABLE:
      handleDisableCommand(request, response);
      break;
    case CMD_RESTART:
      handleRestartCommand(request, response);
      break;
    case CMD_CLEAR_ERROR:
      handleClearErrorCommand(request, response);
      break;
    case CMD_RESET_SENSOR:
      handleResetSensorCommand(request, response);
      break;
    default:
      response->success = false;
      response->message =
          "Unknown command: " + std::to_string(request->command);
      RCLCPP_ERROR(this->get_logger(), "Unknown command: %d", request->command);
      break;
    }
  } catch (const std::exception &e) {
    response->success = false;
    response->message = "Service execution failed: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "Service execution failed: %s", e.what());
  }
}

// 私有命令处理方法
void RobotController::handleConnectCommand(
    const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
    std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response) {
  (void)request;

  if (is_connected_) {
    response->success = true;
    response->message = "Robot already connected.";
    RCLCPP_INFO(this->get_logger(), "Robot already connected.");
    return;
  }

  const std::string ip = this->get_parameter("robot_ip").as_string(); // NOLINT
  const std::string model = this->get_parameter("robot_model").as_string();

  if (!connect(ip, model)) {
    is_connected_ = false;
    response->success = false;
    response->message = "Failed to connect to robot.";
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot.");
    return;
  }

  is_connected_ = true;
  response->success = true;
  response->message = "Robot connected successfully";
  RCLCPP_INFO(this->get_logger(), "Robot connected successfully");
}

void RobotController::handleDisconnectCommand(
    const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
    std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response) {
  (void)request;

  if (!is_connected_) {
    response->success = false;
    response->message = "Robot not connected. Cannot disconnect.";
    RCLCPP_ERROR(this->get_logger(), "Robot not connected. Cannot disconnect.");
    return;
  }

  disconnect();
  response->success = true;
  response->message = "Robot disconnected successfully";
  RCLCPP_INFO(this->get_logger(), "Robot disconnected successfully");
}

void RobotController::handlePowerOnCommand(
    const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
    std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response) {
  (void)request;

  if (!is_connected_) {
    response->success = false;
    response->message = "Robot not connected. Cannot control power.";
    RCLCPP_ERROR(this->get_logger(),
                 "Robot not connected. Cannot control power.");
    return;
  }

  robot_.power_on();
  std::this_thread::sleep_for(8s);
  response->success = true;
  response->message = "Robot powered on successfully";
  RCLCPP_INFO(this->get_logger(), "Robot powered on via service");
}

void RobotController::handlePowerOffCommand(
    const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
    std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response) {
  (void)request;

  if (!is_connected_) {
    response->success = false;
    response->message = "Robot not connected. Cannot control power.";
    RCLCPP_ERROR(this->get_logger(),
                 "Robot not connected. Cannot control power.");
    return;
  }

  robot_.power_off();
  response->success = true;
  response->message = "Robot powered off successfully";
  RCLCPP_INFO(this->get_logger(), "Robot powered off via service");
}

void RobotController::handleEnableCommand(
    const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
    std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response) {
  (void)request;

  if (!is_connected_) {
    response->success = false;
    response->message = "Robot not connected. Cannot control enable.";
    RCLCPP_ERROR(this->get_logger(),
                 "Robot not connected. Cannot control enable.");
    return;
  }

  robot_.enable_robot();
  std::this_thread::sleep_for(4s);
  is_enabled_ = true;
  response->success = true;
  response->message = "Robot enabled successfully";
  RCLCPP_INFO(this->get_logger(), "Robot enabled via service");
}

void RobotController::handleDisableCommand(
    const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
    std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response) {
  (void)request;

  if (!is_connected_) {
    response->success = false;
    response->message = "Robot not connected. Cannot control enable.";
    RCLCPP_ERROR(this->get_logger(),
                 "Robot not connected. Cannot control enable.");
    return;
  }

  robot_.disable_robot();
  is_enabled_ = false;
  response->success = true;
  response->message = "Robot disabled successfully";
  RCLCPP_INFO(this->get_logger(), "Robot disabled via service");
}

void RobotController::handleRestartCommand(
    const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
    std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response) {
  (void)request;

  RCLCPP_INFO(this->get_logger(), "Restart service called");
  restart();
  response->success = true;
  response->message = "Robot restarted successfully";
}

void RobotController::handleClearErrorCommand(
    const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
    std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response) {
  (void)request;

  if (!is_connected_) {
    response->success = false;
    response->message = "Robot not connected. Cannot clear error.";
    RCLCPP_ERROR(this->get_logger(),
                 "Robot not connected. Cannot clear error.");
    return;
  }

  robot_.clear_error();
  error_message_ = "";
  has_error_ = false;
  response->success = true;
  response->message = "Error cleared successfully";
  RCLCPP_INFO(this->get_logger(), "Error cleared");
}

void RobotController::handleResetSensorCommand(
    const std::shared_ptr<jaka_msgs::srv::RobotControl::Request> &request,
    std::shared_ptr<jaka_msgs::srv::RobotControl::Response> &response) {
  (void)request;

  if (!is_connected_) {
    response->success = false;
    response->message = "Robot not connected. Cannot reset sensor.";
    RCLCPP_ERROR(this->get_logger(),
                 "Robot not connected. Cannot reset sensor.");
    return;
  }

  // robot_.reset_sensor(); // TODO(@liangyu): 需要实现

  response->success = true;
  response->message = "Sensor reset successfully";
  RCLCPP_INFO(this->get_logger(), "Sensor reset");
}

rclcpp_action::GoalResponse RobotController::handleGoal(
    const rclcpp_action::GoalUUID &uuid,
    const std::shared_ptr<
        const control_msgs::action::FollowJointTrajectory::Goal> &goal) {
  (void)uuid;

  if (!is_enabled_) {
    RCLCPP_WARN(this->get_logger(), "Controller not enabled. Rejecting goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->trajectory.points.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "Trajectory has no points. Rejecting goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(), "Goal accepted");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotController::handleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        control_msgs::action::FollowJointTrajectory>> &goal_handle) {
  (void)goal_handle;

  RCLCPP_INFO(this->get_logger(), "Goal cancel request received");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotController::handleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        control_msgs::action::FollowJointTrajectory>> &goal_handle) {
  // 在新线程中执行轨迹，使用 lambda 表达式
  std::thread([this, goal_handle]() {
    this->executeTrajectory(goal_handle);
  }).detach();
}

void RobotController::executeTrajectory(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        control_msgs::action::FollowJointTrajectory>> &goal_handle) {

  try {
    // 启用伺服模式
    robot_.servo_move_enable(true); // NOLINT

    auto goal = goal_handle->get_goal();
    const auto &traj = goal->trajectory;
    int point_num = traj.points.size();

    RCLCPP_INFO(this->get_logger(), "Executing trajectory with %d points",
                point_num);

    float last_duration = 0.0;
    JointValue joint_pose;

    for (int i = 1; i < point_num; i++) {
      // 检查是否被取消
      if (goal_handle->is_canceling()) {
        robot_.motion_abort();
        robot_.servo_move_enable(false); // NOLINT
        RCLCPP_INFO(this->get_logger(), "Trajectory execution canceled");

        auto result = std::make_shared<
            control_msgs::action::FollowJointTrajectory::Result>();
        goal_handle->canceled(result);
        return;
      }

      // 获取目标位置
      for (int j = 0; j < 6; j++) {
        joint_pose.jVal[j] = traj.points[i].positions[j];
      }

      // 计算时间间隔
      float duration =
          static_cast<float>(traj.points[i].time_from_start.sec) +
          static_cast<float>(traj.points[i].time_from_start.nanosec) * 1e-9;
      float dt = duration - last_duration; // NOLINT
      last_duration = duration;

      // 计算步数
      int step_num = std::max(static_cast<int>(dt / 0.008f), 1); // NOLINT

      // 执行运动
      int sdk_result = robot_.servo_j(&joint_pose, MoveMode::ABS, step_num);
      if (sdk_result != 0) {
        auto error_it = error_map_.find(sdk_result);
        std::string error_msg =
            (error_it != error_map_.end())
                ? error_it->second
                : "Unknown error code: " + std::to_string(sdk_result);
        RCLCPP_WARN(this->get_logger(), "Servo motion failed: %s",
                    error_msg.c_str());
      }

      RCLCPP_DEBUG(this->get_logger(),
                   "Point %d: joints=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f], "
                   "dt=%.3f, steps=%d",
                   i, joint_pose.jVal[0], joint_pose.jVal[1],
                   joint_pose.jVal[2], joint_pose.jVal[3], joint_pose.jVal[4],
                   joint_pose.jVal[5], dt, step_num);
    }

    // 等待到达最终位置
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        robot_.motion_abort();
        robot_.servo_move_enable(false); // NOLINT
        RCLCPP_INFO(this->get_logger(), "Trajectory execution canceled");

        auto result = std::make_shared<
            control_msgs::action::FollowJointTrajectory::Result>();
        goal_handle->canceled(result);
        return;
      }

      if (checkJointPosition(joint_pose)) {
        robot_.servo_move_enable(false); // NOLINT
        RCLCPP_INFO(this->get_logger(),
                    "Trajectory execution completed successfully");
        break;
      }

      std::this_thread::sleep_for(500ms);
    }

    // 返回成功结果
    auto result =
        std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    goal_handle->succeed(result);

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in trajectory execution: %s",
                 e.what());
    robot_.servo_move_enable(false); // NOLINT

    auto result =
        std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    goal_handle->abort(result);
  }
}

bool RobotController::checkJointPosition(const JointValue &target_pose,
                                         double tolerance) {
  JointValue current_position;
  int result = robot_.get_joint_position(&current_position);

  if (result != 0) {
    auto error_it = error_map_.find(result);
    std::string error_msg =
        (error_it != error_map_.end())
            ? error_it->second
            : "Unknown error code: " + std::to_string(result);
    RCLCPP_WARN(this->get_logger(),
                "Failed to get joint position for checking: %s",
                error_msg.c_str());
    return false;
  }

  bool in_position = true;
  for (int i = 0; i < JOINT_COUNT; i++) {
    double current_deg = current_position.jVal[i] * 180.0 / pi;
    double target_deg = target_pose.jVal[i] * 180.0 / pi;

    bool joint_in_position = (target_deg - tolerance < current_deg) &&
                             (current_deg < target_deg + tolerance);
    in_position = in_position && joint_in_position;
  }

  return in_position;
}

void RobotController::emergencyStop() {
  try {
    if (is_connected_) {
      robot_.motion_abort();
      robot_.servo_move_enable(false); // NOLINT
      RCLCPP_WARN(this->get_logger(), "Emergency stop executed");
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during emergency stop: %s",
                 e.what());
  }
}

// 重构后的 ConnectToRobot 方法，提取公共逻辑
bool RobotController::connect(const std::string &ip, const std::string &model) {
  try {
    robot_ip_ = ip;
    robot_model_ = model;

    if (!performRobotConnection()) {
      return false;
    }

    if (!performRobotInitialization()) {
      return false;
    }

    StartController();

    RCLCPP_INFO(this->get_logger(), "Robot connected and enabled successfully");
    return true;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during robot connection: %s",
                 e.what());
    return false;
  }
}

bool RobotController::performRobotConnection() {
  int login_result = robot_.login_in(robot_ip_.c_str(), false);
  if (login_result != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot at %s",
                 robot_ip_.c_str());
    return false;
  }

  is_connected_ = true;
  RCLCPP_INFO(this->get_logger(), "Successfully connected to robot");
  return true;
}

bool RobotController::performRobotInitialization() {
  try {
    // 关闭伺服模式
    robot_.servo_move_enable(false); // NOLINT
    std::this_thread::sleep_for(500ms);

    // 设置滤波器参数
    robot_.servo_move_use_joint_LPF(0.5);

    // 上电并启用
    robot_.power_on();
    std::this_thread::sleep_for(8s);
    robot_.enable_robot();
    std::this_thread::sleep_for(4s);

    return true;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Exception during robot initialization: %s", e.what());
    return false;
  }
}

void RobotController::StartController() {
  if (!is_connected_) {
    RCLCPP_ERROR(this->get_logger(),
                 "Robot not connected. Cannot start controller.");
    return;
  }

  is_enabled_ = true;
  RCLCPP_INFO(this->get_logger(), "Robot controller started");
}

void RobotController::StopController() {
  if (is_enabled_) {
    emergencyStop();
    is_enabled_ = false;
    RCLCPP_INFO(this->get_logger(), "Robot controller stopped");
  }
}

void RobotController::publishJointStates() {
  if (!is_connected_ || !is_enabled_) {
    return;
  }

  try {
    sensor_msgs::msg::JointState joint_msg;
    JointValue joint_position;

    int result = robot_.get_joint_position(&joint_position);
    if (result != 0) {
      auto error_it = error_map_.find(result);
      std::string error_msg =
          (error_it != error_map_.end())
              ? error_it->second
              : "Unknown error code: " + std::to_string(result);
      RCLCPP_WARN(this->get_logger(), "Failed to get joint position: %s",
                  error_msg.c_str());
      return;
    }

    // 设置关节名称
    for (int i = 0; i < JOINT_COUNT; i++) {
      joint_msg.name.push_back("joint_" + std::to_string(i + 1));
      joint_msg.position.push_back(joint_position.jVal[i]);
    }

    joint_msg.header.stamp = this->now();
    joint_states_pub_->publish(joint_msg);

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in PublishJointStates: %s",
                 e.what());
  }
}

void RobotController::disconnect() {
  try {
    if (is_connected_) {
      emergencyStop();
      is_connected_ = false;
      is_enabled_ = false;
      RCLCPP_INFO(this->get_logger(), "Robot disconnected");
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during robot disconnection: %s",
                 e.what());
  }
}

void RobotController::restart() {
  RCLCPP_INFO(this->get_logger(), "Restarting robot controller...");
  disconnect();
  std::this_thread::sleep_for(2s);
  connect(robot_ip_, robot_model_);
}

void RobotController::publishDetailedRobotStatus() {
  if (!is_connected_) {
    return;
  }

  try {
    jaka_msgs::msg::RobotStatus detailed_status;
    RobotStatus robot_status;

    // 获取完整的机器人状态
    int result = robot_.get_robot_status(&robot_status);
    if (result != 0) {
      auto error_it = error_map_.find(result);
      std::string error_msg =
          (error_it != error_map_.end())
              ? error_it->second
              : "Unknown error code: " + std::to_string(result);
      RCLCPP_WARN(this->get_logger(), "Failed to get robot status: %s",
                  error_msg.c_str());
      return;
    }

    // 设置基础状态信息
    detailed_status.errcode = robot_status.errcode;
    detailed_status.inpos = (robot_status.inpos != 0);
    detailed_status.powered_on = (robot_status.powered_on != 0);
    detailed_status.enabled = (robot_status.enabled != 0);
    detailed_status.rapidrate = robot_status.rapidrate;
    detailed_status.protective_stop = (robot_status.protective_stop != 0);
    detailed_status.emergency_stop = (robot_status.emergency_stop != 0);
    detailed_status.connected = is_connected_;

    // 设置位置信息
    detailed_status.cartesiantran_position.clear();
    detailed_status.joint_position.clear();
    for (int i = 0; i < JOINT_COUNT; i++) {
      detailed_status.cartesiantran_position.push_back(
          robot_status.cartesiantran_position[i]);
      detailed_status.joint_position.push_back(robot_status.joint_position[i]);
    }

    // 设置坐标系信息
    detailed_status.current_tool_id = robot_status.current_tool_id;
    detailed_status.current_user_id = robot_status.current_user_id;
    detailed_status.on_soft_limit = (robot_status.on_soft_limit != 0);

    // 设置运动状态
    detailed_status.drag_status = (robot_status.drag_status != 0);
    detailed_status.is_socket_connect = (robot_status.is_socket_connect != 0);

    // 设置数字IO信号
    for (int i = 0; i < IO_SIGNAL_COUNT; i++) {
      detailed_status.dout.push_back(robot_status.dout[i]);
      detailed_status.din.push_back(robot_status.din[i]);
      detailed_status.ain.push_back(robot_status.ain[i]);
      detailed_status.aout.push_back(robot_status.aout[i]);
    }

    // 设置工具端IO信号
    for (int i = 0; i < TIO_SIGNAL_COUNT; i++) {
      detailed_status.tio_dout.push_back(robot_status.tio_dout[i]);
      detailed_status.tio_din.push_back(robot_status.tio_din[i]);
      detailed_status.tio_ain.push_back(robot_status.tio_ain[i]);
    }

    // 设置工具按键
    for (int &i : robot_status.tio_key) {
      detailed_status.tio_key.push_back(i);
    }

    // 设置时间戳
    detailed_status.timestamp = this->now();

    // 发布详细状态
    robot_state_pub_->publish(detailed_status);

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Exception in PublishDetailedRobotStatus: %s", e.what());
  }
}

} // namespace robot_controller
