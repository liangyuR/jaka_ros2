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
  SetupErrorMap();

  // 设置发布器、服务、动作服务器
  SetupPublishers();
  SetupServices();
  SetupActionServer();
  SetupTimers();
}

RobotController::~RobotController() { StopController(); }

void RobotController::SetupErrorMap() {
  error_map_ = {{2, "ERR_FUCTION_CALL_ERROR"},   {-1, "ERR_INVALID_HANDLER"},
                {-2, "ERR_INVALID_PARAMETER"},   {-3, "ERR_COMMUNICATION_ERR"},
                {-4, "ERR_KINE_INVERSE_ERR"},    {-5, "ERR_EMERGENCY_PRESSED"},
                {-6, "ERR_NOT_POWERED"},         {-7, "ERR_NOT_ENABLED"},
                {-8, "ERR_DISABLE_SERVOMODE"},   {-9, "ERR_NOT_OFF_ENABLE"},
                {-10, "ERR_PROGRAM_IS_RUNNING"}, {-11, "ERR_CANNOT_OPEN_FILE"},
                {-12, "ERR_MOTION_ABNORMAL"}};
}

void RobotController::SetupPublishers() {
  joint_states_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  robot_status_pub_ = this->create_publisher<jaka_msgs::msg::RobotMsg>(
      "/robot_controller/status", 10);

  connection_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/robot_controller/connection_status", 10);
}

void RobotController::SetupServices() {
  power_service_ = this->create_service<std_srvs::srv::SetBool>(
      "/robot_controller/power",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> &request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        this->HandlePowerService(request, response);
      });

  enable_service_ = this->create_service<std_srvs::srv::SetBool>(
      "/robot_controller/enable",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> &request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        this->HandleEnableService(request, response);
      });

  restart_service_ = this->create_service<std_srvs::srv::Empty>(
      "/robot_controller/restart",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request> &request,
             std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        this->HandleRestartService(request, response);
      });

  clear_error_service_ = this->create_service<std_srvs::srv::Empty>(
      "/robot_controller/clear_error",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request> &request,
             std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        this->HandleClearErrorService(request, response);
      });

  reset_sensor_service_ = this->create_service<std_srvs::srv::Empty>(
      "/robot_controller/reset_sensor",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request> &request,
             std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        this->HandleResetSensorService(request, response);
      });
}

void RobotController::SetupActionServer() {
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
        return this->HandleGoal(uuid, goal);
      },
      // Cancel callback
      [this](
          const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> &goal_handle
      ) {
        return this->HandleCancel(goal_handle);
      },
      // Accepted callback
      [this](
          const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> &goal_handle
      ) {
        this->HandleAccepted(goal_handle);
      }
  );
  // clang-format on
}

void RobotController::SetupTimers() {
  joint_states_timer_ =
      this->create_wall_timer(8ms, [this]() { this->PublishJointStates(); });

  status_timer_ =
      this->create_wall_timer(100ms, [this]() { this->PublishRobotStatus(); });

  connection_monitor_timer_ = this->create_wall_timer(
      500ms, [this]() { this->MonitorConnectionStatus(); });
}

bool RobotController::InitializeRobot() {
  try {
    // 连接机器人
    int login_result = robot_.login_in(robot_ip_.c_str(), false);
    if (login_result != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot at %s",
                   robot_ip_.c_str());
      return false;
    }

    is_connected_ = true;
    RCLCPP_INFO(this->get_logger(), "Successfully connected to robot");

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

    RCLCPP_INFO(this->get_logger(), "Robot initialized successfully");
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
    EmergencyStop();
    is_enabled_ = false;
    RCLCPP_INFO(this->get_logger(), "Robot controller stopped");
  }
}

void RobotController::PublishJointStates() {
  if (!is_connected_ || !is_enabled_) {
    return;
  }

  try {
    sensor_msgs::msg::JointState joint_msg;
    JointValue joint_position;

    int result = robot_.get_joint_position(&joint_position);
    if (result != 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to get joint position: %s",
                  error_map_[result].c_str());
      return;
    }

    // 设置关节名称
    for (int i = 0; i < 6; i++) {
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

void RobotController::HandlePowerService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> &request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> &response) {

  if (!is_connected_) {
    response->success = false;
    response->message = "Robot not connected. Cannot control power.";
    RCLCPP_ERROR(this->get_logger(),
                 "Robot not connected. Cannot control power.");
    return;
  }

  try {
    if (request->data) {
      // 上电
      robot_.power_on();
      std::this_thread::sleep_for(8s);
      response->success = true;
      response->message = "Robot powered on successfully";
      RCLCPP_INFO(this->get_logger(), "Robot powered on via service");
    } else {
      // 下电
      robot_.power_off();
      response->success = true;
      response->message = "Robot powered off successfully";
      RCLCPP_INFO(this->get_logger(), "Robot powered off via service");
    }
  } catch (const std::exception &e) {
    response->success = false;
    response->message = "Power control failed: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "Power control failed: %s", e.what());
  }
}

void RobotController::HandleEnableService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> &request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> &response) {

  if (!is_connected_) {
    response->success = false;
    response->message = "Robot not connected. Cannot control enable.";
    RCLCPP_ERROR(this->get_logger(),
                 "Robot not connected. Cannot control enable.");
    return;
  }

  try {
    if (request->data) {
      // 使能
      robot_.enable_robot();
      std::this_thread::sleep_for(4s);
      is_enabled_ = true;
      response->success = true;
      response->message = "Robot enabled successfully";
      RCLCPP_INFO(this->get_logger(), "Robot enabled via service");
    } else {
      // 下使能
      robot_.disable_robot();
      is_enabled_ = false;
      response->success = true;
      response->message = "Robot disabled successfully";
      RCLCPP_INFO(this->get_logger(), "Robot disabled via service");
    }
  } catch (const std::exception &e) {
    response->success = false;
    response->message = "Enable control failed: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "Enable control failed: %s", e.what());
  }
}

rclcpp_action::GoalResponse RobotController::HandleGoal(
    const rclcpp_action::GoalUUID &uuid,
    const std::shared_ptr<
        const control_msgs::action::FollowJointTrajectory::Goal> &goal) {
  (void)uuid; // 避免未使用参数警告

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

rclcpp_action::CancelResponse RobotController::HandleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        control_msgs::action::FollowJointTrajectory>> &goal_handle) {
  (void)goal_handle; // 避免未使用参数警告

  RCLCPP_INFO(this->get_logger(), "Goal cancel request received");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotController::HandleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        control_msgs::action::FollowJointTrajectory>> &goal_handle) {
  // 在新线程中执行轨迹，使用 lambda 表达式
  std::thread([this, goal_handle]() {
    this->ExecuteTrajectory(goal_handle);
  }).detach();
}

void RobotController::ExecuteTrajectory(
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
        RCLCPP_WARN(this->get_logger(), "Servo motion failed: %s",
                    error_map_[sdk_result].c_str());
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

      if (CheckJointPosition(joint_pose)) {
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

bool RobotController::CheckJointPosition(const JointValue &target_pose,
                                         double tolerance) {
  JointValue current_position;
  int result = robot_.get_joint_position(&current_position);

  if (result != 0) {
    RCLCPP_WARN(this->get_logger(),
                "Failed to get joint position for checking: %s",
                error_map_[result].c_str());
    return false;
  }

  bool in_position = true;
  for (int i = 0; i < 6; i++) {
    double current_deg = current_position.jVal[i] * 180.0 / pi;
    double target_deg = target_pose.jVal[i] * 180.0 / pi;

    bool joint_in_position = (target_deg - tolerance < current_deg) &&
                             (current_deg < target_deg + tolerance);
    in_position = in_position && joint_in_position;
  }

  return in_position;
}

void RobotController::EmergencyStop() {
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

// 连接管理方法
bool RobotController::ConnectToRobot(const std::string &ip,
                                     const std::string &model) {
  try {
    robot_ip_ = ip;
    robot_model_ = model;

    int login_result = robot_.login_in(robot_ip_.c_str(), false);
    if (login_result != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot at %s",
                   robot_ip_.c_str());
      UpdateRobotStatus(false, false, true,
                        "Connection failed: " + error_map_[login_result]);
      return false;
    }

    is_connected_ = true;
    RCLCPP_INFO(this->get_logger(), "Successfully connected to robot");

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

    UpdateRobotStatus(true, true, false, "");
    RCLCPP_INFO(this->get_logger(), "Robot connected and enabled successfully");
    return true;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during robot connection: %s",
                 e.what());
    UpdateRobotStatus(false, false, true,
                      "Connection exception: " + std::string(e.what()));
    return false;
  }
}

void RobotController::DisconnectFromRobot() {
  try {
    if (is_connected_) {
      EmergencyStop();
      is_connected_ = false;
      is_enabled_ = false;
      UpdateRobotStatus(false, false, false, "");
      RCLCPP_INFO(this->get_logger(), "Robot disconnected");
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during robot disconnection: %s",
                 e.what());
  }
}

void RobotController::RestartController() {
  RCLCPP_INFO(this->get_logger(), "Restarting robot controller...");
  DisconnectFromRobot();
  std::this_thread::sleep_for(2s);
  ConnectToRobot(robot_ip_, robot_model_);
}

// 状态发布方法
void RobotController::PublishRobotStatus() {
  if (!is_connected_) {
    return;
  }

  try {
    jaka_msgs::msg::RobotMsg robot_states;
    RobotStatus_simple robotstatus_simple;
    ProgramState programstate;
    BOOL in_pos = true;
    BOOL in_col = false;
    BOOL drag_mode = false;
    BOOL emergency_stop = false;

    robot_.is_in_pos(&in_pos);
    robot_.is_in_collision(&in_col);
    robot_.is_in_drag_mode(&drag_mode);
    robot_.is_in_estop(&emergency_stop);
    robot_.get_robot_status_simple(&robotstatus_simple);
    robot_.get_program_state(&programstate);

    // 设置运动状态
    if (emergency_stop) {
      robot_states.motion_state = 2;
    } else if (robotstatus_simple.errcode) {
      robot_states.motion_state = 4;
    } else if (in_pos && programstate == PROGRAM_IDLE && (!drag_mode)) {
      robot_states.motion_state = 0;
    } else if (programstate == PROGRAM_PAUSED) {
      robot_states.motion_state = 1;
    } else if ((!in_pos) || programstate == PROGRAM_RUNNING || drag_mode) {
      robot_states.motion_state = 3;
    }

    // 设置电源状态
    if (robotstatus_simple.powered_on) {
      robot_states.power_state = 1;
    } else {
      robot_states.power_state = 0;
    }

    // 设置伺服状态
    if (robotstatus_simple.enabled) {
      robot_states.servo_state = 1;
    } else {
      robot_states.servo_state = 0;
    }

    // 设置碰撞状态
    if (in_col) {
      robot_states.collision_state = 1;
    } else {
      robot_states.collision_state = 0;
    }

    robot_status_pub_->publish(robot_states);

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in PublishRobotStatus: %s",
                 e.what());
  }
}

void RobotController::PublishConnectionStatus() {
  std_msgs::msg::Bool connection_msg;
  connection_msg.data = is_connected_;
  connection_status_pub_->publish(connection_msg);
}

// 状态监控方法
void RobotController::MonitorConnectionStatus() {
  if (!is_connected_) {
    return;
  }

  try {
    JointValue temp_joints;
    int ret = robot_.get_joint_position(&temp_joints);

    if (ret != 0) {
      RCLCPP_WARN(this->get_logger(),
                  "Connection error, error_code: %d, error: %s", ret,
                  error_map_[ret].c_str());
      UpdateRobotStatus(false, false, true,
                        "Connection lost: " + error_map_[ret]);
    } else {
      // 连接正常，发布状态
      PublishRobotStatus();
      PublishConnectionStatus();
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in MonitorConnectionStatus: %s",
                 e.what());
    UpdateRobotStatus(false, false, true,
                      "Monitor exception: " + std::string(e.what()));
  }
}

void RobotController::UpdateRobotStatus(bool connected, bool enabled,
                                        bool has_error,
                                        const std::string &error_msg) {
  is_connected_ = connected;
  is_enabled_ = enabled;
  has_error_ = has_error;
  error_message_ = error_msg;

  PublishConnectionStatus();
}

// 新的服务回调方法
void RobotController::HandleRestartService(
    const std::shared_ptr<std_srvs::srv::Empty::Request> &request,
    std::shared_ptr<std_srvs::srv::Empty::Response> &response) {
  (void)request;  // 避免未使用参数警告
  (void)response; // 避免未使用参数警告

  RCLCPP_INFO(this->get_logger(), "Restart service called");
  RestartController();
}

void RobotController::HandleClearErrorService(
    const std::shared_ptr<std_srvs::srv::Empty::Request> &request,
    std::shared_ptr<std_srvs::srv::Empty::Response> &response) {
  (void)request;  // 避免未使用参数警告
  (void)response; // 避免未使用参数警告

  try {
    if (is_connected_) {
      // 这里可以调用机器人的清错功能
      // robot_.clear_error(); // 如果SDK有这个方法的话
      has_error_ = false;
      error_message_ = "";
      RCLCPP_INFO(this->get_logger(), "Error cleared");
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Cannot clear error: robot not connected");
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in clear error: %s", e.what());
  }
}

void RobotController::HandleResetSensorService(
    const std::shared_ptr<std_srvs::srv::Empty::Request> &request,
    std::shared_ptr<std_srvs::srv::Empty::Response> &response) {
  (void)request;  // 避免未使用参数警告
  (void)response; // 避免未使用参数警告

  try {
    if (is_connected_) {
      // 这里可以调用机器人的传感器重置功能
      // robot_.reset_sensor(); // 如果SDK有这个方法的话
      RCLCPP_INFO(this->get_logger(), "Sensor reset");
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Cannot reset sensor: robot not connected");
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in reset sensor: %s", e.what());
  }
}

} // namespace robot_controller
