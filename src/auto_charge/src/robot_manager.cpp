#include "auto_charge/robot_manager.h"
#include <chrono>

namespace auto_charge {

RobotManager::RobotManager(rclcpp::Node::SharedPtr node)
    : QObject(nullptr), node(node), is_connected_(false), is_enabled_(false),
      has_error_(false) {

  RCLCPP_INFO(this->node->get_logger(), "Robot Manager initialized");

  SetupClients();
  SetupSubscribers();
}

RobotManager::~RobotManager() {
  RCLCPP_INFO(this->node->get_logger(), "Robot Manager shutting down");
}

void RobotManager::SetupClients() {
  // 参数服务客户端
  set_params_client_ = node->create_client<rcl_interfaces::srv::SetParameters>(
      "/robot_controller/set_parameters");
  get_params_client_ = node->create_client<rcl_interfaces::srv::GetParameters>(
      "/robot_controller/get_parameters");
  describe_params_client_ =
      node->create_client<rcl_interfaces::srv::DescribeParameters>(
          "/robot_controller/describe_parameters");

  // 机器人控制服务客户端
  power_client_ =
      node->create_client<std_srvs::srv::SetBool>("/robot_controller/power");
  enable_client_ =
      node->create_client<std_srvs::srv::SetBool>("/robot_controller/enable");
  restart_client_ =
      node->create_client<std_srvs::srv::Empty>("/robot_controller/restart");
  clear_error_client_ = node->create_client<std_srvs::srv::Empty>(
      "/robot_controller/clear_error");
  reset_sensor_client_ = node->create_client<std_srvs::srv::Empty>(
      "/robot_controller/reset_sensor");
}

void RobotManager::SetupSubscribers() {
  // 订阅机器人状态
  robot_status_sub_ = node->create_subscription<jaka_msgs::msg::RobotMsg>(
      "/robot_controller/status", 10,
      std::bind(&RobotManager::RobotStatusCallback, this,
                std::placeholders::_1));

  // 订阅连接状态
  connection_status_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/robot_controller/connection_status", 10,
      std::bind(&RobotManager::ConnectionStatusCallback, this,
                std::placeholders::_1));
}

// 参数操作辅助方法
bool RobotManager::SetParameter(const std::string &name,
                                const std::string &value) {
  if (!set_params_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Set parameters service not available");
    return false;
  }

  auto request =
      std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

  rcl_interfaces::msg::Parameter param;
  param.name = name;
  param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  param.value.string_value = value;

  request->parameters.push_back(param);

  auto future = set_params_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, future,
                                         std::chrono::seconds(5)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = future.get();
    if (result->results[0].successful) {
      RCLCPP_INFO(node->get_logger(), "Parameter %s set to %s successfully",
                  name.c_str(), value.c_str());
      return true;
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to set parameter %s: %s",
                   name.c_str(), result->results[0].reason.c_str());
      return false;
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call set parameters service");
    return false;
  }
}

std::string RobotManager::GetParameter(const std::string &name) {
  if (!get_params_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Get parameters service not available");
    return "";
  }

  auto request =
      std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names.push_back(name);

  auto future = get_params_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, future,
                                         std::chrono::seconds(5)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = future.get();
    if (!result->values.empty() &&
        result->values[0].type ==
            rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
      return result->values[0].string_value;
    }
  }

  RCLCPP_ERROR(node->get_logger(), "Failed to get parameter %s", name.c_str());
  return "";
}

// 状态回调
void RobotManager::RobotStatusCallback(
    const jaka_msgs::msg::RobotMsg::SharedPtr msg) {
  current_status_ = *msg;

  // 更新内部状态
  is_enabled_ = (msg->servo_state == 1);
  has_error_ = (msg->motion_state == 4); // 错误状态

  // 触发 QML 信号
  onStatusChanged(*msg);

  RCLCPP_DEBUG(node->get_logger(),
               "Robot status: motion=%d, power=%d, servo=%d, collision=%d",
               msg->motion_state, msg->power_state, msg->servo_state,
               msg->collision_state);
}

void RobotManager::ConnectionStatusCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  is_connected_ = msg->data;

  // 触发 QML 信号
  onConnectionChanged(msg->data);

  RCLCPP_DEBUG(node->get_logger(), "Connection status: %s",
               msg->data ? "Connected" : "Disconnected");
}

// QML 接口实现
bool RobotManager::setRobotIP(const QString &ip) {
  bool result = SetParameter("robot_ip", ip.toStdString());
  emit operationCompleted("setRobotIP", result);
  if (result) {
    emit configChanged();
  }
  return result;
}

bool RobotManager::setRobotModel(const QString &model) {
  bool result = SetParameter("robot_model", model.toStdString());
  emit operationCompleted("setRobotModel", result);
  if (result) {
    emit configChanged();
  }
  return result;
}

QString RobotManager::getRobotIP() {
  return QString::fromStdString(GetParameter("robot_ip"));
}

QString RobotManager::getRobotModel() {
  return QString::fromStdString(GetParameter("robot_model"));
}

bool RobotManager::connectRobot() {
  RCLCPP_INFO(node->get_logger(), "Connecting to robot...");
  // 这里可以添加连接逻辑，比如重启 robot_controller 节点
  // 或者调用特定的连接服务
  bool result = true;
  emit operationCompleted("connectRobot", result);
  return result;
}

bool RobotManager::disconnectRobot() {
  RCLCPP_INFO(node->get_logger(), "Disconnecting from robot...");
  // 这里可以添加断开连接逻辑
  bool result = true;
  emit operationCompleted("disconnectRobot", result);
  return result;
}

bool RobotManager::powerRobot(bool power) {
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = power;
  bool result = CallService(power_client_, request, node);
  emit operationCompleted("powerRobot", result);
  return result;
}

bool RobotManager::enableRobot(bool enable) {
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enable;
  bool result = CallService(enable_client_, request, node);
  emit operationCompleted("enableRobot", result);
  return result;
}

bool RobotManager::clearError() {
  bool result =
      CallService(clear_error_client_,
                  std::make_shared<std_srvs::srv::Empty::Request>(), node);
  emit operationCompleted("clearError", result);
  return result;
}

bool RobotManager::resetSensor() {
  bool result =
      CallService(reset_sensor_client_,
                  std::make_shared<std_srvs::srv::Empty::Request>(), node);
  emit operationCompleted("resetSensor", result);
  return result;
}

bool RobotManager::isConnected() { return is_connected_; }

bool RobotManager::isEnabled() { return is_enabled_; }

bool RobotManager::hasError() { return has_error_; }

QString RobotManager::getErrorMessage() {
  return QString::fromStdString(error_message_);
}

void RobotManager::onConnectionChanged(bool connected) {
  emit connectionChanged(connected);
}

void RobotManager::onStatusChanged(const jaka_msgs::msg::RobotMsg &status) {
  emit statusChanged();
}

} // namespace auto_charge
