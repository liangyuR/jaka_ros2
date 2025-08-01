#include "auto_charge/robot_manager.h"
#include <chrono>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace auto_charge {

RobotManager::RobotManager(QObject *parent) : QObject(parent) {

  node_ = std::make_shared<rclcpp::Node>("robot_manager");
  RCLCPP_INFO(node_->get_logger(), "Robot Manager initialized");

  setupClients();
  setupSubscribers();

  std::thread([this]() {
    std::this_thread::sleep_for(std::chrono::seconds(5));
    this->delayInit();
  }).detach();
}

RobotManager::~RobotManager() {
  RCLCPP_INFO(this->node_->get_logger(), "Robot Manager shutting down");
}

void RobotManager::delayInit() {
  // 获取并更新 IP 参数
  ip_ = getParameter("robot_ip");
  emit ipChanged();
  RCLCPP_INFO(node_->get_logger(), "Robot Manager delayInit: %s", ip_.c_str());
}

void RobotManager::setupClients() {
  // 参数服务客户端
  set_params_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>(
      "/robot_controller/set_parameters");
  get_params_client_ = node_->create_client<rcl_interfaces::srv::GetParameters>(
      "/robot_controller/get_parameters");
  describe_params_client_ =
      node_->create_client<rcl_interfaces::srv::DescribeParameters>(
          "/robot_controller/describe_parameters");

  robot_control_client_ = node_->create_client<jaka_msgs::srv::RobotControl>(
      "/robot_controller/control");
}

void RobotManager::setupSubscribers() {
  // 订阅机器人状态
  robot_status_sub_ = node_->create_subscription<jaka_msgs::msg::RobotStatus>(
      "/robot_controller/detailed_status", 10,
      [this](const jaka_msgs::msg::RobotStatus::SharedPtr msg) {
        // 更新状态变量
        errcode_ = msg->errcode;
        inpos_ = msg->inpos;
        powered_on_ = msg->powered_on;
        enabled_ = msg->enabled;
        rapidrate_ = msg->rapidrate;
        protective_stop_ = msg->protective_stop;
        emergency_stop_ = msg->emergency_stop;
        connected_ = msg->connected;
        cartesiantran_position_ = msg->cartesiantran_position;
        joint_position_ = msg->joint_position;
        current_tool_id_ = msg->current_tool_id;
        current_user_id_ = msg->current_user_id;
        on_soft_limit_ = msg->on_soft_limit;
        drag_status_ = msg->drag_status;
        is_socket_connect_ = msg->is_socket_connect;
        dout_ = msg->dout;
        din_ = msg->din;
        ain_ = msg->ain;
        aout_ = msg->aout;
        tio_dout_ = msg->tio_dout;
        tio_din_ = msg->tio_din;
        tio_ain_ = msg->tio_ain;
        tio_key_ = msg->tio_key;
        timestamp_ = msg->timestamp;

        emit statusChanged();
      });
}

// 参数操作辅助方法
bool RobotManager::setParameter(const std::string &name,
                                const std::string &value) {
  if (!set_params_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "Set parameters service not available");
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
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call set parameters service");
    return false;
  }

  auto result = future.get();
  if (result->results.empty() || !result->results[0].successful) {
    std::string reason =
        result->results.empty() ? "Unknown error" : result->results[0].reason;
    RCLCPP_ERROR(node_->get_logger(), "Failed to set parameter %s: %s",
                 name.c_str(), reason.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Parameter %s set to %s successfully",
              name.c_str(), value.c_str());
  return true;
}

std::string RobotManager::getParameter(const std::string &name) {
  if (!get_params_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "Get parameters service not available");
    return "";
  }

  auto request =
      std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names.push_back(name);

  auto future = get_params_client_->async_send_request(request);

  if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
    auto result = future.get();
    if (!result->values.empty() &&
        result->values[0].type ==
            rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
      return result->values[0].string_value;
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "Failed to get parameter %s", name.c_str());
  return "";
}

void RobotManager::setIp(const QString &ip) {
  ip_ = ip.toStdString();
  emit ipChanged();
}

bool RobotManager::connect() {
  connecting_ = true;
  emit statusChanged();

  auto request = std::make_shared<jaka_msgs::srv::RobotControl::Request>();
  request->command = CMD_CONNECT;

  auto future = robot_control_client_->async_send_request(request);
  bool success =
      (future.wait_for(std::chrono::seconds(20)) == std::future_status::ready);

  connecting_ = false;
  connected_ = success;
  RCLCPP_INFO(node_->get_logger(), "Connect: %s",
              success ? "Success" : "Failed");
  emit statusChanged();
  return success;
}

bool RobotManager::disconnect() {
  connecting_ = true;
  emit statusChanged();

  auto request = std::make_shared<jaka_msgs::srv::RobotControl::Request>();
  request->command = CMD_DISCONNECT;

  auto future = robot_control_client_->async_send_request(request);
  bool success =
      (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready);

  connecting_ = false;
  connected_ = !success ? connected_ : false;
  RCLCPP_INFO(node_->get_logger(), "Disconnect: %s",
              success ? "Success" : "Failed");
  emit statusChanged();
  return success;
}

void RobotManager::updateRobotConnectConfig() { setParameter("robot_ip", ip_); }

bool RobotManager::power(bool power) {
  if (!robot_control_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "Robot control service not available");
    return false;
  }

  auto request = std::make_shared<jaka_msgs::srv::RobotControl::Request>();
  request->command = power ? CMD_POWER_ON : CMD_POWER_OFF;

  auto future = robot_control_client_->async_send_request(request);
  // 上电约 8s 后返回
  if (future.wait_for(std::chrono::seconds(8)) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call robot control service");
    return false;
  }

  auto result = future.get();
  if (!result->success) {
    RCLCPP_ERROR(node_->get_logger(), "Power control failed: %s",
                 result->message.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Power control: %s",
              result->message.c_str());
  return true;
}

bool RobotManager::enable(bool enable) {
  if (!robot_control_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "Robot control service not available");
    return false;
  }

  auto request = std::make_shared<jaka_msgs::srv::RobotControl::Request>();
  request->command = enable ? CMD_ENABLE : CMD_DISABLE;

  auto future = robot_control_client_->async_send_request(request);
  // 使能约 8s 后返回
  if (future.wait_for(std::chrono::seconds(8)) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call robot control service");
    return false;
  }

  auto result = future.get();
  if (!result->success) {
    RCLCPP_ERROR(node_->get_logger(), "Enable control failed: %s",
                 result->message.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Enable control: %s",
              result->message.c_str());
  return true;
}

bool RobotManager::clearError() {
  if (!robot_control_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "Robot control service not available");
    return false;
  }

  auto request = std::make_shared<jaka_msgs::srv::RobotControl::Request>();
  request->command = CMD_CLEAR_ERROR;

  auto future = robot_control_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call robot control service");
    return false;
  }

  auto result = future.get();
  if (!result->success) {
    RCLCPP_ERROR(node_->get_logger(), "Clear error failed: %s",
                 result->message.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Clear error: %s", result->message.c_str());
  return true;
}

bool RobotManager::resetSensor() {
  if (!robot_control_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "Robot control service not available");
    return false;
  }

  auto request = std::make_shared<jaka_msgs::srv::RobotControl::Request>();
  request->command = CMD_RESET_SENSOR;

  auto future = robot_control_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call robot control service");
    return false;
  }

  auto result = future.get();
  if (!result->success) {
    RCLCPP_ERROR(node_->get_logger(), "Reset sensor failed: %s",
                 result->message.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Reset sensor: %s", result->message.c_str());
  return true;
}

} // namespace auto_charge
