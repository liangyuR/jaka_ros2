#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include "jaka_msgs/msg/robot_msg.hpp"
#include "rcl_interfaces/srv/describe_parameters.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <QObject>
#include <QString>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace auto_charge {

class RobotManager : public QObject {
  Q_OBJECT

public:
  explicit RobotManager(rclcpp::Node::SharedPtr node);
  ~RobotManager();

  // QML 接口 - 参数配置功能
  Q_INVOKABLE bool setRobotIP(const QString &ip);
  Q_INVOKABLE bool setRobotModel(const QString &model);
  Q_INVOKABLE QString getRobotIP();
  Q_INVOKABLE QString getRobotModel();

  // QML 接口 - 机器人控制功能
  Q_INVOKABLE bool connectRobot();
  Q_INVOKABLE bool disconnectRobot();
  Q_INVOKABLE bool powerRobot(bool power);   // 上下电
  Q_INVOKABLE bool enableRobot(bool enable); // 使能
  Q_INVOKABLE bool clearError();             // 清除错误
  Q_INVOKABLE bool resetSensor();            // 重置传感器

  // QML 接口 - 状态查询功能
  Q_INVOKABLE bool isConnected();
  Q_INVOKABLE bool isEnabled();
  Q_INVOKABLE bool hasError();
  Q_INVOKABLE QString getErrorMessage();

  // QML 属性
  Q_PROPERTY(bool connected READ isConnected NOTIFY connectionChanged)
  Q_PROPERTY(bool enabled READ isEnabled NOTIFY statusChanged)
  Q_PROPERTY(bool hasError READ hasError NOTIFY statusChanged)
  Q_PROPERTY(QString errorMessage READ getErrorMessage NOTIFY statusChanged)
  Q_PROPERTY(
      QString robotIP READ getRobotIP WRITE setRobotIP NOTIFY configChanged)
  Q_PROPERTY(QString robotModel READ getRobotModel WRITE setRobotModel NOTIFY
                 configChanged)

signals:
  void connectionChanged(bool connected);
  void statusChanged();
  void configChanged();
  void operationCompleted(const QString &operation, bool success);

private slots:
  void onConnectionChanged(bool connected);
  void onStatusChanged(const jaka_msgs::msg::RobotMsg &status);

private:
  rclcpp::Node::SharedPtr node;

  // 服务客户端
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr
      set_params_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr
      get_params_client_;
  rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedPtr
      describe_params_client_;

  // 机器人控制服务客户端
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr power_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr restart_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_error_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_sensor_client_;

  // 状态订阅者
  rclcpp::Subscription<jaka_msgs::msg::RobotMsg>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr connection_status_sub_;

  // 当前状态
  bool is_connected_;
  bool is_enabled_;
  bool has_error_;
  std::string error_message_;
  jaka_msgs::msg::RobotMsg current_status_;

  // 私有方法
  void SetupClients();
  void SetupSubscribers();

  // 参数操作辅助方法
  bool SetParameter(const std::string &name, const std::string &value);
  std::string GetParameter(const std::string &name);

  // 状态回调
  void RobotStatusCallback(const jaka_msgs::msg::RobotMsg::SharedPtr msg);
  void ConnectionStatusCallback(const std_msgs::msg::Bool::SharedPtr msg);
};

// 模板方法 - 必须在类内部完整定义
template <typename ServiceType>
bool CallService(typename rclcpp::Client<ServiceType>::SharedPtr client,
                 typename ServiceType::Request::SharedPtr request,
                 rclcpp::Node::SharedPtr node) {
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Service not available");
    return false;
  }

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future,
                                         std::chrono::seconds(5)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service call successful");
    return true;
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
    return false;
  }
}

} // namespace auto_charge

#endif // ROBOT_MANAGER_H
