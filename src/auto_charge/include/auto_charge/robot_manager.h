#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include "builtin_interfaces/msg/time.hpp"
#include "jaka_msgs/msg/robot_status.hpp"
#include "rcl_interfaces/srv/describe_parameters.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <jaka_msgs/srv/robot_control.hpp>

// 机器人控制命令常量
namespace auto_charge {
constexpr uint8_t CMD_CONNECT = 1;
constexpr uint8_t CMD_DISCONNECT = 2;
constexpr uint8_t CMD_POWER_ON = 3;
constexpr uint8_t CMD_POWER_OFF = 4;
constexpr uint8_t CMD_ENABLE = 5;
constexpr uint8_t CMD_DISABLE = 6;
constexpr uint8_t CMD_RESTART = 7;
constexpr uint8_t CMD_CLEAR_ERROR = 8;
constexpr uint8_t CMD_RESET_SENSOR = 9;
} // namespace auto_charge

#include <QObject>
#include <QString>
#include <QVariant>
#include <QVector>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace auto_charge {

class RobotManager : public QObject {
  Q_OBJECT

public:
  explicit RobotManager(QObject *parent = nullptr);
  ~RobotManager() override;

  Q_PROPERTY(QString ip READ getIp WRITE setIp NOTIFY ipChanged)
  Q_PROPERTY(bool connected READ getConnected NOTIFY statusChanged)
  Q_PROPERTY(bool connecting READ getConnecting NOTIFY statusChanged)
  Q_PROPERTY(bool enabled READ getEnabled NOTIFY statusChanged)
  Q_PROPERTY(bool error READ getError NOTIFY statusChanged)
  Q_PROPERTY(int errcode READ getErrcode NOTIFY statusChanged)
  Q_PROPERTY(bool inpos READ getInpos NOTIFY statusChanged)
  Q_PROPERTY(double rapidrate READ getRapidrate NOTIFY statusChanged)
  Q_PROPERTY(bool protectiveStop READ getProtectiveStop NOTIFY statusChanged)
  Q_PROPERTY(bool emergencyStop READ getEmergencyStop NOTIFY statusChanged)
  Q_PROPERTY(bool onSoftLimit READ getOnSoftLimit NOTIFY statusChanged)
  Q_PROPERTY(bool dragStatus READ getDragStatus NOTIFY statusChanged)
  Q_PROPERTY(bool poweredOn READ getPoweredOn NOTIFY statusChanged)
  Q_PROPERTY(
      QVariant cartesianPosition READ getCartesianPosition NOTIFY statusChanged)
  Q_PROPERTY(QVariant jointPosition READ getJointPosition NOTIFY statusChanged)
  Q_PROPERTY(uint currentToolId READ getCurrentToolId NOTIFY statusChanged)
  Q_PROPERTY(uint currentUserId READ getCurrentUserId NOTIFY statusChanged)
  Q_PROPERTY(bool isSocketConnect READ getIsSocketConnect NOTIFY statusChanged)
  Q_PROPERTY(QVariant dout READ getDout NOTIFY statusChanged)
  Q_PROPERTY(QVariant din READ getDin NOTIFY statusChanged)
  Q_PROPERTY(QVariant ain READ getAin NOTIFY statusChanged)
  Q_PROPERTY(QVariant aout READ getAout NOTIFY statusChanged)
  Q_PROPERTY(QVariant tioDout READ getTioDout NOTIFY statusChanged)
  Q_PROPERTY(QVariant tioDin READ getTioDin NOTIFY statusChanged)
  Q_PROPERTY(QVariant tioAin READ getTioAin NOTIFY statusChanged)
  Q_PROPERTY(QVariant tioKey READ getTioKey NOTIFY statusChanged)
  Q_PROPERTY(QVariant timestamp READ getTimestamp NOTIFY statusChanged)

  void setIp(const QString &ip); // NOLINT
  QString getIp() { return QString::fromStdString(ip_); }
  bool getConnected() const { return connected_; }
  bool getConnecting() const { return connecting_; }
  bool getEnabled() const { return enabled_; }
  bool getError() const { return errcode_ != 0; }
  bool getInpos() const { return inpos_; }
  double getRapidrate() const { return rapidrate_; }
  bool getProtectiveStop() const { return protective_stop_; }
  bool getEmergencyStop() const { return emergency_stop_; }
  bool getOnSoftLimit() const { return on_soft_limit_; }
  bool getDragStatus() const { return drag_status_; }

  // 新增的 getter 方法，全部替换为 Qt 类型
  int getErrcode() const { return static_cast<int>(errcode_); }
  bool getPoweredOn() const { return powered_on_; }
  QVariant getCartesianPosition() const {
    QVector<double> vec;
    for (const auto &val : cartesiantran_position_) {
      vec.append(val);
    }
    return QVariant::fromValue(vec);
  }
  QVariant getJointPosition() const {
    QVector<double> vec;
    for (const auto &val : joint_position_) {
      vec.append(val);
    }
    return QVariant::fromValue(vec);
  }
  uint getCurrentToolId() const { return current_tool_id_; }
  uint getCurrentUserId() const { return current_user_id_; }
  bool getIsSocketConnect() const { return is_socket_connect_; }
  QVariant getDout() const {
    QVector<int> vec;
    for (const auto &val : dout_) {
      vec.append(val);
    }
    return QVariant::fromValue(vec);
  }
  QVariant getDin() const {
    QVector<int> vec;
    for (const auto &val : din_) {
      vec.append(val);
    }
    return QVariant::fromValue(vec);
  }
  QVariant getAin() const {
    QVector<double> vec;
    for (const auto &val : ain_) {
      vec.append(val);
    }
    return QVariant::fromValue(vec);
  }
  QVariant getAout() const {
    QVector<double> vec;
    for (const auto &val : aout_) {
      vec.append(val);
    }
    return QVariant::fromValue(vec);
  }
  QVariant getTioDout() const {
    QVector<int> vec;
    for (const auto &val : tio_dout_) {
      vec.append(val);
    }
    return QVariant::fromValue(vec);
  }
  QVariant getTioDin() const {
    QVector<int> vec;
    for (const auto &val : tio_din_) {
      vec.append(val);
    }
    return QVariant::fromValue(vec);
  }
  QVariant getTioAin() const {
    QVector<double> vec;
    for (const auto &val : tio_ain_) {
      vec.append(val);
    }
    return QVariant::fromValue(vec);
  }
  QVariant getTioKey() const {
    QVector<int> vec;
    for (const auto &val : tio_key_) {
      vec.append(val);
    }
    return QVariant::fromValue(vec);
  }
  QVariant getTimestamp() const { return QVariant::fromValue(timestamp_); }

  Q_INVOKABLE bool connect();
  Q_INVOKABLE bool disconnect();
  Q_INVOKABLE void updateRobotConnectConfig();
  Q_INVOKABLE bool power(bool power);
  Q_INVOKABLE bool enable(bool enable);
  Q_INVOKABLE bool clearError();
  Q_INVOKABLE bool resetSensor();

  std::shared_ptr<rclcpp::Node> getNode() const { return node_; }

signals:
  void ipChanged();
  void statusChanged();

private:
  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr
      set_params_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr
      get_params_client_;
  rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedPtr
      describe_params_client_;
  rclcpp::Client<jaka_msgs::srv::RobotControl>::SharedPtr robot_control_client_;

  rclcpp::Subscription<jaka_msgs::msg::RobotStatus>::SharedPtr
      robot_status_sub_;

  bool connecting_{false};
  bool connected_{false};

  // RobotStatus 相关状态变量
  int32_t errcode_{0};
  bool inpos_{false};
  bool powered_on_{false};
  bool enabled_{false};
  double rapidrate_{0.0};
  bool protective_stop_{false};
  bool emergency_stop_{false};
  std::vector<double> cartesiantran_position_;
  std::vector<double> joint_position_;
  uint32_t current_tool_id_{0};
  uint32_t current_user_id_{0};
  bool on_soft_limit_{false};
  bool drag_status_{false};
  bool is_socket_connect_{false};
  std::vector<int32_t> dout_;
  std::vector<int32_t> din_;
  std::vector<double> ain_;
  std::vector<double> aout_;
  std::vector<int32_t> tio_dout_;
  std::vector<int32_t> tio_din_;
  std::vector<double> tio_ain_;
  std::vector<int32_t> tio_key_;
  builtin_interfaces::msg::Time timestamp_;
  std::string ip_{"192.168.1.122"};

  void delayInit();

  // 私有方法
  void setupClients();
  void setupSubscribers();

  bool setParameter(const std::string &name, const std::string &value);
  std::string getParameter(const std::string &name);
};
} // namespace auto_charge

#endif // ROBOT_MANAGER_H
