#ifndef CAMERA_CONTROLLER_H
#define CAMERA_CONTROLLER_H

#include <QObject>
#include <QVariant>
#include <QVariantMap>
#include <jaka_msgs/msg/alson_event.hpp>
#include <jaka_msgs/srv/update_camera_para.hpp> // namespace rclcpp
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp> // namespace rclcpp
#include <std_srvs/srv/trigger.hpp>

namespace auto_charge {

using UpdateCameraPara = jaka_msgs::srv::UpdateCameraPara;

class CameraController : public QObject {
  Q_OBJECT

  // 暴露给QML的属性
  Q_PROPERTY(bool isConnected READ isConnected NOTIFY isConnectedChanged)
  Q_PROPERTY(bool isConnecting READ isConnecting NOTIFY isConnectingChanged)
  Q_PROPERTY(QString currentHost READ currentHost WRITE setCurrentHost NOTIFY
                 currentHostChanged)
  Q_PROPERTY(int currentPort READ currentPort WRITE setCurrentPort NOTIFY
                 currentPortChanged)
  Q_PROPERTY(QString chargeStationId READ chargeStationId WRITE
                 setChargeStationId NOTIFY chargeStationIdChanged)
  Q_PROPERTY(QString connectorId READ connectorId WRITE setConnectorId NOTIFY
                 connectorIdChanged)
  Q_PROPERTY(QString placementId READ placementId WRITE setPlacementId NOTIFY
                 placementIdChanged)
  Q_PROPERTY(QString chargeBoxId READ chargeBoxId WRITE setChargeBoxId NOTIFY
                 chargeBoxIdChanged)
  Q_PROPERTY(
      int waitForConnectionTimeout READ waitForConnectionTimeout WRITE
          setWaitForConnectionTimeout NOTIFY waitForConnectionTimeoutChanged)
  Q_PROPERTY(int runProjectTimeout READ runProjectTimeout WRITE
                 setRunProjectTimeout NOTIFY runProjectTimeoutChanged)

public:
  explicit CameraController(rclcpp::Node *node, QObject *parent = nullptr);
  ~CameraController() override;

  // QML可调用的方法
  Q_INVOKABLE void DisconnectCamera();
  Q_INVOKABLE void ConnectCamera();
  Q_INVOKABLE void RestartCamera();
  Q_INVOKABLE void UpdateCameraParam();
  Q_INVOKABLE void loadConfig(const std::string &config_path);
  Q_INVOKABLE void saveConfig();

  // 属性访问方法
  bool isConnected() const { return isConnected_; }
  bool isConnecting() const { return isConnecting_; }
  QString currentHost() const { return currentHost_; }
  int currentPort() const { return currentPort_; }
  QString chargeStationId() const { return chargeStationId_; }
  QString connectorId() const { return connectorId_; }
  QString placementId() const { return placementId_; }
  QString chargeBoxId() const { return chargeBoxId_; }
  int waitForConnectionTimeout() const { return waitForConnectionTimeout_; }
  int runProjectTimeout() const { return runProjectTimeout_; }

  void setCurrentHost(const QString &host);
  void setCurrentPort(int port);
  void setChargeStationId(const QString &id); // NOLINT
  void setConnectorId(const QString &id);     // NOLINT
  void setPlacementId(const QString &id);     // NOLINT
  void setChargeBoxId(const QString &id);     // NOLINT
  void setWaitForConnectionTimeout(int timeout);
  void setRunProjectTimeout(int timeout);

  // 配置路径查找
  std::string findConfigFile(const std::string &filename);

  // 连接状态管理
  void setConnected(bool connected);
  void setConnecting(bool connecting);

  // ID: 视觉项目名称, position: 机械臂法兰末端位姿, 相对于机械臂 base
  Q_INVOKABLE void RunProject(const std::string &project_id,
                              const std::vector<float> &position);

signals:
  void isConnectedChanged();
  void isConnectingChanged();
  void currentHostChanged();
  void currentPortChanged();
  void chargeStationIdChanged();
  void connectorIdChanged();
  void placementIdChanged();
  void chargeBoxIdChanged();
  void waitForConnectionTimeoutChanged();
  void runProjectTimeoutChanged();

private:
  // 使用auto_charge节点的ROS2客户端
  rclcpp::Node *node_;
  std::shared_ptr<rclcpp::Client<jaka_msgs::srv::UpdateCameraPara>>
      update_camera_para_client_;
  std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> connect_client_;
  std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> disconnect_client_;
  std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> restart_client_;

  // 状态监听
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::Subscription<jaka_msgs::msg::AlsonEvent>::SharedPtr
      alson_event_subscription_;
  rclcpp::TimerBase::SharedPtr status_check_timer_;

  // 私有方法
  void handleUpdateCameraResponse(
      const rclcpp::Client<jaka_msgs::srv::UpdateCameraPara>::SharedFuture
          future);
  void handleConnectResponse(
      const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
  void handleDisconnectResponse(
      const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
  void handleRestartResponse(
      const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

  // 状态监听方法
  void handleStatusMessage(const std_msgs::msg::String::SharedPtr msg);
  void handleAlsonEvent(const jaka_msgs::msg::AlsonEvent::SharedPtr msg);
  void
  handleConnectionStatusEvent(const jaka_msgs::msg::AlsonEvent::SharedPtr msg);
  void handleDataReceivedEvent(const jaka_msgs::msg::AlsonEvent::SharedPtr msg);

  void checkCameraStatus();
  void initializeStatusMonitoring();

  bool isConnected_;
  bool isConnecting_;
  QString currentHost_;
  int currentPort_;
  QString chargeStationId_;
  QString connectorId_;
  QString placementId_;
  QString chargeBoxId_;

  // 配置相关成员变量
  int waitForConnectionTimeout_;
  int runProjectTimeout_;
};

} // namespace auto_charge

#endif // CAMERA_CONTROLLER_H