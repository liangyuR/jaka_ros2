#include "auto_charge/camera_controller.h"
#include <QDebug>
#include <QVariant>
#include <QVariantList>
#include <QVariantMap>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace auto_charge {

CameraController::CameraController(QObject *parent)
    : QObject(parent), isConnected_(false), isConnecting_(false),
      waitForConnectionTimeout_(30), runProjectTimeout_(30) {
  node_ = std::make_shared<rclcpp::Node>("camera_controller");
  // 加载配置文件
  const auto config_path = findConfigFile("alson_config.yaml");
  try {
    loadConfig(config_path);
    RCLCPP_INFO(node_->get_logger(), "Configuration loaded from: %s",
                config_path.toStdString().c_str());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load configuration: %s",
                 e.what());
    RCLCPP_ERROR(node_->get_logger(), "Using default configuration values");
  }

  // 初始化相机参数更新服务客户端
  update_camera_para_client_ =
      node_->create_client<jaka_msgs::srv::UpdateCameraPara>(
          "alson_client_node/update_camera_para");

  // 初始化连接控制服务客户端
  connect_client_ =
      node_->create_client<std_srvs::srv::Trigger>("alson_client_node/connect");
  disconnect_client_ = node_->create_client<std_srvs::srv::Trigger>(
      "alson_client_node/disconnect");
  restart_client_ =
      node_->create_client<std_srvs::srv::Trigger>("alson_client_node/restart");
  run_project_client_ = node_->create_client<jaka_msgs::srv::RunProject>(
      "alson_client_node/run_project");

  if (node_ == nullptr) {
    RCLCPP_ERROR(node_->get_logger(), "CameraController: Invalid node pointer");
    return;
  }

  // 初始化状态监听
  initializeStatusMonitoring();

  RCLCPP_INFO(node_->get_logger(), "CameraController initialized with node: %s",
              node_->get_name());
}

CameraController::~CameraController() {
  RCLCPP_INFO(node_->get_logger(), "CameraController destroyed");
}

void CameraController::DisconnectCamera() {
  RCLCPP_INFO(node_->get_logger(), "DisconnectCamera called");

  if (!node_) {
    RCLCPP_ERROR(node_->get_logger(), "DisconnectCamera: Node not available");
    return;
  }

  if (!disconnect_client_->service_is_ready()) {
    RCLCPP_ERROR(node_->get_logger(), "DisconnectCamera: Service not ready");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  // 异步调用服务
  auto future = disconnect_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        handleDisconnectResponse(future);
      });
}

void CameraController::ConnectCamera() {
  RCLCPP_INFO(node_->get_logger(), "ConnectCamera called");

  if (!node_) {
    RCLCPP_ERROR(node_->get_logger(), "ConnectCamera: Node not available");
    return;
  }

  if (!connect_client_->service_is_ready()) {
    RCLCPP_ERROR(node_->get_logger(), "ConnectCamera: Service not ready");
    return;
  }

  // 设置连接中状态
  setConnecting(true);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  // 异步调用服务
  auto future = connect_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        handleConnectResponse(future);
      });
}

void CameraController::UpdateCameraParam() {
  RCLCPP_INFO(node_->get_logger(), "UpdateCameraParam called");

  if (node_ == nullptr) {
    RCLCPP_ERROR(node_->get_logger(), "UpdateCameraParam: Node not available");
    return;
  }

  if (!update_camera_para_client_->service_is_ready()) {
    RCLCPP_ERROR(node_->get_logger(), "UpdateCameraParam: Service not ready");
    return;
  }

  auto request = std::make_shared<jaka_msgs::srv::UpdateCameraPara::Request>();
  request->ip = currentHost_.toStdString();
  request->port = currentPort_;
  request->charge_station_id = chargeStationId_.toStdString();
  request->connector_id = connectorId_.toStdString();
  request->placement_id = placementId_.toStdString();
  request->charge_box_id = chargeBoxId_.toStdString();

  // 异步调用服务
  auto future = update_camera_para_client_->async_send_request(
      request,
      [this](rclcpp::Client<jaka_msgs::srv::UpdateCameraPara>::SharedFuture
                 future) { handleUpdateCameraResponse(future); });
}

void CameraController::loadConfig(const QString &config_path) {
  RCLCPP_INFO(node_->get_logger(), "loadConfig: %s",
              config_path.toStdString().c_str());
  try {
    YAML::Node config = YAML::LoadFile(config_path.toStdString());
    if (!config["alson_client"]) {
      RCLCPP_WARN(node_->get_logger(), "Missing 'alson_client' in config");
      return;
    }
    auto alson = config["alson_client"];
    // clang-format off
    if (alson["host"]) setCurrentHost(QString::fromStdString(alson["host"].as<std::string>()));
    if (alson["port"]) setCurrentPort(alson["port"].as<int>());
    if (alson["wait_for_connection"]) setWaitForConnectionTimeout(alson["wait_for_connection"].as<int>());
    if (alson["run_project_timeout"]) setRunProjectTimeout(alson["run_project_timeout"].as<int>());
    if (alson["charge_station"]) setChargeStationId(QString::fromStdString(alson["charge_station"].as<std::string>()));
    if (alson["charge_box"]) setChargeBoxId(QString::fromStdString(alson["charge_box"].as<std::string>()));
    if (alson["connector"]) setConnectorId(QString::fromStdString(alson["connector"].as<std::string>()));
    if (alson["placement"]) setPlacementId(QString::fromStdString(alson["placement"].as<std::string>()));
    // clang-format on
    RCLCPP_INFO(node_->get_logger(), "Config loaded from: %s",
                config_path.toStdString().c_str());
  } catch (const std::exception &e) {
    RCLCPP_WARN(node_->get_logger(), "Failed to load config: %s", e.what());
  }
}

QString CameraController::findConfigFile(const QString &filename) {
  try {
    // 使用 ROS2 包路径查找器
    QString pkg_share_dir = QString::fromStdString(
        ament_index_cpp::get_package_share_directory("auto_charge"));
    QString config_path = pkg_share_dir + "/config/" + filename;

    // 检查文件是否存在
    std::ifstream file(config_path.toStdString());
    if (file.good()) {
      file.close();
      RCLCPP_INFO(node_->get_logger(), "Found config file at: %s",
                  config_path.toStdString().c_str());
      return config_path;
    }
    // 文件不存在，抛出异常
    QString error_msg = "Config file not found: " + config_path;
    RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.toStdString().c_str());
    throw std::runtime_error(error_msg.toStdString());

  } catch (const std::exception &e) {
    QString error_msg =
        "Failed to find config file '" + filename + "': " + e.what();
    RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.toStdString().c_str());
    throw std::runtime_error(error_msg.toStdString());
  }
}

void CameraController::saveConfig() {
  RCLCPP_DEBUG(node_->get_logger(), "saveConfig called");

  try {
    // 创建 YAML 节点
    YAML::Node config;
    YAML::Node alson_client;

    // 设置网络配置
    alson_client["host"] = currentHost_.toStdString();
    alson_client["port"] = currentPort_;

    // 设置超时配置
    alson_client["wait_for_connection"] = waitForConnectionTimeout_;
    alson_client["run_project_timeout"] = runProjectTimeout_;

    // 设置充电站配置
    alson_client["charge_station"] = chargeStationId_.toStdString();
    alson_client["charge_box"] = chargeBoxId_.toStdString();
    alson_client["connector"] = connectorId_.toStdString();
    alson_client["placement"] = placementId_.toStdString();

    // 将 alson_client 节点添加到主配置节点
    config["alson_client"] = alson_client;

    // 查找配置文件路径
    const auto config_path = findConfigFile("alson_config.yaml");
    std::ofstream file(config_path.toStdString());
    if (file.is_open()) {
      file << config;
      file.close();
      RCLCPP_INFO(node_->get_logger(),
                  "Configuration saved successfully to: %s",
                  config_path.toStdString().c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "Failed to open config file for writing: %s",
                  config_path.toStdString().c_str());
    }

  } catch (const YAML::Exception &e) {
    RCLCPP_WARN(node_->get_logger(), "Failed to create YAML config: %s",
                e.what());
  } catch (const std::exception &e) {
    RCLCPP_WARN(node_->get_logger(), "Failed to save config file: %s",
                e.what());
  }
}

void CameraController::handleUpdateCameraResponse(
    const rclcpp::Client<jaka_msgs::srv::UpdateCameraPara>::SharedFuture
        future) {
  try {
    auto response = future.get();

    QVariantMap resultData;
    resultData["success"] = response->success;
    resultData["message"] = QString::fromStdString(response->message);

    RCLCPP_INFO(node_->get_logger(),
                "UpdateCameraParam result: success=%d, message=%s",
                response->success, response->message.c_str());
  } catch (const std::exception &e) {
    RCLCPP_WARN(node_->get_logger(),
                "UpdateCameraParam service call failed: %s", e.what());
  }
}
void CameraController::RunProject(const QString &project_id,
                                  const QVector<float> &position) {
  if (node_ == nullptr) {
    RCLCPP_WARN(rclcpp::get_logger("camera_controller"),
                "RunProject: Node not available");
    return;
  }
  if (project_id.isEmpty() || position.size() != 6) {
    RCLCPP_WARN(rclcpp::get_logger("camera_controller"),
                "RunProject: Invalid parameters");
    return;
  }

  auto request = std::make_shared<jaka_msgs::srv::RunProject::Request>();
  request->project_id = project_id.toStdString();
  request->fl_tcp_position.assign(position.begin(), position.end());
  run_project_client_->async_send_request(
      request,
      [this](rclcpp::Client<jaka_msgs::srv::RunProject>::SharedFuture future) {
        try {
          auto response = future.get();
          RCLCPP_INFO(node_->get_logger(),
                      "RunProject: message=%s, status_code=%d",
                      response->message.c_str(), response->status_code);
        } catch (const std::exception &e) {
          RCLCPP_ERROR(rclcpp::get_logger("camera_controller"),
                       "RunProject service call failed: %s", e.what());
        }
      });
}

void CameraController::handleConnectResponse(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
  try {
    auto response = future.get();
    RCLCPP_INFO(node_->get_logger(), "ConnectCamera response: %s",
                response->success ? "success" : "failed");
    setConnecting(false);
    setConnected(response->success);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "ConnectCamera failed: %s", e.what());
    setConnecting(false);
    setConnected(false);
  }
}

void CameraController::handleDisconnectResponse(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
  try {
    RCLCPP_INFO(node_->get_logger(), "DisconnectCamera response: %s",
                future.get()->success ? "success" : "failed");

    setConnected(false);
    setConnecting(false);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "DisconnectCamera failed: %s", e.what());
    setConnected(false);
    setConnecting(false);
  }
}

void CameraController::RestartCamera() {
  qDebug() << "RestartCamera called";

  if (node_ == nullptr) {
    RCLCPP_ERROR(node_->get_logger(), "RestartCamera: Node not available");
    return;
  }

  if (!restart_client_->service_is_ready()) {
    RCLCPP_ERROR(node_->get_logger(), "RestartCamera: Service not ready");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  // 异步调用服务
  auto future = restart_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        handleRestartResponse(future);
      });
}

void CameraController::initializeStatusMonitoring() {
  if (node_ == nullptr) {
    RCLCPP_ERROR(node_->get_logger(),
                 "initializeStatusMonitoring: Node not available");
    return;
  }

  // 订阅相机状态消息
  status_subscription_ = node_->create_subscription<std_msgs::msg::String>(
      "alson_events", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        handleStatusMessage(msg);
      });

  // 订阅 AlsonEvent 消息
  alson_event_subscription_ =
      node_->create_subscription<jaka_msgs::msg::AlsonEvent>(
          "alson_event", 10,
          [this](const jaka_msgs::msg::AlsonEvent::SharedPtr msg) {
            handleAlsonEvent(msg);
          });

  // 创建定时器，定期检查相机状态
  // status_check_timer_ =
  //     node_->create_wall_timer(std::chrono::seconds(2), // 每2秒检查一次
  //                              [this]() { CheckCameraStatus(); });

  qDebug() << "Status monitoring initialized";
}

void CameraController::handleStatusMessage(
    const std_msgs::msg::String::SharedPtr msg) {
  try {
    // 解析JSON消息
    nlohmann::json json = nlohmann::json::parse(msg->data);

    if (json["event_type"] == "connection_status") {
      bool connected = json["connected"].get<bool>();
      QString message =
          QString::fromStdString(json["message"].get<std::string>());

      RCLCPP_INFO(node_->get_logger(), "Camera connection status: %s",
                  connected ? "connected" : "disconnected");

      if (connected) {
        setConnected(true);
      } else {
        setConnected(false);
      }
    }
  } catch (const std::exception &e) {
    qWarning() << "Failed to parse status message:" << e.what();
  }
}

void CameraController::handleAlsonEvent(
    const jaka_msgs::msg::AlsonEvent::SharedPtr msg) {
  try {
    qDebug() << "Received AlsonEvent: type=" << msg->event_type
             << "status=" << msg->status
             << "message=" << QString::fromStdString(msg->message);

    // 根据事件类型处理不同的消息
    switch (msg->event_type) {
    case jaka_msgs::msg::AlsonEvent::EVENT_TYPE_CONNECTION_STATUS:
      handleConnectionStatusEvent(msg);
      break;
    case jaka_msgs::msg::AlsonEvent::EVENT_TYPE_DATA_RECEIVED:
      handleDataReceivedEvent(msg);
      break;
    default:
      qWarning() << "Unknown event type:" << msg->event_type;
      break;
    }
  } catch (const std::exception &e) {
    qWarning() << "Failed to handle AlsonEvent:" << e.what();
  }
}

// TODO(@liangyu) 定时检查相机是否连接正常
void CameraController::checkCameraStatus() {
  // 这里可以主动查询相机状态
  // 比如通过调用一个查询服务来获取当前连接状态
  // 目前我们主要依赖alson_events topic的消息
  // 如果需要主动查询，可以在这里实现
}

void CameraController::handleConnectionStatusEvent(
    const jaka_msgs::msg::AlsonEvent::SharedPtr msg) {
  RCLCPP_INFO(node_->get_logger(), "Connection status event: %s",
              msg->connected ? "connected" : "disconnected");
  setConnected(msg->connected);
  setConnecting(false);
}

void CameraController::handleDataReceivedEvent(
    const jaka_msgs::msg::AlsonEvent::SharedPtr msg) {
  RCLCPP_INFO(node_->get_logger(), "Data received event: %s",
              msg->data.c_str());
}

void CameraController::handleRestartResponse(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
  try {
    RCLCPP_INFO(node_->get_logger(), "RestartCamera response: %s",
                future.get()->success ? "success" : "failed");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "RestartCamera failed: %s", e.what());
  }
}

// Setter 方法实现
void CameraController::setCurrentHost(const QString &host) {
  if (currentHost_ != host) {
    currentHost_ = host;
    emit currentHostChanged();
  }
}

void CameraController::setCurrentPort(int port) {
  if (currentPort_ != port) {
    currentPort_ = port;
    emit currentPortChanged();
  }
}

void CameraController::setChargeStationId(const QString &id) {
  if (chargeStationId_ != id) {
    chargeStationId_ = id;
    emit chargeStationIdChanged();
  }
}

void CameraController::setConnectorId(const QString &id) {
  if (connectorId_ != id) {
    connectorId_ = id;
    emit connectorIdChanged();
  }
}

void CameraController::setPlacementId(const QString &id) {
  if (placementId_ != id) {
    placementId_ = id;
    emit placementIdChanged();
  }
}

void CameraController::setChargeBoxId(const QString &id) {
  if (chargeBoxId_ != id) {
    chargeBoxId_ = id;
    emit chargeBoxIdChanged();
  }
}

void CameraController::setWaitForConnectionTimeout(int timeout) {
  if (waitForConnectionTimeout_ != timeout) {
    waitForConnectionTimeout_ = timeout;
    emit waitForConnectionTimeoutChanged();
  }
}

void CameraController::setRunProjectTimeout(int timeout) {
  if (runProjectTimeout_ != timeout) {
    runProjectTimeout_ = timeout;
    emit runProjectTimeoutChanged();
  }
}

void CameraController::setConnected(bool connected) {
  if (isConnected_ != connected) {
    isConnected_ = connected;
    emit isConnectedChanged();
  }
}

void CameraController::setConnecting(bool connecting) {
  if (isConnecting_ != connecting) {
    isConnecting_ = connecting;
    emit isConnectingChanged();
  }
}

} // namespace auto_charge