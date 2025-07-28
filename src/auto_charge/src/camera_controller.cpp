#include "auto_charge/camera_controller.h"
#include <QDebug>
#include <QVariant>
#include <QVariantList>
#include <QVariantMap>
#include <fstream>
#include <nlohmann/json.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace auto_charge {

CameraController::CameraController(rclcpp::Node *node, QObject *parent)
    : QObject(parent), node_(node), isConnected_(false), isConnecting_(false),
      waitForConnectionTimeout_(30), runProjectTimeout_(30) {
  // 加载配置文件
  std::string config_path = findConfigFile("alson_config.yaml");
  try {
    loadConfig(config_path);
    RCLCPP_INFO(node_->get_logger(), "Configuration loaded from: %s",
                config_path.c_str());
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
  RCLCPP_INFO(node_->get_logger(), "ReconnectCamera called");

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

void CameraController::loadConfig(const std::string &config_path) {
  RCLCPP_INFO(node_->get_logger(), "loadConfig called with: %s",
              config_path.c_str());

  try {
    // 使用 yaml-cpp 解析配置文件
    YAML::Node config = YAML::LoadFile(config_path);

    if (!config["alson_client"]) {
      RCLCPP_WARN(node_->get_logger(),
                  "loadConfig: Missing 'alson_client' section in config file");
      return;
    }

    YAML::Node alson_config = config["alson_client"];

    // 读取网络配置
    if (alson_config["host"]) {
      QString host =
          QString::fromStdString(alson_config["host"].as<std::string>());
      setCurrentHost(host);
      RCLCPP_INFO(node_->get_logger(), "Loaded host: %s",
                  host.toStdString().c_str());
    }

    if (alson_config["port"]) {
      int port = alson_config["port"].as<int>();
      setCurrentPort(port);
      RCLCPP_INFO(node_->get_logger(), "Loaded port: %d", port);
    }

    // 读取超时配置
    if (alson_config["wait_for_connection"]) {
      waitForConnectionTimeout_ = alson_config["wait_for_connection"].as<int>();
      RCLCPP_INFO(node_->get_logger(), "Loaded wait_for_connection timeout: %d",
                  waitForConnectionTimeout_);
    }

    if (alson_config["run_project_timeout"]) {
      runProjectTimeout_ = alson_config["run_project_timeout"].as<int>();
      RCLCPP_INFO(node_->get_logger(), "Loaded run_project_timeout: %d",
                  runProjectTimeout_);
    }

    // 读取充电站配置
    if (alson_config["charge_station"]) {
      QString chargeStation = QString::fromStdString(
          alson_config["charge_station"].as<std::string>());
      setChargeStationId(chargeStation);
      RCLCPP_INFO(node_->get_logger(), "Loaded charge_station: %s",
                  chargeStation.toStdString().c_str());
    }

    if (alson_config["charge_box"]) {
      QString chargeBox =
          QString::fromStdString(alson_config["charge_box"].as<std::string>());
      setChargeBoxId(chargeBox);
      RCLCPP_INFO(node_->get_logger(), "Loaded charge_box: %s",
                  chargeBox.toStdString().c_str());
    }

    if (alson_config["connector"]) {
      QString connector =
          QString::fromStdString(alson_config["connector"].as<std::string>());
      setConnectorId(connector);
      RCLCPP_INFO(node_->get_logger(), "Loaded connector: %s",
                  connector.toStdString().c_str());
    }

    if (alson_config["placement"]) {
      QString placement =
          QString::fromStdString(alson_config["placement"].as<std::string>());
      setPlacementId(placement);
      RCLCPP_INFO(node_->get_logger(), "Loaded placement: %s",
                  placement.toStdString().c_str());
    }

    RCLCPP_INFO(node_->get_logger(),
                "Configuration loaded successfully from: %s",
                config_path.c_str());

  } catch (const YAML::Exception &e) {
    RCLCPP_WARN(node_->get_logger(), "Failed to parse YAML config file: %s",
                e.what());
  } catch (const std::exception &e) {
    RCLCPP_WARN(node_->get_logger(), "Failed to load config file: %s",
                e.what());
  }
}

std::string CameraController::findConfigFile(const std::string &filename) {
  std::vector<std::string> search_paths = {
      filename, // 当前目录
      "config/" + filename, "../config/" + filename, "../../config/" + filename,
      "../../../config/" + filename};

  for (const auto &path : search_paths) {
    std::ifstream file(path);
    if (file.good()) {
      file.close();
      return path;
    }
  }

  // 如果找不到文件，返回默认路径
  return "config/" + filename;
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
    std::string config_path = findConfigFile("alson_config.yaml");
    std::ofstream file(config_path);
    if (file.is_open()) {
      file << config;
      file.close();
      RCLCPP_INFO(node_->get_logger(),
                  "Configuration saved successfully to: %s",
                  config_path.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "Failed to open config file for writing: %s",
                  config_path.c_str());
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

void CameraController::RunProject(const std::string &project_id,
                                  const std::vector<float> &position) {
  std::ostringstream pos_stream;
  for (size_t i = 0; i < position.size(); ++i) {
    pos_stream << position[i];
    if (i != position.size() - 1) {
      pos_stream << ", ";
    }
  }
  RCLCPP_INFO(node_->get_logger(),
              "RunProject called with: project_id=%s, position=[%s]",
              project_id.c_str(), pos_stream.str().c_str());

  if (node_ == nullptr) {
    RCLCPP_WARN(rclcpp::get_logger("camera_controller"),
                "RunProject: Node not available");
    return;
  }

  // 检查参数完整性
  if (project_id.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("camera_controller"),
                "RunProject: Missing required parameters");
    return;
  }

  // 验证fl_tcp_position格式
  if (position.size() != 6) {
    RCLCPP_WARN(rclcpp::get_logger("camera_controller"),
                "RunProject: fl_tcp_position must be an array of 6 elements");
    return;
  }

  // TODO: 实现RunProject服务调用
  // 这里需要根据实际的RunProject.srv定义来实现
  // 示例代码：
  /*
  auto request = std::make_shared<jaka_msgs::srv::RunProject::Request>();
  request->project_id = project_id;

  // 转换fl_tcp_position
  request->fl_tcp_position = position;

  // 异步调用服务
  auto future = run_project_client_->async_send_request(request);

  std::thread([this, future]() {
    try {
      auto response = future.get();

      // 处理 response
      RCLCPP_INFO(node_->get_logger(), "RunProject result: success=%d,
  message=%s, status_code=%d", response->success, response->message.c_str(),
  response->status_code);

    } catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("camera_controller"), "RunProject service
  call failed: %s", e.what());
    }
  }).detach();
  */

  // 临时返回成功状态（实际实现时需要替换上面的TODO代码）
  RCLCPP_INFO(node_->get_logger(), "RunProject called successfully");
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

  ConnectCamera(); // TODO(@liangyu) for test
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