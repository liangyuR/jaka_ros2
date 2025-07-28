#include "alson_client/alson_client_node.h"
#include <nlohmann/json.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace alson_client {

AlsonClientNode::AlsonClientNode(const rclcpp::NodeOptions &options)
    : Node("alson_client_node", options) {
  alson_events_publisher_ =
      this->create_publisher<std_msgs::msg::String>("alson_events", 10);
  alson_event_publisher_ =
      this->create_publisher<jaka_msgs::msg::AlsonEvent>("alson_event", 10);

  // 注册 runproj service
  runproj_service_ = this->create_service<jaka_msgs::srv::RunProject>(
      "~/run_project",
      [this](
          const std::shared_ptr<jaka_msgs::srv::RunProject::Request> &request,
          std::shared_ptr<jaka_msgs::srv::RunProject::Response> response) {
        this->handleRunProjService(request, response);
      });

  // 注册连接控制服务
  connect_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/connect",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> &request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        this->handleConnectService(request, response);
      });

  disconnect_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/disconnect",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> &request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        this->handleDisconnectService(request, response);
      });

  restart_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/restart",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> &request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        this->handleRestartService(request, response);
      });

  get_status_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/get_status",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> &request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        this->handleGetStatusService(request, response);
      });
  // 初始化 alson client
  const auto host =
      this->declare_parameter<std::string>("host", "192.168.0.188");
  const auto port = this->declare_parameter<int>("port", 54600);

  client_ = new AlsonClient(host, port, true); // 自动连接
  client_->setEventCallback([this](const AlsonClient::EventData &event) {
    RCLCPP_INFO(this->get_logger(), "Alson event: %s", event.message.c_str());

    // 将 EventData 转换为 AlsonEvent 消息并发布
    auto msg = jaka_msgs::msg::AlsonEvent();
    msg.header.stamp = this->now();

    switch (event.type) {
    case AlsonClient::EventType::CONNECTION_STATUS:
      msg.event_type = jaka_msgs::msg::AlsonEvent::EVENT_TYPE_CONNECTION_STATUS;
      msg.connected = event.success;
      break;
    case AlsonClient::EventType::DATA_RECEIVED:
      msg.event_type = jaka_msgs::msg::AlsonEvent::EVENT_TYPE_DATA_RECEIVED;
      msg.data = event.data;
      break;
    case AlsonClient::EventType::RECONNECT_STATUS:
      msg.event_type = jaka_msgs::msg::AlsonEvent::EVENT_TYPE_RECONNECT_STATUS;
      msg.retry_count = event.retry_count;
      msg.delay_seconds = event.delay;
      break;
    case AlsonClient::EventType::HEARTBEAT_STATUS:
      msg.event_type = jaka_msgs::msg::AlsonEvent::EVENT_TYPE_HEARTBEAT_STATUS;
      break;
    case AlsonClient::EventType::PROJECT_STATUS:
      msg.event_type = jaka_msgs::msg::AlsonEvent::EVENT_TYPE_PROJECT_STATUS;
      break;
    }

    msg.status = event.success ? jaka_msgs::msg::AlsonEvent::STATUS_SUCCESS
                               : jaka_msgs::msg::AlsonEvent::STATUS_FAILED;
    msg.message = event.message;

    alson_event_publisher_->publish(msg);
  });
  RCLCPP_INFO(this->get_logger(), "AlsonClient initialized");
}

AlsonClientNode::~AlsonClientNode() {
  // 等待重启线程完成
  if (restarting_.load()) {
    if (restart_thread_.joinable()) {
      restart_thread_.join();
    }
  }

  // 删除客户端实例
  if (client_ != nullptr) {
    delete client_;
    client_ = nullptr;
  }
}

void AlsonClientNode::handleRunProjService(
    const std::shared_ptr<jaka_msgs::srv::RunProject::Request> &request,
    std::shared_ptr<jaka_msgs::srv::RunProject::Response> &response) {
  if (!validateRequest(request, response)) {
    return;
  }

  std::vector<float> pose;
  std::string message;
  auto success = executeProject(request->project_id, request->fl_tcp_position,
                                &pose, &message);
  fillResponse(response, success, pose, message);

  auto json = nlohmann::json::object();
  json["project_id"] = request->project_id;
  json["status_code"] = response->status_code;
  json["success"] = success;
  json["pose"] = pose;
  json["message"] = message;
  PublishEvents(json.dump());
}

void AlsonClientNode::PublishEvents(const std::string &json) {
  auto msg = std_msgs::msg::String();
  msg.data = json;
  alson_events_publisher_->publish(msg);
  RCLCPP_DEBUG(this->get_logger(), "Published response: %s", json.c_str());
}

bool AlsonClientNode::validateRequest(
    const std::shared_ptr<jaka_msgs::srv::RunProject::Request> &request,
    std::shared_ptr<jaka_msgs::srv::RunProject::Response> &response) {
  if (request->project_id.empty() || request->fl_tcp_position.size() != 6) {
    response->status_code = -1;
    response->message = "Project ID is empty or fl_tcp_position is not 6D";
    return false;
  }
  return true;
}

void AlsonClientNode::fillResponse(
    std::shared_ptr<jaka_msgs::srv::RunProject::Response> &response,
    bool success, const std::vector<float> &pose, const std::string &message) {
  response->pose = pose;
  response->message = message;
  response->status_code = success ? 0 : -1;
}

bool AlsonClientNode::executeProject(const std::string &project_id,
                                     const std::vector<float> &fl_tcp_position,
                                     std::vector<float> *pose,
                                     std::string *message) {
  std::lock_guard<std::mutex> lock(client_mutex_);

  RCLCPP_INFO(this->get_logger(), "Change project to %s", project_id.c_str());
  if (!client_->ChangeProject(project_id)) {
    *message = "Project change failed";
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Run project");
  bool success = client_->RunProject(fl_tcp_position, pose);
  if (!success) {
    *message = "Project execution failed";
    return false;
  }
  *message = "Project executed successfully";
  return true;
}

void AlsonClientNode::handleConnectService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> &request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> &response) {
  RCLCPP_INFO(this->get_logger(), "Connecting to camera...");
  (void)request;

  std::lock_guard<std::mutex> lock(client_mutex_);
  bool success = client_->Connect();

  response->success = success;
  response->message =
      success ? "Camera connected successfully" : "Failed to connect to camera";

  // 发布连接状态事件
  PublishAlsonEvent(jaka_msgs::msg::AlsonEvent::EVENT_TYPE_CONNECTION_STATUS,
                    success ? jaka_msgs::msg::AlsonEvent::STATUS_SUCCESS
                            : jaka_msgs::msg::AlsonEvent::STATUS_FAILED,
                    response->message, success);

  RCLCPP_INFO(this->get_logger(), "Connect result: %s",
              response->message.c_str());
}

void AlsonClientNode::handleDisconnectService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> &request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> &response) {
  RCLCPP_INFO(this->get_logger(), "Disconnecting from camera...");
  (void)request;

  std::lock_guard<std::mutex> lock(client_mutex_);
  client_->Disconnect();

  response->success = true;
  response->message = "Camera disconnected successfully";

  // 发布连接状态事件
  PublishAlsonEvent(jaka_msgs::msg::AlsonEvent::EVENT_TYPE_CONNECTION_STATUS,
                    jaka_msgs::msg::AlsonEvent::STATUS_SUCCESS,
                    response->message,
                    false // disconnected
  );

  RCLCPP_INFO(this->get_logger(), "Disconnect result: %s",
              response->message.c_str());
}

void AlsonClientNode::handleRestartService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> &request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> &response) {
  RCLCPP_INFO(this->get_logger(), "Restarting alson client...");
  (void)request;

  // 检查是否已经在重启中
  if (restarting_.load()) {
    response->success = false;
    response->message = "Already restarting, please wait";
    RCLCPP_WARN(this->get_logger(), "Restart already in progress");
    return;
  }

  // 立即返回成功，实际重启在后台进行
  response->success = true;
  response->message = "Restart initiated, will complete in 5 seconds";

  // 发布重启开始事件
  PublishAlsonEvent(jaka_msgs::msg::AlsonEvent::EVENT_TYPE_CONNECTION_STATUS,
                    jaka_msgs::msg::AlsonEvent::STATUS_WAITING,
                    "Restart initiated", false);

  // 在后台线程中执行重启
  performRestart();

  RCLCPP_INFO(this->get_logger(), "Restart initiated");
}

void AlsonClientNode::PublishAlsonEvent(uint8_t event_type, uint8_t status,
                                        const std::string &message,
                                        bool connected, const std::string &data,
                                        int retry_count, int delay_seconds) {
  auto msg = jaka_msgs::msg::AlsonEvent();
  msg.header.stamp = this->now();
  msg.event_type = event_type;
  msg.status = status;
  msg.message = message;
  msg.data = data;
  msg.retry_count = retry_count;
  msg.delay_seconds = delay_seconds;
  msg.connected = connected;

  alson_event_publisher_->publish(msg);
  RCLCPP_DEBUG(this->get_logger(),
               "Published AlsonEvent: type=%d, status=%d, message=%s",
               event_type, status, message.c_str());
}

void AlsonClientNode::performRestart() {
  // 设置重启标志
  restarting_ = true;

  // 启动重启线程
  restart_thread_ = std::thread([this]() {
    RCLCPP_INFO(this->get_logger(), "=== Starting alson client restart ===");

    // 1. 断开当前连接
    {
      std::lock_guard<std::mutex> lock(client_mutex_);
      if (client_ != nullptr) {
        RCLCPP_INFO(this->get_logger(), "Disconnecting current client...");
        client_->Disconnect();
      }
    }

    // 发布断开事件
    PublishAlsonEvent(jaka_msgs::msg::AlsonEvent::EVENT_TYPE_CONNECTION_STATUS,
                      jaka_msgs::msg::AlsonEvent::STATUS_SUCCESS,
                      "Client disconnected for restart", false);

    // 2. 等待5秒
    RCLCPP_INFO(this->get_logger(), "Waiting 5 seconds before restart...");
    for (int i = 5; i > 0; --i) {
      PublishAlsonEvent(jaka_msgs::msg::AlsonEvent::EVENT_TYPE_RECONNECT_STATUS,
                        jaka_msgs::msg::AlsonEvent::STATUS_WAITING,
                        "Restarting in " + std::to_string(i) + " seconds",
                        false, "", 0, i);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 3. 重新初始化客户端
    {
      std::lock_guard<std::mutex> lock(client_mutex_);
      RCLCPP_INFO(this->get_logger(), "Reinitializing alson client...");

      // 重新初始化客户端 - 使用get_parameter而不是declare_parameter
      std::string host;
      int port;
      this->get_parameter("host", host);
      this->get_parameter("port", port);

      // 删除旧的客户端实例
      if (client_ != nullptr) {
        delete client_;
        client_ = nullptr;
      }

      // 创建新的客户端实例
      client_ = new AlsonClient(host, port, false); // 不自动连接
      client_->setEventCallback([this](const AlsonClient::EventData &event) {
        RCLCPP_INFO(this->get_logger(), "Alson event: %s",
                    event.message.c_str());

        // 将 EventData 转换为 AlsonEvent 消息并发布
        auto msg = jaka_msgs::msg::AlsonEvent();
        msg.header.stamp = this->now();

        switch (event.type) {
        case AlsonClient::EventType::CONNECTION_STATUS:
          msg.event_type =
              jaka_msgs::msg::AlsonEvent::EVENT_TYPE_CONNECTION_STATUS;
          msg.connected = event.success;
          break;
        case AlsonClient::EventType::DATA_RECEIVED:
          msg.event_type = jaka_msgs::msg::AlsonEvent::EVENT_TYPE_DATA_RECEIVED;
          msg.data = event.data;
          break;
        case AlsonClient::EventType::RECONNECT_STATUS:
          msg.event_type =
              jaka_msgs::msg::AlsonEvent::EVENT_TYPE_RECONNECT_STATUS;
          msg.retry_count = event.retry_count;
          msg.delay_seconds = event.delay;
          break;
        case AlsonClient::EventType::HEARTBEAT_STATUS:
          msg.event_type =
              jaka_msgs::msg::AlsonEvent::EVENT_TYPE_HEARTBEAT_STATUS;
          break;
        case AlsonClient::EventType::PROJECT_STATUS:
          msg.event_type =
              jaka_msgs::msg::AlsonEvent::EVENT_TYPE_PROJECT_STATUS;
          break;
        }

        msg.status = event.success ? jaka_msgs::msg::AlsonEvent::STATUS_SUCCESS
                                   : jaka_msgs::msg::AlsonEvent::STATUS_FAILED;
        msg.message = event.message;

        alson_event_publisher_->publish(msg);
      });
    }

    // 4. 手动连接
    RCLCPP_INFO(this->get_logger(), "Manually connecting after restart...");
    client_->Connect();

    // 5. 发布重启完成事件
    PublishAlsonEvent(jaka_msgs::msg::AlsonEvent::EVENT_TYPE_CONNECTION_STATUS,
                      jaka_msgs::msg::AlsonEvent::STATUS_SUCCESS,
                      "Alson client restart completed", true);

    RCLCPP_INFO(this->get_logger(), "=== Alson client restart completed ===");

    // 6. 清除重启标志
    restarting_ = false;
  });

  // 分离线程，不等待其完成
  restart_thread_.detach();
}

void AlsonClientNode::handleGetStatusService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> &request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> &response) {
  RCLCPP_INFO(this->get_logger(), "Getting alson client status...");
  (void)request;

  std::lock_guard<std::mutex> lock(client_mutex_);

  if (client_) {
    // 获取当前连接状态
    bool isConnected = client_->IsConnected();

    response->success = true;
    response->message = isConnected ? "Connected" : "Disconnected";

    // 发布状态事件
    PublishAlsonEvent(jaka_msgs::msg::AlsonEvent::EVENT_TYPE_CONNECTION_STATUS,
                      isConnected ? jaka_msgs::msg::AlsonEvent::STATUS_SUCCESS
                                  : jaka_msgs::msg::AlsonEvent::STATUS_FAILED,
                      response->message, isConnected);

    RCLCPP_INFO(this->get_logger(), "Status result: %s",
                response->message.c_str());
  } else {
    response->success = false;
    response->message = "Client not initialized";
    RCLCPP_ERROR(this->get_logger(), "Status result: %s",
                 response->message.c_str());
  }
}
} // namespace alson_client
