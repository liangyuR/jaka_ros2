#include "alson_client/alson_client.h"
#include <boost/asio.hpp>
#include <iostream>
#include <nlohmann/json.hpp> // Added for JSON handling
#include <rclcpp/rclcpp.hpp>
#include <sstream>

namespace alson_client {
#define LOGGER rclcpp::get_logger("alson_client::AlsonClient")

AlsonClient::AlsonClient(const std::string &host, int port,
                         bool auto_connect) // NOLINT
    : host_(host), port_(port) {
  if (auto_connect) {
    std::thread([this]() {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      this->Connect();
    }).detach();
  }
}

bool AlsonClient::Connect() {
  RCLCPP_INFO(LOGGER, "Connecting AlsonClient to %s:%d", host_.c_str(), port_);
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (connected_) {
    RCLCPP_INFO(LOGGER, "AlsonClient already connected");
    if (structured_event_callback_) {
      EventData event{
          EventType::CONNECTION_STATUS, true, "Already connected", "", 0, 0};
      structured_event_callback_(event);
    }
    return true;
  }

  try {
    socket_ = std::make_unique<boost::asio::ip::tcp::socket>(io_context_);
    boost::asio::ip::tcp::resolver resolver(io_context_);
    auto endpoints = resolver.resolve(host_, std::to_string(port_));
    boost::asio::connect(*socket_, endpoints);
    connected_ = true;
    running_ = true;
    recv_thread_ =
        std::make_unique<std::thread>(&AlsonClient::receiveLoop, this);
    // TODO(@liangyu) 暂时关闭心跳
    // startHeartbeatThread(); // 启动心跳线程
    if (structured_event_callback_) {
      EventData event{EventType::CONNECTION_STATUS,
                      true,
                      "Connected successfully",
                      "",
                      0,
                      0};
      structured_event_callback_(event);
    }
    return true;
  } catch (std::exception &e) {
    RCLCPP_ERROR(LOGGER, "Connect failed: %s", e.what());
    connected_ = false;
    if (structured_event_callback_) {
      EventData event{EventType::CONNECTION_STATUS,
                      false,
                      "Connect failed: " + std::string(e.what()),
                      "",
                      0,
                      0};
      structured_event_callback_(event);
    }
    return false;
  }
}

void AlsonClient::Disconnect() {
  RCLCPP_INFO(LOGGER, "Disconnecting AlsonClient");

  // 先设置运行标志为false，让接收线程有机会退出
  running_ = false;
  connected_ = false;

  // 给接收线程一点时间退出
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // 然后关闭 socket
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (socket_) {
      try {
        socket_->close(); // 关闭 socket
        RCLCPP_INFO(LOGGER, "Socket closed");
      } catch (...) {
        RCLCPP_WARN(LOGGER, "Error closing socket");
      }
      socket_.reset();
    }
  }

  // 等待接收线程结束
  if (recv_thread_ && recv_thread_->joinable()) {
    RCLCPP_INFO(LOGGER, "Waiting for receive thread to finish...");
    recv_thread_->join();
    RCLCPP_INFO(LOGGER, "Receive thread finished");
  }

  // 停止心跳线程
  {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    heartbeat_running_ = false;
    heartbeat_cv_.notify_all();
  }
  if (heartbeat_thread_ && heartbeat_thread_->joinable()) {
    heartbeat_thread_->join();
  }

  // Clear reconnecting state
  reconnecting_ = false;
  if (reconnect_thread_ && reconnect_thread_->joinable()) {
    reconnect_thread_->join();
  }

  connected_ = false;
  RCLCPP_INFO(LOGGER, "AlsonClient disconnected");
  if (structured_event_callback_) {
    EventData event{
        EventType::CONNECTION_STATUS, true, "Disconnected", "", 0, 0};
    structured_event_callback_(event);
  }
}

bool AlsonClient::send(const std::string &msg) { // NOLINT
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (!connected_ || !socket_) {
    RCLCPP_ERROR(LOGGER, "AlsonClient not connected, cannot send message");
    return false;
  }

  try {
    RCLCPP_DEBUG(LOGGER, "Send message: %s", msg.c_str());
    boost::asio::write(*socket_, boost::asio::buffer(msg));
  } catch (const std::exception &e) {
    connected_ = false;
    RCLCPP_ERROR(LOGGER, "Send message failed: %s", e.what());

    // Start async reconnect
    startAsyncReconnect();
    return false;
  }
  return true;
}

void AlsonClient::receiveLoop() {
  while (running_ && connected_) {
    try {
      // 检查是否应该退出
      if (!running_ || !connected_) {
        break;
      }

      // 使用带超时的读取方式
      std::array<char, 4096> data{};

      // 检查socket是否可用
      {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        if (!socket_ || !connected_) {
          break;
        }
      }

      // 使用select或poll来检查是否有数据可读
      boost::system::error_code ec;
      size_t len = 0;

      // 尝试读取数据，设置较短的超时
      {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        if (socket_ && connected_) {
          // 设置socket为非阻塞模式
          socket_->non_blocking(true);

          // 尝试读取数据
          len = socket_->read_some(boost::asio::buffer(data), ec);
        }
      }

      if (ec) {
        if (ec == boost::asio::error::would_block) {
          // 没有数据可读，短暂等待后继续
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          continue;
        } else {
          // 其他错误，退出循环
          RCLCPP_ERROR(LOGGER, "Socket read error: %s", ec.message().c_str());
          connected_ = false;
          if (structured_event_callback_) {
            EventData event{EventType::CONNECTION_STATUS,
                            false,
                            "Socket read error: " + ec.message(),
                            "",
                            0,
                            0};
            structured_event_callback_(event);
          }
          break;
        }
      }

      if (len == 0) {
        connected_ = false;
        if (structured_event_callback_) {
          EventData event{
              EventType::CONNECTION_STATUS, false, "Connection lost", "", 0, 0};
          structured_event_callback_(event);
        }
        break;
      }

      std::string response(data.data(), len);
      if (structured_event_callback_) {
        EventData event{
            EventType::DATA_RECEIVED, true, "Data received", response, 0, 0};
        structured_event_callback_(event);
      }
      parseResponse(response);
    } catch (std::exception &e) {
      RCLCPP_ERROR(LOGGER, "Receive error: %s", e.what());
      connected_ = false;
      if (structured_event_callback_) {
        EventData event{EventType::CONNECTION_STATUS,
                        false,
                        "Receive error: " + std::string(e.what()),
                        "",
                        0,
                        0};
        structured_event_callback_(event);
      }
      break;
    }
  }
  RCLCPP_INFO(LOGGER, "Receive loop exited");
}

// 辅助函数：等待特定响应码
bool AlsonClient::waitForResponse(const std::string &expect_code,
                                  int timeout_sec) {
  std::unique_lock<std::mutex> lock(response_mutex_);
  expected_code_ = expect_code;
  response_received_ = false;
  waiting_response_ = true;
  bool ok = response_cv_.wait_for(lock, std::chrono::seconds(timeout_sec),
                                  [this] { return response_received_.load(); });
  waiting_response_ = false;
  return ok && last_response_code_ == expect_code;
}

bool AlsonClient::RunProject(const std::vector<float> &fl_tcp_position,
                             std::vector<float> *target_pose) {
  RCLCPP_INFO(LOGGER, "=== RunProject 开始 ===");
  // 1. 发送运行命令
  if (!send(CommandType::RUN_PRJ)) {
    RCLCPP_ERROR(LOGGER, "发送运行项目命令失败");
    return false;
  }
  // 2. 等待 Alson 请求机器人坐标
  if (!waitForResponse(ResponseCode::ROBOT_COORD, 30)) {
    RCLCPP_ERROR(LOGGER, "等待 ROBOT_COORD 超时或失败");
    return false;
  }
  // 3. 发送机器人坐标
  std::string coord_msg = ResponseCode::ROBOT_COORD;
  for (float v : fl_tcp_position)
    coord_msg += "," + std::to_string(v);
  if (!send(coord_msg)) {
    RCLCPP_ERROR(LOGGER, "发送机器人坐标失败");
    return false;
  }
  // 4. 等待目标位姿
  if (!waitForResponse(ResponseCode::SEND_POS, 30)) {
    RCLCPP_ERROR(LOGGER, "等待 SEND_POS 超时或失败");
    return false;
  }
  // 5. 拷贝目标位姿
  if (target_pose != nullptr) {
    std::lock_guard<std::mutex> pose_lock(target_pose_mutex_);
    *target_pose = target_pose_;
  }
  RCLCPP_INFO(LOGGER, "=== RunProject 完成 ===");
  return true;
}

void AlsonClient::parseResponse(const std::string &response) {
  try {
    std::istringstream iss(response);
    std::string command;
    std::getline(iss, command, ',');
    if (command.empty()) {
      RCLCPP_ERROR(LOGGER, "Parse response error: empty command");
      return;
    }

    std::vector<float> values;
    std::string value;
    while (std::getline(iss, value, ',')) {
      try {
        if (!value.empty())
          values.push_back(std::stof(value));
      } catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "Parse response value error: [%s], %s",
                     value.c_str(), e.what());
      }
    }
    const std::string response_code = command;
    if (waiting_response_ && response_code == expected_code_) {
      last_response_code_ = response_code;
      response_received_ = true;
      if (response_code == ResponseCode::SEND_POS && !values.empty()) {
        std::lock_guard<std::mutex> pose_lock(target_pose_mutex_);
        target_pose_ = values;
      }
      response_cv_.notify_one();
    }
    RCLCPP_INFO(LOGGER, "Response: %s", response.c_str());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(LOGGER, "Parse response error: %s", e.what());
  }
}

// 命令实现
bool AlsonClient::ChangeProject(const std::string &project_name) {
  std::string msg = std::string(CommandType::CHANGE_PRJ) + "," + project_name;
  RCLCPP_INFO(LOGGER, "=== ChangeProject 开始: %s ===", project_name.c_str());
  if (!send(msg)) {
    RCLCPP_ERROR(LOGGER, "ChangeProject 发送命令失败: %s",
                 project_name.c_str());
    return false;
  }
  if (!waitForResponse(ResponseCode::PRJ_CHANGE_SUCCESS, 10)) {
    RCLCPP_ERROR(LOGGER, "ChangeProject 超时或失败: %s", project_name.c_str());
    return false;
  }
  RCLCPP_INFO(LOGGER, "=== ChangeProject 成功: %s ===", project_name.c_str());
  return true;
}

void AlsonClient::startAsyncReconnect() {
  std::lock_guard<std::mutex> lock(reconnect_mutex_);

  // 如果已经在重连中，直接返回
  if (reconnecting_.load()) {
    return;
  }

  reconnecting_ = true;
  retry_count_++;

  // 启动异步重连线程
  reconnect_thread_ = std::make_unique<std::thread>([this]() {
    const int max_retries = 5;

    if (retry_count_ <= max_retries) {
      // 指数退避：2, 4, 8, 16, 30秒
      int delay = std::min(30, (1 << retry_count_));

      RCLCPP_WARN(LOGGER, "发送失败，%d秒后重试 (%d/%d)", delay,
                  retry_count_.load(), max_retries);
      if (structured_event_callback_) {
        EventData event{EventType::RECONNECT_STATUS,
                        true,
                        "Reconnect wait: " + std::to_string(delay) + "s",
                        "",
                        0,
                        0};
        structured_event_callback_(event);
      }

      std::this_thread::sleep_for(std::chrono::seconds(delay));

      // 尝试重连
      if (Connect()) {
        RCLCPP_INFO(LOGGER, "重连成功");
        if (structured_event_callback_) {
          EventData event{
              EventType::RECONNECT_STATUS, true, "Reconnect success", "", 0, 0};
          structured_event_callback_(event);
        }
        retry_count_ = 0; // 重置计数器
      } else {
        RCLCPP_ERROR(LOGGER, "重连失败");
        if (structured_event_callback_) {
          EventData event{
              EventType::RECONNECT_STATUS, false, "Reconnect failed", "", 0, 0};
          structured_event_callback_(event);
        }
      }
    } else {
      RCLCPP_ERROR(LOGGER, "重连失败，已达到最大重试次数");
      if (structured_event_callback_) {
        EventData event{
            EventType::RECONNECT_STATUS, false, "Reconnect giveup", "", 0, 0};
        structured_event_callback_(event);
      }
      retry_count_ = 0; // 重置计数器
    }

    reconnecting_ = false;
  });

  // 分离线程，不等待其完成
  reconnect_thread_->detach();
}

void AlsonClient::startHeartbeatThread() {
  std::lock_guard<std::mutex> lock(heartbeat_mutex_);
  if (heartbeat_running_)
    return;
  heartbeat_running_ = true;
  heartbeat_thread_ = std::make_unique<std::thread>([this]() {
    while (heartbeat_running_) {
      std::unique_lock<std::mutex> lk(heartbeat_mutex_);
      heartbeat_cv_.wait_for(lk, std::chrono::seconds(heartbeat_interval_sec_));
      if (!heartbeat_running_)
        break;
      sendHeartbeat();
    }
  });
}

void AlsonClient::sendHeartbeat() {
  // 发送心跳包，可以自定义内容，如"PING"或特殊命令
  if (!connected_)
    return;
  std::string heartbeat_msg = "PING";
  if (!send(heartbeat_msg)) {
    RCLCPP_WARN(LOGGER, "Heartbeat send failed, will trigger reconnect");
    if (structured_event_callback_) {
      EventData event{
          EventType::HEARTBEAT_STATUS, false, "Heartbeat failed", "", 0, 0};
      structured_event_callback_(event);
    }
    // 这里可以选择立即断开重连
    Disconnect();
    startAsyncReconnect();
  } else {
    RCLCPP_DEBUG(LOGGER, "Heartbeat sent");
    if (structured_event_callback_) {
      EventData event{
          EventType::HEARTBEAT_STATUS, true, "Heartbeat sent", "", 0, 0};
      structured_event_callback_(event);
    }
  }
}
} // namespace alson_client
