#include "alson_client/alson_client.h"
#include <boost/asio.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

namespace alson_client {
#define LOGGER rclcpp::get_logger("alson_client::AlsonClient")

AlsonClient *AlsonClient::GetInstance(const std::string &host, int port) {
  static AlsonClient instance(host, port);
  return &instance;
}

AlsonClient::AlsonClient(const std::string &host, int port) // NOLINT
    : host_(host), port_(port) {
  std::thread([this]() { this->Connect(); }).detach();
}

bool AlsonClient::Connect() {
  RCLCPP_INFO(LOGGER, "Connecting AlsonClient to %s:%d", host_.c_str(), port_);
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (connected_) {
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
    return true;
  } catch (std::exception &e) {
    RCLCPP_ERROR(LOGGER, "Connect failed: %s", e.what());
    connected_ = false;
    return false;
  }
}

void AlsonClient::Disconnect() {
  RCLCPP_INFO(LOGGER, "Disconnecting AlsonClient");
  running_ = false;
  if (recv_thread_ && recv_thread_->joinable()) {
    recv_thread_->join();
  }

  // Clear reconnecting state
  reconnecting_ = false;
  if (reconnect_thread_ && reconnect_thread_->joinable()) {
    reconnect_thread_->join();
  }

  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (socket_) {
    try {
      socket_->close();
    } catch (...) {
    }
    socket_.reset();
  }
  connected_ = false;
  RCLCPP_INFO(LOGGER, "AlsonClient disconnected");
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
      std::array<char, 4096> data{};
      size_t len = socket_->read_some(boost::asio::buffer(data));
      if (len == 0) {
        connected_ = false;
        break;
      }
      std::string response(data.data(), len);
      parseResponse(response);
    } catch (std::exception &e) {
      RCLCPP_ERROR(LOGGER, "Receive error: %s", e.what());
      connected_ = false;
      break;
    }
  }
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

      std::this_thread::sleep_for(std::chrono::seconds(delay));

      // 尝试重连
      if (Connect()) {
        RCLCPP_INFO(LOGGER, "重连成功");
        retry_count_ = 0; // 重置计数器
      } else {
        RCLCPP_ERROR(LOGGER, "重连失败");
      }
    } else {
      RCLCPP_ERROR(LOGGER, "重连失败，已达到最大重试次数");
      retry_count_ = 0; // 重置计数器
    }

    reconnecting_ = false;
  });

  // 分离线程，不等待其完成
  reconnect_thread_->detach();
}
} // namespace alson_client
