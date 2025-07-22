#include "jaka_manipulation/alson_client.h"
#include <boost/asio.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

namespace jaka_manipulation {
#define LOGGER rclcpp::get_logger("jaka_manipulation::AlsonClient")

AlsonClient *AlsonClient::GetInstance(const std::string &host, int port) {
  static AlsonClient instance(host, port);
  return &instance;
}

AlsonClient::AlsonClient(const std::string &host, int port) // NOLINT
    : host_(host), port_(port) {}

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

void AlsonClient::parseResponse(const std::string &response) {
  try {
    std::istringstream iss(response);
    std::string command;
    if (!std::getline(iss, command, ',')) {
      RCLCPP_ERROR(LOGGER, "Parse response error: empty command");
      return;
    }

    // 检查 command 是否为数字
    int cmd_code = 0;
    try {
      cmd_code = std::stoi(command);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(LOGGER, "Parse response error: invalid command [%s]",
                   command.c_str());
      return;
    }

    std::vector<double> values;
    std::string value;
    while (std::getline(iss, value, ',')) {
      try {
        if (!value.empty())
          values.push_back(std::stod(value));
      } catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "Parse response value error: [%s], %s",
                     value.c_str(), e.what());
      }
    }

    if (response_cb_) {
      response_cb_(static_cast<ResponseCode>(cmd_code), values);
    }
    RCLCPP_INFO(LOGGER, "Response: %s", response.c_str());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(LOGGER, "Parse response error: %s", e.what());
  }
}

// 命令实现
bool AlsonClient::ChangeProject(const std::string &project_name) {
  std::string msg = "510," + project_name;
  return send(msg);
}

bool AlsonClient::ChangeParameter(const std::string &param_group_name) {
  std::string msg = "ChangePara," + param_group_name;
  return send(msg);
}

bool AlsonClient::ChangeMode(const std::string &template_name) {
  std::string msg = "ChangeMode," + template_name;
  return send(msg);
}

bool AlsonClient::RunProject() { return send("210"); }

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

bool AlsonClient::ScanDone() { return send("ScanDone"); }

bool AlsonClient::RequestRobotCoord() { return send("REQ_RobotCoord"); }
} // namespace jaka_manipulation
