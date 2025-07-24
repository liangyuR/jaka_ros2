#pragma once

#include <atomic>
#include <boost/asio.hpp>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace alson_client {

struct CommandType {
  static constexpr const char *CHANGE_PRJ = "510";
  static constexpr const char *RUN_PRJ = "210";
  static constexpr const char *CHANGE_PARA = "520";
  static constexpr const char *CHANGE_MODE = "530";
  static constexpr const char *REQ_ROBOT_COORD = "000";
};

struct ResponseCode {
  static constexpr const char *PRJ_CHANGE_SUCCESS = "512";
  static constexpr const char *PRJ_CHANGE_FAIL = "511";
  static constexpr const char *RUN_PRJ_FAILED = "403";
  static constexpr const char *RUN_PRJ_FAILED_PLAN = "405";
  static constexpr const char *RUN_PRJ_FAILED_TOTE_EMPTY = "406";
  static constexpr const char *PARA_CHANGE_SUCCESS = "522";
  static constexpr const char *PARA_CHANGE_FAIL = "521";
  static constexpr const char *MODE_CHANGE_SUCCESS = "532";
  static constexpr const char *MODE_CHANGE_FAIL = "531";
  static constexpr const char *ROBOT_COORD = "000";
  // 发送抓取位 || 目标点位, tf_target_in_base
  static constexpr const char *SEND_POS = "310";
  static constexpr const char *SCAN_DONE = "408";
};

class AlsonClient {
public:
  static AlsonClient *GetInstance(const std::string &host, int port);

  bool Connect();
  void Disconnect();

  bool ChangeProject(const std::string &project_name);
  bool RunProject(const std::vector<float> &fl_tcp_position,
                  std::vector<float> *target_pose);

  void
  setEventCallback(const std::function<void(const std::string &)> &callback) {
    event_callback_ = callback;
  }

private:
  AlsonClient(const std::string &host, int port);
  void receiveLoop();
  bool send(const std::string &msg);
  void parseResponse(const std::string &response);
  void startAsyncReconnect();
  bool waitForResponse(const std::string &expect_code, int timeout_sec);

  // 心跳机制
  void startHeartbeatThread();
  void sendHeartbeat();
  std::unique_ptr<std::thread> heartbeat_thread_;
  std::atomic<bool> heartbeat_running_{false};
  std::mutex heartbeat_mutex_;
  std::condition_variable heartbeat_cv_;
  int heartbeat_interval_sec_ = 5;

  boost::asio::io_context io_context_;
  std::string host_;
  int port_;
  std::atomic<bool> connected_{false};
  std::atomic<bool> running_{false};
  std::unique_ptr<std::thread> recv_thread_;
  std::mutex socket_mutex_;
  std::unique_ptr<boost::asio::ip::tcp::socket> socket_;

  // 重连相关成员变量
  std::atomic<int> retry_count_{0};
  std::atomic<bool> reconnecting_{false};
  std::unique_ptr<std::thread> reconnect_thread_;
  std::mutex reconnect_mutex_;

  // 同步等待相关成员变量
  std::atomic<bool> waiting_response_{false};
  std::atomic<bool> response_received_{false};
  std::string last_response_code_;
  std::string expected_code_;
  std::mutex response_mutex_;
  std::condition_variable response_cv_;

  // 存储目标位姿
  std::vector<float> target_pose_;
  std::mutex target_pose_mutex_;

  // 事件回调
  std::function<void(const std::string &)> event_callback_;
};

} // namespace alson_client
