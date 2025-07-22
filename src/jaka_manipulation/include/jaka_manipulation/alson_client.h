#pragma once
#include <atomic>
#include <boost/asio.hpp>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace jaka_manipulation {

enum class CommandType {
  CHANGE_PRJ,
  CHANGE_PARA,
  CHANGE_MODE,
  RUN_PRJ,
  SCAN_DONE,
  REQ_ROBOT_COORD,
  SEND_POS
};

enum class ResponseCode {
  PRJ_CHANGE_SUCCESS,
  PRJ_CHANGE_FAIL,
  PARA_CHANGE_SUCCESS,
  PARA_CHANGE_FAIL,
  MODE_CHANGE_SUCCESS,
  MODE_CHANGE_FAIL,
  RUN_SUCCESS,
  RUN_INVALID_PARAM,
  RUN_FAIL,
  SCAN_DONE,
  ROBOT_COORD,
  SEND_POS
};

enum class VisionType { CHARGE, CONNECT, PLACE, BOX };

class AlsonClient {
public:
  static AlsonClient *GetInstance(const std::string &host, int port);

  bool Connect();
  void Disconnect();
  bool ChangeProject(const std::string &project_name);
  bool ChangeParameter(const std::string &param_group_name);
  bool ChangeMode(const std::string &template_name);
  bool RunProject();
  bool ScanDone();
  bool RequestRobotCoord();

  // 事件回调注册
  void RegisterResponseCallback(
      std::function<void(ResponseCode, const std::vector<double> &)> cb) {
    response_cb_ = cb;
  }

private:
  AlsonClient(const std::string &host, int port);
  void receiveLoop();
  bool send(const std::string &msg);
  void parseResponse(const std::string &response);
  void startAsyncReconnect();

  boost::asio::io_context io_context_;
  std::string host_;
  int port_;
  std::atomic<bool> connected_{false};
  std::atomic<bool> running_{false};
  std::unique_ptr<std::thread> recv_thread_;
  std::mutex socket_mutex_;
  std::unique_ptr<boost::asio::ip::tcp::socket> socket_;
  std::function<void(ResponseCode, const std::vector<double> &)> response_cb_;

  // 重连相关成员变量
  std::atomic<int> retry_count_{0};
  std::atomic<bool> reconnecting_{false};
  std::unique_ptr<std::thread> reconnect_thread_;
  std::mutex reconnect_mutex_;
};

} // namespace jaka_manipulation
