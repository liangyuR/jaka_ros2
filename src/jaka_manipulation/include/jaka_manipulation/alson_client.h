#pragma once
#include <string>
#include <thread>
#include <atomic>
#include <memory>
#include <functional>
#include <vector>
#include <mutex>
#include <boost/asio.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


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

enum class VisionType {
    CHARGE,
    CONNECT,
    PLACE,
    BOX
};

class AlsonClient {
public:
    static AlsonClient* GetInstance(const rclcpp::Node::SharedPtr& node, const std::string& host, int port);

    bool Connect();
    void Disconnect();
    bool ChangeProject(const std::string& project_name);
    bool ChangeParameter(const std::string& param_group_name);
    bool ChangeMode(const std::string& template_name);
    bool RunProject();
    bool ScanDone();
    bool RequestRobotCoord();

    // 事件回调注册
    void RegisterResponseCallback(std::function<void(ResponseCode, const std::vector<double>&)> cb);

private:
    AlsonClient(const rclcpp::Node::SharedPtr& node, const std::string& host, int port);
    void receiveLoop_();
    bool send(const std::string& msg);
    void parseResponse_(const std::string& response);

    rclcpp::Node::SharedPtr node_;
    boost::asio::io_context io_context_;
    std::string host_;
    int port_;
    std::atomic<bool> connected_;
    std::atomic<bool> running_;
    std::unique_ptr<std::thread> recv_thread_;
    std::mutex socket_mutex_;
    std::unique_ptr<boost::asio::ip::tcp::socket> socket_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::function<void(ResponseCode, const std::vector<double>&)> response_cb_;
};

} // namespace jaka_manipulation
