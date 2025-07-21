#include "jaka_manipulation/alson_client.h"
#include <iostream>
#include <sstream>
#include <boost/asio.hpp>

namespace jaka_manipulation {

AlsonClient* AlsonClient::GetInstance(const rclcpp::Node::SharedPtr& node, const std::string& host, int port) {
    static AlsonClient instance(node, host, port);
    return &instance;
}

AlsonClient::AlsonClient(const rclcpp::Node::SharedPtr& node, const std::string& host, int port) // NOLINT
    : host_(host), port_(port), connected_(false), running_(false) {
    node_ = node;
    publisher_ = node_->create_publisher<std_msgs::msg::String>("alson_events", 10);
    auto timer = node_->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
            if (connected_) {
                return;
            }

            if (Connect()) {
                RCLCPP_INFO(node_->get_logger(), "AlsonClient connected to %s:%d", host_.c_str(), port_);
            } else {
                RCLCPP_WARN(node_->get_logger(), "AlsonClient connect failed, retrying...");
            }
        }
    );
}

bool AlsonClient::Connect() {
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
        recv_thread_ = std::make_unique<std::thread>(&AlsonClient::receiveLoop_, this);
        return true;
    } catch (std::exception& e) {
        if (node_) {
            RCLCPP_ERROR(node_->get_logger(), "连接失败: %s", e.what());
        } else {
            std::cerr << "连接失败: " << e.what() << std::endl;
        }
        connected_ = false;
        return false;
    }
}

void AlsonClient::Disconnect() {
    running_ = false;
    if (recv_thread_ && recv_thread_->joinable()) {
        recv_thread_->join();
    }
    
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (socket_) {
        try {
            socket_->close();
        } catch (...) {}
        socket_.reset();
    }
    connected_ = false;
}

bool AlsonClient::send(const std::string& msg) { // NOLINT
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (!connected_ || !socket_) {
        return false;
    }

    try {
        boost::asio::write(*socket_, boost::asio::buffer(msg));
    } catch (const std::exception& e) {
        connected_ = false;
        if (node_) {
            RCLCPP_ERROR(node_->get_logger(), "发送失败: %s", e.what());
        } else {
            std::cerr << "发送失败: " << e.what() << std::endl;
        }
        return false;
    }

    if (node_) {
        RCLCPP_DEBUG(node_->get_logger(), "发送消息: %s", msg.c_str());
    }
    return true;
}

void AlsonClient::receiveLoop_() {
    while (running_ && connected_) {
        try {
            std::array<char, 4096> data{};
            size_t len = socket_->read_some(boost::asio::buffer(data));
            if (len == 0) {
                connected_ = false;
                break;
            }
            std::string response(data.data(), len);
            parseResponse_(response);
        } catch (std::exception& e) {
            if (node_) {
                RCLCPP_ERROR(node_->get_logger(), "接收错误: %s", e.what());
            } else {
                std::cerr << "接收错误: " << e.what() << std::endl;
            }
            connected_ = false;
            break;
        }
    }
}

void AlsonClient::parseResponse_(const std::string& response) {
    // 发布原始响应到 ROS2
    if (publisher_) {
        auto msg = std_msgs::msg::String();
        msg.data = response;
        publisher_->publish(msg);
    }
    
    // 解析响应类型
    try {
        std::istringstream iss(response);
        std::string command;
        std::getline(iss, command, ',');
        
        std::vector<double> values;
        std::string value;
        while (std::getline(iss, value, ',')) {
            try {
                values.push_back(std::stod(value));
            } catch (...) {}
        }
        
        // 这里可以添加响应类型处理逻辑
        // 例如: if (command == "512") { ... }
        
    } catch (std::exception& e) {
        if (node_) {
            RCLCPP_ERROR(node_->get_logger(), "解析错误: %s", e.what());
        } else {
            std::cerr << "解析错误: " << e.what() << std::endl;
        }
    }
}

// 命令实现
bool AlsonClient::ChangeProject(const std::string& project_name) {
    std::string msg = "510," + project_name;
    return send(msg);
}

bool AlsonClient::ChangeParameter(const std::string& param_group_name) {
    std::string msg = "ChangePara," + param_group_name;
    return send(msg);
}

bool AlsonClient::ChangeMode(const std::string& template_name) {
    std::string msg = "ChangeMode," + template_name;
    return send(msg);
}

bool AlsonClient::RunProject() {
    return send("210");
}

bool AlsonClient::ScanDone() {
    return send("ScanDone");
}

bool AlsonClient::RequestRobotCoord() {
    return send("REQ_RobotCoord");
}
} // namespace jaka_manipulation
