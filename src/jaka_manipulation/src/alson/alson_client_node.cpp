#include "jaka_manipulation/alson_client_node.h"

namespace jaka_manipulation {

AlsonClientNode::AlsonClientNode(const rclcpp::NodeOptions &options)
    : Node("alson_client_node", options) {
  const auto host =
      this->declare_parameter<std::string>("host", "192.168.0.188");
  const auto port = this->declare_parameter<int>("port", 54600);

  client_ = AlsonClient::GetInstance(host, port);
  client_->RegisterResponseCallback(
      [this](ResponseCode code, const std::vector<double> &data) {
        this->onResponseCallback(code, data);
      });

  // topic publisher
  alson_events_publisher_ =
      this->create_publisher<std_msgs::msg::String>("alson_events", 10);

  RCLCPP_INFO(this->get_logger(), "AlsonClientNode initialized");
}

void AlsonClientNode::handleRunProject() {
  std::lock_guard<std::mutex> lock(client_mutex_);
  bool success = client_->RunProject();

  RCLCPP_INFO(this->get_logger(), "Run project called, result: %s",
              success ? "success" : "failed");
}

void AlsonClientNode::handleScanDone() {
  std::lock_guard<std::mutex> lock(client_mutex_);
  bool success = client_->ScanDone();

  RCLCPP_INFO(this->get_logger(), "Scan done called, result: %s",
              success ? "success" : "failed");
}

void AlsonClientNode::handleRequestRobotCoord() {
  std::lock_guard<std::mutex> lock(client_mutex_);
  bool success = client_->RequestRobotCoord();

  RCLCPP_INFO(this->get_logger(), "Request robot coord called, result: %s",
              success ? "success" : "failed");
}

void AlsonClientNode::onResponseCallback(ResponseCode code,
                                         const std::vector<double> &data) {
  auto msg = std_msgs::msg::String();

  std::string response_str =
      "ResponseCode:" + std::to_string(static_cast<int>(code));
  if (!data.empty()) {
    response_str += ",Data:";
    for (size_t i = 0; i < data.size(); ++i) {
      if (i > 0)
        response_str += ",";
      response_str += std::to_string(data[i]);
    }
  }

  msg.data = response_str;
  alson_events_publisher_->publish(msg);

  RCLCPP_DEBUG(this->get_logger(), "Published response: %s",
               response_str.c_str());
}

} // namespace jaka_manipulation
