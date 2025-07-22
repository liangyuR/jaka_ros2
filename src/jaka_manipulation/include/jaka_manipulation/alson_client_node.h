#pragma once
#include "jaka_manipulation/alson_client.h"
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>

namespace jaka_manipulation {

struct TriggerRequest {
  bool dummy;
};

struct TriggerResponse {
  bool success;
  std::string message;
};

class AlsonClientNode : public rclcpp::Node {
public:
  explicit AlsonClientNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  // Service servers
  void handleRunProject();
  void handleChangeProject();
  void handleScanDone();
  void handleRequestRobotCoord();

  // Response callback for AlsonClient
  void onResponseCallback(ResponseCode code, const std::vector<double> &data);

  // Topic publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alson_events_publisher_;

  // business object
  AlsonClient *client_;
  std::mutex client_mutex_;
};

} // namespace jaka_manipulation