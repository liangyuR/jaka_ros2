#pragma once

#include "alson_client/alson_client.h"
#include <algorithm>
#include <chrono>
#include <future>
#include <jaka_msgs/srv/get_pose.hpp>
#include <jaka_msgs/srv/run_project.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>

namespace alson_client {

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
  void initAlsonClient();
  // Service servers
  rclcpp::Service<jaka_msgs::srv::RunProject>::SharedPtr runproj_service_;

  // Service callbacks
  void handleRunProjService(
      const std::shared_ptr<jaka_msgs::srv::RunProject::Request> &request,
      std::shared_ptr<jaka_msgs::srv::RunProject::Response> &response);
  void handleRunProject();
  void handleRequestRobotCoord();

  // Response callback for AlsonClient
  void onResponseCallback(const std::string &code,
                          const std::vector<float> &data);

  // Topic publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alson_events_publisher_;

  // business object
  AlsonClient *client_;
  std::mutex client_mutex_;

  struct ProjectExecutionResult {
    bool success;
    std::vector<float> pose;
    std::string message;
  };

  ProjectExecutionResult
  executeProject(const std::string &project_id,
                 const std::vector<float> &fl_tcp_position);

  static bool validateRequest(
      const std::shared_ptr<jaka_msgs::srv::RunProject::Request> &request,
      std::shared_ptr<jaka_msgs::srv::RunProject::Response> &response);

  static void
  fillResponse(std::shared_ptr<jaka_msgs::srv::RunProject::Response> &response,
               const ProjectExecutionResult &result);
};

} // namespace alson_client