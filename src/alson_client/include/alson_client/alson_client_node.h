#pragma once

#include "alson_client/alson_client.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <jaka_msgs/msg/alson_event.hpp>
#include <jaka_msgs/srv/get_pose.hpp>
#include <jaka_msgs/srv/run_project.hpp>
#include <jaka_msgs/srv/update_camera_para.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>

namespace alson_client {

using RunProject = jaka_msgs::srv::RunProject;
using UpdateCameraPara = jaka_msgs::srv::UpdateCameraPara;

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
  ~AlsonClientNode() override;

private:
  // Service servers
  rclcpp::Service<jaka_msgs::srv::RunProject>::SharedPtr runproj_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disconnect_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restart_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_status_service_;

  // Service callbacks
  void handleRunProjService(
      const std::shared_ptr<jaka_msgs::srv::RunProject::Request> &request,
      std::shared_ptr<jaka_msgs::srv::RunProject::Response> &response);
  void handleConnectService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> &request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> &response);
  void handleDisconnectService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> &request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> &response);
  void handleRestartService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> &request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> &response);
  void handleGetStatusService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> &request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> &response);

  // 发布响应到 topic
  void PublishEvents(const std::string &json);
  void PublishAlsonEvent(uint8_t event_type, uint8_t status,
                         const std::string &message, bool connected = false,
                         const std::string &data = "", int retry_count = 0,
                         int delay_seconds = 0);

  // Topic publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alson_events_publisher_;
  rclcpp::Publisher<jaka_msgs::msg::AlsonEvent>::SharedPtr
      alson_event_publisher_;

  // business object
  AlsonClient *client_;
  std::mutex client_mutex_;

  bool executeProject(const std::string &project_id,
                      const std::vector<float> &fl_tcp_position,
                      std::vector<float> *pose, std::string *message);

  static bool validateRequest(
      const std::shared_ptr<jaka_msgs::srv::RunProject::Request> &request,
      std::shared_ptr<jaka_msgs::srv::RunProject::Response> &response);

  static void
  fillResponse(std::shared_ptr<jaka_msgs::srv::RunProject::Response> &response,
               bool success, const std::vector<float> &pose,
               const std::string &message);

  // 重启相关
  void performRestart();
  std::thread restart_thread_;
  std::atomic<bool> restarting_{false};
};

} // namespace alson_client