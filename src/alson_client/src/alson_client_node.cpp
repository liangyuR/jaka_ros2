#include "alson_client/alson_client_node.h"
#include <nlohmann/json.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace alson_client {

AlsonClientNode::AlsonClientNode(const rclcpp::NodeOptions &options)
    : Node("alson_client_node", options) {
  alson_events_publisher_ =
      this->create_publisher<std_msgs::msg::String>("alson_events", 10);

  // 注册 runproj service
  runproj_service_ = this->create_service<jaka_msgs::srv::RunProject>(
      "~/run_project",
      [this](
          const std::shared_ptr<jaka_msgs::srv::RunProject::Request> &request,
          std::shared_ptr<jaka_msgs::srv::RunProject::Response> response) {
        this->handleRunProjService(request, response);
      });
  initAlsonClient();

  RCLCPP_INFO(this->get_logger(), "AlsonClientNode initialized");
}

void AlsonClientNode::initAlsonClient() {
  RCLCPP_INFO(this->get_logger(), "Initializing AlsonClient");
  const auto host =
      this->declare_parameter<std::string>("host", "192.168.0.188");
  const auto port = this->declare_parameter<int>("port", 54600);
  client_ = AlsonClient::GetInstance(host, port);
  client_->setEventCallback([this](const std::string &event) {
    RCLCPP_INFO(this->get_logger(), "Alson event: %s", event.c_str());
    PublishEvents(event);
  });
  RCLCPP_INFO(this->get_logger(), "AlsonClient initialized");
}

void AlsonClientNode::handleRunProjService(
    const std::shared_ptr<jaka_msgs::srv::RunProject::Request> &request,
    std::shared_ptr<jaka_msgs::srv::RunProject::Response> &response) {
  if (!validateRequest(request, response)) {
    return;
  }

  std::vector<float> pose;
  std::string message;
  auto success = executeProject(request->project_id, request->fl_tcp_position,
                                &pose, &message);
  fillResponse(response, success, pose, message);

  auto json = nlohmann::json::object();
  json["project_id"] = request->project_id;
  json["status_code"] = response->status_code;
  json["success"] = success;
  json["pose"] = pose;
  json["message"] = message;
  PublishEvents(json.dump());
}

void AlsonClientNode::PublishEvents(const std::string &json) {
  auto msg = std_msgs::msg::String();
  msg.data = json;
  alson_events_publisher_->publish(msg);
  RCLCPP_DEBUG(this->get_logger(), "Published response: %s", json.c_str());
}

bool AlsonClientNode::validateRequest(
    const std::shared_ptr<jaka_msgs::srv::RunProject::Request> &request,
    std::shared_ptr<jaka_msgs::srv::RunProject::Response> &response) {
  if (request->project_id.empty() || request->fl_tcp_position.size() != 6) {
    response->status_code = -1;
    response->message = "Project ID is empty or fl_tcp_position is not 6D";
    return false;
  }
  return true;
}

void AlsonClientNode::fillResponse(
    std::shared_ptr<jaka_msgs::srv::RunProject::Response> &response,
    bool success, const std::vector<float> &pose, const std::string &message) {
  response->pose = pose;
  response->message = message;
  response->status_code = success ? 0 : -1;
}

bool AlsonClientNode::executeProject(const std::string &project_id,
                                     const std::vector<float> &fl_tcp_position,
                                     std::vector<float> *pose,
                                     std::string *message) {
  std::lock_guard<std::mutex> lock(client_mutex_);

  RCLCPP_INFO(this->get_logger(), "Change project to %s", project_id.c_str());
  if (!client_->ChangeProject(project_id)) {
    *message = "Project change failed";
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Run project");
  bool success = client_->RunProject(fl_tcp_position, pose);
  if (!success) {
    *message = "Project execution failed";
    return false;
  }
  *message = "Project executed successfully";
  return true;
}
} // namespace alson_client
