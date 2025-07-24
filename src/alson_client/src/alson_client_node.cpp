#include "alson_client/alson_client_node.h"
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
  RCLCPP_INFO(this->get_logger(), "AlsonClient initialized");
}

void AlsonClientNode::handleRequestRobotCoord() {
  // TODO(@liangyu) get robot coord from jaka_planner, send to alson client
}

void AlsonClientNode::handleRunProjService(
    const std::shared_ptr<jaka_msgs::srv::RunProject::Request> &request,
    std::shared_ptr<jaka_msgs::srv::RunProject::Response> &response) {

  if (!validateRequest(request, response)) {
    return;
  }

  if (request->wait_for_completion) {
    // 阻塞模式
    auto result = executeProject(request->project_id, request->fl_tcp_position);
    fillResponse(response, result);
    onResponseCallback(ResponseCode::SEND_POS, result.pose);
  } else {
    // 异步模式
    response->success = true;
    response->message = "Request accepted, processing asynchronously...";
    response->status_code = 1;
    onResponseCallback(ResponseCode::RUN_PRJ_SUCCESS, {});

    std::thread([this, request]() {
      auto result =
          executeProject(request->project_id, request->fl_tcp_position);
      onResponseCallback(ResponseCode::SEND_POS, result.pose);
    }).detach();
  }
}

void AlsonClientNode::onResponseCallback(const std::string &code,
                                         const std::vector<float> &data) {
  auto msg = std_msgs::msg::String();

  std::string response_str = "ResponseCode:" + code;
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

bool AlsonClientNode::validateRequest(
    const std::shared_ptr<jaka_msgs::srv::RunProject::Request> &request,
    std::shared_ptr<jaka_msgs::srv::RunProject::Response> &response) {
  if (request->project_id.empty() || request->fl_tcp_position.size() != 6) {
    response->success = false;
    response->message = "Project ID is empty or fl_tcp_position is not 6D";
    return false;
  }
  return true;
}

void AlsonClientNode::fillResponse(
    std::shared_ptr<jaka_msgs::srv::RunProject::Response> &response,
    const ProjectExecutionResult &result) {
  response->success = result.success;
  response->pose = result.pose;
  response->message = result.message;
  response->status_code = result.success ? 0 : -1;
}

AlsonClientNode::ProjectExecutionResult
AlsonClientNode::executeProject(const std::string &project_id,
                                const std::vector<float> &fl_tcp_position) {
  ProjectExecutionResult result;
  std::lock_guard<std::mutex> lock(client_mutex_);

  RCLCPP_INFO(this->get_logger(), "Change project to %s", project_id.c_str());
  bool success = client_->ChangeProject(project_id);

  if (success) {
    // 发布切换项目成功的消息
    onResponseCallback(ResponseCode::PRJ_CHANGE_SUCCESS, {});
    RCLCPP_INFO(this->get_logger(), "Run project");
    success = client_->RunProject(fl_tcp_position, &result.pose);
  }

  result.success = success;
  result.message =
      success ? "Project executed successfully" : "Project execution failed";

  return result;
}
} // namespace alson_client
