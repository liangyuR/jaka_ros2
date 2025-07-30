#include "robot_controller/robot_controller.h"
#include <csignal>

void sigintHandler(int /*sig*/) { rclcpp::shutdown(); }

int main(int argc, char **argv) {
  setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);
  signal(SIGINT, sigintHandler);

  auto controller = std::make_shared<robot_controller::RobotController>();

  // 初始化机器人
  if (!controller->InitializeRobot()) {
    RCLCPP_ERROR(controller->get_logger(),
                 "Failed to initialize robot. Exiting.");
    return 1;
  }

  // 启动控制器
  controller->StartController();

  RCLCPP_INFO(controller->get_logger(),
              "Robot controller started successfully");

  // 主循环
  rclcpp::spin(controller);

  rclcpp::shutdown();
  return 0;
}
