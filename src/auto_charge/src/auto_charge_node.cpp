#include "auto_charge/auto_charge_node.h"
#include "auto_charge/camera_controller.h"
#include <iostream>
#include <memory>

namespace auto_charge {

AutoChargeNode::AutoChargeNode() : Node("auto_charge_node") {

  // 创建相机控制器和机器人管理器，它们内部会创建自己的节点
  camera_controller_ = std::make_unique<CameraController>();
  robot_manager_ = std::make_unique<RobotManager>();

  RCLCPP_INFO(this->get_logger(),
              "AutoChargeNode initialized with CameraController");
}

AutoChargeNode::~AutoChargeNode() {}

} // namespace auto_charge
