#include "auto_charge/auto_charge_node.h"
#include "auto_charge/camera_controller.h"
#include <iostream>
#include <memory>

namespace auto_charge {

AutoChargeNode::AutoChargeNode() : Node("auto_charge_node") {

  // 创建相机控制器，传入当前节点指针
  camera_controller_ = std::make_unique<CameraController>(this);

  RCLCPP_INFO(this->get_logger(),
              "AutoChargeNode initialized with CameraController");
}

AutoChargeNode::~AutoChargeNode() {}

} // namespace auto_charge
