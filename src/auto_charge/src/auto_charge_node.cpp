#include "auto_charge/auto_charge_node.h"
#include "auto_charge/camera_controller.h"
#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace auto_charge {

AutoChargeNode::AutoChargeNode(QObject *parent)
    : QObject(parent), Node("auto_charge_node"), ros_running_(false) {

  // 创建相机控制器，传入当前节点指针
  camera_controller_ = std::make_unique<CameraController>(this, this);
  // 启动ROS2节点线程
  ros_running_ = true;
  ros_thread_ = std::thread(&AutoChargeNode::SpinRosNode, this);

  qDebug() << "AutoChargeNode initialized with CameraController";
}

AutoChargeNode::~AutoChargeNode() {
  ros_running_ = false;
  if (ros_thread_.joinable()) {
    ros_thread_.join();
  }
}

void AutoChargeNode::SpinRosNode() {
  while (ros_running_) {
    rclcpp::spin_some(this->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
} // namespace auto_charge
