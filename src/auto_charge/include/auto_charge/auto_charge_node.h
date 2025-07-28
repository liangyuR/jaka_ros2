#ifndef AUTO_CHARGE_NODE_H
#define AUTO_CHARGE_NODE_H

#include <QObject>
#include <QVariant>
#include <QVariantMap>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

// 包含完整的 CameraController 定义
#include "auto_charge/camera_controller.h"

namespace auto_charge {

class AutoChargeNode : public QObject, public rclcpp::Node {
  Q_OBJECT

public:
  explicit AutoChargeNode(QObject *parent = nullptr);
  virtual ~AutoChargeNode();

  // 获取相机控制器
  Q_INVOKABLE QObject *GetCameraController() const {
    return camera_controller_.get();
  }

private:
  // 相机控制器
  std::unique_ptr<CameraController> camera_controller_;

  // ROS2节点线程
  std::thread ros_thread_;
  bool ros_running_;

  // 私有方法
  void SpinRosNode();
};

} // namespace auto_charge

#endif // AUTO_CHARGE_NODE_H