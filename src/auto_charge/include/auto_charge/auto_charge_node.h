#ifndef AUTO_CHARGE_NODE_H
#define AUTO_CHARGE_NODE_H

#include "auto_charge/camera_controller.h"
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace auto_charge {

class AutoChargeNode : public rclcpp::Node {
public:
  explicit AutoChargeNode();
  ~AutoChargeNode() override;

  // 获取相机控制器
  CameraController *GetCameraController() const {
    return camera_controller_.get();
  }

private:
  // 相机控制器
  std::unique_ptr<CameraController> camera_controller_;
};

} // namespace auto_charge

#endif // AUTO_CHARGE_NODE_H
