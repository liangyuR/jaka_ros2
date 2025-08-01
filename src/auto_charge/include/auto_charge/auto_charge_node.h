#ifndef AUTO_CHARGE_NODE_H
#define AUTO_CHARGE_NODE_H

#include "auto_charge/camera_controller.h"
#include "auto_charge/robot_manager.h"
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

  RobotManager *GetRobotManager() const { return robot_manager_.get(); }

private:
  // 相机控制器
  std::unique_ptr<CameraController> camera_controller_;

  // 机器人管理器
  std::unique_ptr<RobotManager> robot_manager_;
};

} // namespace auto_charge

#endif // AUTO_CHARGE_NODE_H
