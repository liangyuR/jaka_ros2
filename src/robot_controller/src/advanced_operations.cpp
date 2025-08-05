#include "robot_controller/advanced_operations.h"
#include <jaka_sdk/jktypes.h>

namespace robot_controller {

AdvancedOperations::AdvancedOperations(JAKAZuRobot &robot,
                                       rclcpp::Logger logger)
    : robot_(robot), logger_(logger) {
  RCLCPP_INFO(logger_, "AdvancedOperations initialized");
}

void AdvancedOperations::InitStatus(const INIStatus &status) {
  switch (status) {
  case kInsert:
    break;
  case kPull:
    break;
  case kConnect:
    break;
  case kRelease:
    break;
  default:
    RCLCPP_ERROR(logger_, "Invalid status: %d", status);
    break;
  }
}

bool AdvancedOperations::InsertGun() {
  RCLCPP_INFO(logger_, "Starting gun insertion operation");

  try {
    InitStatus(kInsert);
    const auto ret = robot_.get_tcp_position(&reference_pose_);
    if (ret != ERR_SUCC) {
      RCLCPP_ERROR(logger_, "Failed to get TCP position, error code: %d", ret);
      return false;
    }

    if (!executeForceMotion(reference_pose_, 100, {}, {})) {
      RCLCPP_ERROR(logger_, "Failed to execute force motion");
      return false;
    }

    int attempt = 0;
    while (attempt < 3) {
      if (setForce({{0, 1, 0, 0, 0, 0}})) {
        break;
      }
      // linemove
      CartesianPose move_offset = {0, 0, 20, 0, 0, 0};
      robot_.linear_move(&move_offset, MoveMode::ABS, true, 15, 0.1, 0, nullptr,
                         3.14, 12.56);

      if (checkMotionStatus(45, 5)) {
        reference_pose_ = {0, 0, 0, 0, 0, 0};
        break;
      }
      attempt++;
    }

    RCLCPP_INFO(logger_, "Gun insertion completed successfully");
    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Exception during gun insertion: %s", e.what());
    return false;
  }
}

bool AdvancedOperations::PullGun() {
  RCLCPP_INFO(logger_, "Starting gun pulling operation");

  try {
    // 设置IO
    if (!setIO(1, false)) {
      RCLCPP_ERROR(logger_, "Failed to set digital output for gun pulling");
      return false;
    }

    // 执行运动
    RCLCPP_DEBUG(logger_, "Moving to pulling position");
    // ... 运动逻辑

    RCLCPP_INFO(logger_, "Gun pulling completed successfully");
    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Exception during gun pulling: %s", e.what());
    return false;
  }
}

bool AdvancedOperations::ConnectGun() {
  RCLCPP_INFO(logger_, "Starting gun connection operation");

  try {
    // 设置IO
    if (!setIO(2, true)) {
      RCLCPP_ERROR(logger_, "Failed to set digital output for gun connection");
      return false;
    }

    // 执行运动
    RCLCPP_DEBUG(logger_, "Moving to connection position");
    // ... 运动逻辑

    RCLCPP_INFO(logger_, "Gun connection completed successfully");
    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Exception during gun connection: %s", e.what());
    return false;
  }
}

bool AdvancedOperations::setIO(int pin, bool value) {
  RCLCPP_DEBUG(logger_, "Setting IO pin %d to %s", pin,
               value ? "true" : "false");

  errno_t result = robot_.set_digital_output(IO_TOOL, pin, value);
  if (result != ERR_SUCC) {
    RCLCPP_ERROR(logger_, "Failed to set IO pin %d, error code: %d", pin,
                 result);
    return false;
  }

  return true;
}

bool AdvancedOperations::getIO(int pin) {
  RCLCPP_DEBUG(logger_, "Getting IO pin %d value", pin);

  BOOL value;
  errno_t result = robot_.get_digital_input(IO_TOOL, pin, &value);
  if (result != ERR_SUCC) {
    RCLCPP_ERROR(logger_, "Failed to get IO pin %d, error code: %d", pin,
                 result);
    return false;
  }

  return value;
}

bool AdvancedOperations::setForce(const AdmitCtrlType &admit_ctrl) {
  RCLCPP_DEBUG(logger_,
               "Setting force control config - axis: %d, opt: %d, ft_user: "
               "%.2f, ft_rebound: %.2f, ft_constant: %.2f",
               admit_ctrl.axis, admit_ctrl.opt, admit_ctrl.ft_user,
               admit_ctrl.ft_rebound, admit_ctrl.ft_constant);

  errno_t result = robot_.set_ft_ctrl_config(admit_ctrl);
  if (result != ERR_SUCC) {
    RCLCPP_ERROR(
        logger_,
        "Failed to set force control config for axis %d, error code: %d",
        admit_ctrl.axis, result);
    return false;
  }

  return true;
}

bool AdvancedOperations::setForce(
    const std::vector<AdmitCtrlType> &admit_ctrl) {
  if (admit_ctrl.empty()) {
    RCLCPP_ERROR(logger_, "Empty force control configuration vector");
    return false;
  }

  RCLCPP_INFO(logger_, "Setting force control config for %zu axes",
              admit_ctrl.size());

  // 分别设置每个轴的力控配置，调用单轴设置方法
  for (const auto &ctrl : admit_ctrl) {
    if (!setForce(ctrl)) {
      RCLCPP_ERROR(logger_, "Failed to set force control config for axis %d",
                   ctrl.axis);
      return false;
    }
  }

  RCLCPP_INFO(logger_, "Force control config set successfully for %zu axes",
              admit_ctrl.size());
  return true;
}

bool AdvancedOperations::enableForce(bool enable) {
  RCLCPP_INFO(logger_, "%s force control", enable ? "Enabling" : "Disabling");

  errno_t result = robot_.set_ft_ctrl_mode(enable); // NOLINT
  if (result != ERR_SUCC) {
    RCLCPP_ERROR(logger_, "Failed to %s force control, error code: %d",
                 enable ? "enable" : "disable", result);
    return false;
  }

  RCLCPP_INFO(logger_, "Force control %s successfully",
              enable ? "enabled" : "disabled");
  return true;
}

bool AdvancedOperations::enableForceStop(const ForceStopConditionList &force) {
  RCLCPP_INFO(logger_, "Setting terminate force condition");

  errno_t result = robot_.set_force_stop_condition(force);
  if (result != ERR_SUCC) {
    RCLCPP_ERROR(logger_, "Failed to set terminate force, error code: %d",
                 result);
    return false;
  }

  RCLCPP_INFO(logger_, "Terminate force set successfully");
  return true;
}
} // namespace robot_controller