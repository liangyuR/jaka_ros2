#pragma once

#include "jaka_sdk/JAKAZuRobot.h"
#include "jaka_sdk/jktypes.h"
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace robot_controller {
enum IO_PIN {
  kQuick = 19,
  KQuickCheck = 0,
  kButton = 20,
  kButtonCheck = 1,
};

enum INIStatus {
  kInsert,
  kPull,
  kConnect,
  kRelease,
};

class AdvancedOperations {
public:
  explicit AdvancedOperations(JAKAZuRobot &robot, rclcpp::Logger logger);

  // IO传感器等初始化
  void InitStatus(const INIStatus &status);

  // 枪在入口时调用，采用力控方式渐进插入，并自动检测是否插入到位
  bool InsertGun();

  // 枪在出口时调用，采用力控方式拉出，并自动检测是否拉出到位
  bool PullGun();

  // 连接枪时调用, 此时连接件应该与枪尾对齐, 会多次尝试连接, 直到连接成功,
  // 或者失败超过N.
  bool ConnectGun();

private:
  JAKAZuRobot &robot_;
  rclcpp::Logger logger_;
  CartesianPose reference_pose_; // 参考点位姿

  bool executeForceMotion(const CartesianPose &pose, int speed,
                          const std::vector<AdmitCtrlType> &admit_ctrl,
                          const ForceStopConditionList &force);
  bool checkMotionStatus(double offset, double tolerance);

  // IO控制
  bool setIO(int pin, bool value);
  bool getIO(int pin);

  // 柔性力控控制 - 重载函数
  bool setForce(const AdmitCtrlType &admit_ctrl);
  bool setForce(const std::vector<AdmitCtrlType> &admit_ctrl);
  bool enableForce(bool enable);

  // 力控终止运动
  bool enableForceStop(const ForceStopConditionList &force);
};
} // namespace robot_controller
