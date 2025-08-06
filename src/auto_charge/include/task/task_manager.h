#pragma once

#include <jaka_sdk/jktypes.h>

class TaskManager {
public:
  TaskManager();
  ~TaskManager();

  void Charge();    // 充电
  void ChargeEnd(); // 充电结束

private:
  void setupSence(); // 加载固定场景信息
  void findBox();    // 查找充电箱位置 & 将充电箱构建到 Sence 中

  void moveToPose(const CartesianPose &pose);
};