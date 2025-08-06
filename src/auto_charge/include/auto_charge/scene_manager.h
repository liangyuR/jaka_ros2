#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>
#include <vector>

namespace auto_charge {

struct SceneObject {
  std::string id;
  std::string type; // "box", "cylinder", "sphere"
  std::vector<double> dimensions;
  geometry_msgs::msg::Pose pose;
  std_msgs::msg::ColorRGBA color;
  std::string mesh_file; // 可选，用于复杂形状
};

class SceneManager {
public:
  // 单例获取方法
  static SceneManager *GetInstance();

  // 场景管理
  bool ClearScene();
  bool AddObject(const SceneObject &object);
  bool RemoveObject(const std::string &object_id);
  bool UpdateObject(const SceneObject &object);
  std::vector<std::string> GetObjectIds() const;

  // 预定义对象
  bool
  AddGround(const geometry_msgs::msg::Pose &pose = geometry_msgs::msg::Pose());
  bool
  AddTable(const geometry_msgs::msg::Pose &pose = geometry_msgs::msg::Pose());
  bool AddChargingStation(
      const geometry_msgs::msg::Pose &pose = geometry_msgs::msg::Pose());
  bool AddBox(const std::string &id, const geometry_msgs::msg::Pose &pose,
              const std::vector<double> &dimensions = {0.1, 0.1, 0.1});

  // 工具函数
  geometry_msgs::msg::Pose CreatePose(double x, double y, double z,
                                      double qx = 0.0, double qy = 0.0,
                                      double qz = 0.0, double qw = 1.0);
  std_msgs::msg::ColorRGBA CreateColor(double r, double g, double b,
                                       double a = 1.0);

  // 设置参数
  void SetFrameId(const std::string &frame_id) { frame_id_ = frame_id; }

  void LoadDefaultScene();

  // 禁用拷贝构造和赋值操作符
  SceneManager(const SceneManager &) = delete;
  SceneManager &operator=(const SceneManager &) = delete;

private:
  // 私有构造函数和析构函数
  SceneManager();
  ~SceneManager() = default;

  // 转换函数
  moveit_msgs::msg::CollisionObject
  CreateCollisionObject_(const SceneObject &object);

  // 形状处理
  shape_msgs::msg::SolidPrimitive
  CreatePrimitive_(const std::string &type,
                   const std::vector<double> &dimensions);

  // 成员变量
  rclcpp::Logger logger_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>
      planning_scene_interface_;
  std::string frame_id_;
  std::vector<SceneObject> current_objects_;

  // 实例级别的互斥锁，用于线程安全操作
  mutable std::mutex instance_mutex_;
};

} // namespace auto_charge