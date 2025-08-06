#include "auto_charge/scene_manager.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace auto_charge {

// 静态成员变量初始化
SceneManager *SceneManager::GetInstance() {
  static SceneManager instance;
  return &instance;
}

SceneManager::SceneManager()
    : logger_(rclcpp::get_logger("scene_manager")), frame_id_("world") {
  planning_scene_interface_ =
      std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  RCLCPP_INFO(logger_, "场景管理器已初始化");
}

bool SceneManager::ClearScene() {
  std::lock_guard<std::mutex> lock(instance_mutex_);
  std::vector<std::string> object_ids =
      planning_scene_interface_->getKnownObjectNames();

  if (!object_ids.empty()) {
    planning_scene_interface_->removeCollisionObjects(object_ids);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  current_objects_.clear();
  RCLCPP_INFO(logger_, "场景已清除");
  return true;
}

bool SceneManager::AddObject(const SceneObject &object) {
  std::lock_guard<std::mutex> lock(instance_mutex_);
  try {
    moveit_msgs::msg::CollisionObject collision_obj =
        CreateCollisionObject_(object);

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_obj);

    planning_scene_interface_->addCollisionObjects(collision_objects);
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    // 添加到当前对象列表
    current_objects_.push_back(object);

    RCLCPP_INFO(logger_, "已添加对象: %s", object.id.c_str());
    return true;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "添加对象时发生错误: %s", e.what());
    return false;
  }
}

bool SceneManager::RemoveObject(const std::string &object_id) {
  std::lock_guard<std::mutex> lock(instance_mutex_);
  std::vector<std::string> object_ids = {object_id};
  planning_scene_interface_->removeCollisionObjects(object_ids);
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // 从当前对象列表中移除
  current_objects_.erase(std::remove_if(current_objects_.begin(),
                                        current_objects_.end(),
                                        [&object_id](const SceneObject &obj) {
                                          return obj.id == object_id;
                                        }),
                         current_objects_.end());

  RCLCPP_INFO(logger_, "已移除对象: %s", object_id.c_str());
  return true;
}

bool SceneManager::UpdateObject(const SceneObject &object) {
  std::lock_guard<std::mutex> lock(instance_mutex_);
  // 先移除旧对象，再添加新对象
  RemoveObject(object.id);
  return AddObject(object);
}

std::vector<std::string> SceneManager::GetObjectIds() const {
  std::lock_guard<std::mutex> lock(instance_mutex_);
  std::vector<std::string> ids;
  for (const auto &obj : current_objects_) {
    ids.push_back(obj.id);
  }
  return ids;
}

bool SceneManager::AddGround(const geometry_msgs::msg::Pose &pose) {
  SceneObject ground;
  ground.id = "ground";
  ground.type = "box";
  ground.dimensions = {10.0, 10.0, 0.01}; // 10m x 10m x 1cm
  ground.pose = pose;
  ground.color = CreateColor(0.5, 0.5, 0.5, 0.8); // 灰色

  return AddObject(ground);
}

bool SceneManager::AddTable(const geometry_msgs::msg::Pose &pose) {
  SceneObject table;
  table.id = "table";
  table.type = "box";
  table.dimensions = {1.0, 0.6, 0.7}; // 1m x 0.6m x 0.7m
  table.pose = pose;
  table.color = CreateColor(0.8, 0.6, 0.4, 1.0); // 棕色

  return AddObject(table);
}

bool SceneManager::AddChargingStation(const geometry_msgs::msg::Pose &pose) {
  SceneObject charging_station;
  charging_station.id = "charging_station";
  charging_station.type = "box";
  charging_station.dimensions = {0.3, 0.2, 0.4}; // 30cm x 20cm x 40cm
  charging_station.pose = pose;
  charging_station.color = CreateColor(0.2, 0.8, 0.2, 1.0); // 绿色

  return AddObject(charging_station);
}

bool SceneManager::AddBox(const std::string &id,
                          const geometry_msgs::msg::Pose &pose,
                          const std::vector<double> &dimensions) {
  SceneObject box;
  box.id = id;
  box.type = "box";
  box.dimensions = dimensions;
  box.pose = pose;
  box.color = CreateColor(0.8, 0.8, 0.8, 1.0); // 白色

  return AddObject(box);
}

geometry_msgs::msg::Pose SceneManager::CreatePose(double x, double y, double z,
                                                  double qx, double qy,
                                                  double qz, double qw) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
  pose.orientation.w = qw;
  return pose;
}

std_msgs::msg::ColorRGBA SceneManager::CreateColor(double r, double g, double b,
                                                   double a) {
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

void SceneManager::LoadDefaultScene() {
  RCLCPP_INFO(logger_, "加载默认场景");

  // 清除现有场景
  ClearScene();

  // 设置工作空间

  // 添加充电站
  geometry_msgs::msg::Pose charging_pose = CreatePose(0.8, 0.0, 0.2);

  AddChargingStation(charging_pose);

  RCLCPP_INFO(logger_, "默认场景加载完成");
}

moveit_msgs::msg::CollisionObject
SceneManager::CreateCollisionObject_(const SceneObject &object) {
  moveit_msgs::msg::CollisionObject collision_obj;
  collision_obj.header.frame_id = frame_id_;
  collision_obj.id = object.id;
  collision_obj.operation = collision_obj.ADD;

  // 使用基本几何形状
  shape_msgs::msg::SolidPrimitive primitive =
      CreatePrimitive_(object.type, object.dimensions);
  collision_obj.primitives.push_back(primitive);
  collision_obj.primitive_poses.push_back(object.pose);

  return collision_obj;
}

shape_msgs::msg::SolidPrimitive
SceneManager::CreatePrimitive_(const std::string &type,
                               const std::vector<double> &dimensions) {
  shape_msgs::msg::SolidPrimitive primitive;

  if (type == "box") {
    primitive.type = primitive.BOX;
    if (dimensions.size() >= 3) {
      primitive.dimensions = {dimensions[0], dimensions[1], dimensions[2]};
    }
  } else if (type == "cylinder") {
    primitive.type = primitive.CYLINDER;
    if (dimensions.size() >= 2) {
      primitive.dimensions = {dimensions[0], dimensions[1]}; // radius, height
    }
  } else if (type == "sphere") {
    primitive.type = primitive.SPHERE;
    if (dimensions.size() >= 1) {
      primitive.dimensions = {dimensions[0]}; // radius
    }
  } else {
    // 默认使用盒子
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.1, 0.1, 0.1};
  }

  return primitive;
}

} // namespace auto_charge