# pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <memory>
#include <mutex>

namespace jaka_manipulation
{

struct SceneObject
{
    std::string id;
    std::string type;  // "box", "cylinder", "sphere"
    std::vector<double> dimensions;
    geometry_msgs::msg::Pose pose;
    std_msgs::msg::ColorRGBA color;
    std::string mesh_file;  // 可选，用于复杂形状
};

class SceneManager
{
public:
    // 单例获取方法
    static SceneManager* GetInstance();

    // 场景管理
    bool Save(const std::string& filename);
    bool Load(const std::string& filename);
    bool ClearScene();

    // 对象管理
    bool AddObject(const SceneObject& object);
    bool RemoveObject(const std::string& object_id);
    bool UpdateObject(const SceneObject& object);
    std::vector<std::string> GetObjectIds() const;

    // 预定义对象
    bool AddGround(const geometry_msgs::msg::Pose& pose = geometry_msgs::msg::Pose());
    bool AddTable(const geometry_msgs::msg::Pose& pose = geometry_msgs::msg::Pose());
    bool AddChargingStation(const geometry_msgs::msg::Pose& pose = geometry_msgs::msg::Pose());
    bool AddBox(const std::string& id, const geometry_msgs::msg::Pose& pose, 
                const std::vector<double>& dimensions = {0.1, 0.1, 0.1});
    bool AddStlObject(const std::string& id, const std::string& stl_file, 
                      const geometry_msgs::msg::Pose& pose);

    // 工具函数
    geometry_msgs::msg::Pose CreatePose(double x, double y, double z, 
                                        double qx = 0.0, double qy = 0.0, 
                                        double qz = 0.0, double qw = 1.0);
    std_msgs::msg::ColorRGBA CreateColor(double r, double g, double b, double a = 1.0);

    // 设置参数
    void SetFrameId(const std::string& frame_id) { frame_id_ = frame_id; }
    void SetDefaultScenePath(const std::string& path) { default_scene_path_ = path; }

    void LoadDefaultScene();

    // 禁用拷贝构造和赋值操作符
    SceneManager(const SceneManager&) = delete;
    SceneManager& operator=(const SceneManager&) = delete;

private:
    // 私有构造函数和析构函数
    SceneManager();
    ~SceneManager() = default;

    // 转换函数
    moveit_msgs::msg::CollisionObject createCollisionObject(const SceneObject& object);
    SceneObject collisionObjectToSceneObject(const moveit_msgs::msg::CollisionObject& collision_obj);
    
    // JSON 处理
    nlohmann::json sceneObjectToJson(const SceneObject& object);
    SceneObject jsonToSceneObject(const nlohmann::json& json);
    
    // 形状处理
    shape_msgs::msg::SolidPrimitive createPrimitive(const std::string& type, 
                                                    const std::vector<double>& dimensions);
    
    std::string getDefaultScenePath();
    
    // 静态成员变量
    static SceneManager* instance_;
    static std::mutex mutex_;
    
    // 成员变量
    rclcpp::Logger logger_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::string frame_id_;
    std::string default_scene_path_;
    std::vector<SceneObject> current_objects_;
    
    // 实例级别的互斥锁，用于线程安全操作
    mutable std::mutex instance_mutex_;
};

} // namespace jaka_manipulation
