#include "jaka_manipulation/scene_manager.h"
#include <fstream>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometric_shapes/mesh_operations.h> // For createMeshFromResource
#include <geometric_shapes/shape_operations.h> // For constructMsgFromShape
#include <boost/variant/get.hpp>

namespace jaka_manipulation
{

// 静态成员变量初始化
SceneManager* SceneManager::GetInstance()
{
    static SceneManager instance;
    return &instance;
}

SceneManager::SceneManager()
    : logger_(rclcpp::get_logger("scene_manager")), frame_id_("world")
{
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    RCLCPP_INFO(logger_, "场景管理器已初始化");
}

bool SceneManager::Save(const std::string& filename)
{
    std::lock_guard<std::mutex> lock(instance_mutex_);
    try {
        nlohmann::json scene_json;
        scene_json["scene_name"] = "saved_scene";
        scene_json["frame_id"] = frame_id_;
        scene_json["timestamp"] = std::time(nullptr);
        
        // 获取当前场景中的所有对象
        nlohmann::json objects_json = nlohmann::json::array();
        for (const auto& obj : current_objects_) {
            objects_json.push_back(sceneObjectToJson(obj));
        }
        
        scene_json["objects"] = objects_json;
        
        // 保存到文件
        std::ofstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(logger_, "无法创建场景文件: %s", filename.c_str());
            return false;
        }
        
        file << scene_json.dump(4);
        file.close();
        
        RCLCPP_INFO(logger_, "场景已保存到: %s", filename.c_str());
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "保存场景时发生错误: %s", e.what());
        return false;
    }
}

bool SceneManager::Load(const std::string& filename)
{
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(logger_, "无法打开场景文件: %s", filename.c_str());
            return false;
        }
        
        nlohmann::json scene_json;
        file >> scene_json;
        file.close();
        
        // 清除当前场景
        ClearScene();
        
        // 设置frame_id
        if (scene_json.contains("frame_id")) {
            frame_id_ = scene_json["frame_id"];
        }
        
        // 加载对象
        if (scene_json.contains("objects")) {
            for (const auto& obj_json : scene_json["objects"]) {
                SceneObject obj = jsonToSceneObject(obj_json);
                AddObject(obj);
            }
        }
        
        RCLCPP_INFO(logger_, "场景已从文件加载: %s", filename.c_str());
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "加载场景时发生错误: %s", e.what());
        return false;
    }
}

bool SceneManager::ClearScene()
{
    std::vector<std::string> object_ids = planning_scene_interface_->getKnownObjectNames();
    
    if (!object_ids.empty()) {
        planning_scene_interface_->removeCollisionObjects(object_ids);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    
    current_objects_.clear();
    RCLCPP_INFO(logger_, "场景已清除");
    return true;
}

bool SceneManager::AddObject(const SceneObject& object)
{
    std::lock_guard<std::mutex> lock(instance_mutex_);
    try {
        moveit_msgs::msg::CollisionObject collision_obj = createCollisionObject(object);
        
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(collision_obj);
        
        planning_scene_interface_->addCollisionObjects(collision_objects);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        
        // 添加到当前对象列表
        current_objects_.push_back(object);
        
        RCLCPP_INFO(logger_, "已添加对象: %s", object.id.c_str());
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "添加对象时发生错误: %s", e.what());
        return false;
    }
}

bool SceneManager::RemoveObject(const std::string& object_id)
{
    std::lock_guard<std::mutex> lock(instance_mutex_);
    std::vector<std::string> object_ids = {object_id};
    planning_scene_interface_->removeCollisionObjects(object_ids);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    
    // 从当前对象列表中移除
    current_objects_.erase(
        std::remove_if(current_objects_.begin(), current_objects_.end(),
                      [&object_id](const SceneObject& obj) { return obj.id == object_id; }),
        current_objects_.end());
    
    RCLCPP_INFO(logger_, "已移除对象: %s", object_id.c_str());
    return true;
}

bool SceneManager::UpdateObject(const SceneObject& object)
{
    std::lock_guard<std::mutex> lock(instance_mutex_);
    // 先移除旧对象，再添加新对象
    RemoveObject(object.id);
    return AddObject(object);
}

std::vector<std::string> SceneManager::GetObjectIds() const
{
    std::lock_guard<std::mutex> lock(instance_mutex_);
    std::vector<std::string> ids;
    for (const auto& obj : current_objects_) {
        ids.push_back(obj.id);
    }
    return ids;
}

bool SceneManager::AddGround(const geometry_msgs::msg::Pose& pose)
{
    SceneObject ground;
    ground.id = "ground";
    ground.type = "box";
    ground.dimensions = {5.0, 5.0, 0.1};
    ground.pose = pose.position.x == 0 && pose.position.y == 0 && pose.position.z == 0 ? 
                  CreatePose(0.0, 0.0, -0.05) : pose;
    ground.color = CreateColor(0.5, 0.5, 0.5, 1.0);
    
    return AddObject(ground);
}

bool SceneManager::AddTable(const geometry_msgs::msg::Pose& pose)
{
    SceneObject table;
    table.id = "table";
    table.type = "box";
    table.dimensions = {1.2, 0.8, 0.05};
    table.pose = pose.position.x == 0 && pose.position.y == 0 && pose.position.z == 0 ? 
                 CreatePose(0.6, 0.0, 0.4) : pose;
    table.color = CreateColor(0.8, 0.6, 0.4, 1.0);
    
    return AddObject(table);
}

bool SceneManager::AddChargingStation(const geometry_msgs::msg::Pose& pose)
{
    SceneObject charging_station;
    charging_station.id = "charging_station";
    charging_station.type = "box";
    charging_station.dimensions = {0.5, 0.3, 0.8};
    charging_station.pose = pose.position.x == 0 && pose.position.y == 0 && pose.position.z == 0 ? 
                           CreatePose(-0.5, 0.5, 0.4) : pose;
    charging_station.color = CreateColor(0.2, 0.8, 0.2, 1.0);
    
    return AddObject(charging_station);
}

bool SceneManager::AddBox(const std::string& id, const geometry_msgs::msg::Pose& pose, 
                         const std::vector<double>& dimensions)
{
    SceneObject box;
    box.id = id;
    box.type = "box";
    box.dimensions = dimensions;
    box.pose = pose;
    box.color = CreateColor(0.8, 0.2, 0.2, 1.0);
    
    return AddObject(box);
}

bool SceneManager::AddStlObject(const std::string& id, const std::string& stl_file, 
                               const geometry_msgs::msg::Pose& pose)
{
    SceneObject stl_obj;
    stl_obj.id = id;
    stl_obj.type = "mesh";
    stl_obj.mesh_file = stl_file;
    stl_obj.pose = pose;
    stl_obj.color = CreateColor(0.7, 0.7, 0.7, 1.0);
    // STL文件不需要dimensions，但为了兼容性设置默认值
    stl_obj.dimensions = {1.0, 1.0, 1.0};
    
    return AddObject(stl_obj);
}

geometry_msgs::msg::Pose SceneManager::CreatePose(double x, double y, double z, 
                                                  double qx, double qy, double qz, double qw)
{
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

std_msgs::msg::ColorRGBA SceneManager::CreateColor(double r, double g, double b, double a)
{
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

void SceneManager::LoadDefaultScene(){
    RCLCPP_INFO(logger_, "加载默认场景");
    const auto sence_config_path = getDefaultScenePath();
    RCLCPP_INFO(logger_, "场景文件路径: %s", sence_config_path.c_str());
    Load(sence_config_path);
}

// 私有方法实现
moveit_msgs::msg::CollisionObject SceneManager::createCollisionObject(const SceneObject& object)
{
    moveit_msgs::msg::CollisionObject collision_obj;
    collision_obj.header.frame_id = frame_id_;
    collision_obj.id = object.id;
    collision_obj.operation = collision_obj.ADD;
    
    // 如果指定了mesh文件，使用mesh
    if (!object.mesh_file.empty()) {
        // STL文件加载为mesh
        auto *mesh = shapes::createMeshFromResource(object.mesh_file);
        if (mesh != nullptr) {
            // 单位转换：mm -> m
            for (unsigned int i = 0; i < mesh->vertex_count; ++i) {
                mesh->vertices[i * 3 + 0] /= 1000.0;
                mesh->vertices[i * 3 + 1] /= 1000.0;
                mesh->vertices[i * 3 + 2] /= 1000.0;
            }
            shape_msgs::msg::Mesh mesh_msg;
            shapes::ShapeMsg shape_msg;
            shapes::constructMsgFromShape(mesh, shape_msg);
            collision_obj.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));
            collision_obj.mesh_poses.push_back(object.pose);
            delete mesh;
            RCLCPP_INFO(logger_, "STL文件已作为mesh加载(单位mm->m): %s", object.mesh_file.c_str());
        } else {
            RCLCPP_ERROR(logger_, "无法加载STL mesh: %s", object.mesh_file.c_str());
        }
    } else {
        // 使用基本几何形状
        shape_msgs::msg::SolidPrimitive primitive = createPrimitive(object.type, object.dimensions);
        collision_obj.primitives.push_back(primitive);
        collision_obj.primitive_poses.push_back(object.pose);
    }
    
    return collision_obj;
}

nlohmann::json SceneManager::sceneObjectToJson(const SceneObject& object)
{
    nlohmann::json obj_json;
    obj_json["id"] = object.id;
    obj_json["type"] = object.type;
    obj_json["dimensions"] = object.dimensions;
    
    obj_json["pose"]["position"] = {
        object.pose.position.x,
        object.pose.position.y,
        object.pose.position.z
    };
    
    obj_json["pose"]["orientation"] = {
        object.pose.orientation.x,
        object.pose.orientation.y,
        object.pose.orientation.z,
        object.pose.orientation.w
    };
    
    obj_json["color"] = {
        object.color.r,
        object.color.g,
        object.color.b,
        object.color.a
    };
    
    if (!object.mesh_file.empty()) {
        obj_json["mesh_file"] = object.mesh_file;
    }
    
    return obj_json;
}

SceneObject SceneManager::jsonToSceneObject(const nlohmann::json& json)
{
    SceneObject object;
    object.id = json["id"];
    object.type = json["type"];
    object.dimensions = json["dimensions"].get<std::vector<double>>();
    
    object.pose.position.x = json["pose"]["position"][0];
    object.pose.position.y = json["pose"]["position"][1];
    object.pose.position.z = json["pose"]["position"][2];
    
    object.pose.orientation.x = json["pose"]["orientation"][0];
    object.pose.orientation.y = json["pose"]["orientation"][1];
    object.pose.orientation.z = json["pose"]["orientation"][2];
    object.pose.orientation.w = json["pose"]["orientation"][3];
    
    object.color.r = json["color"][0];
    object.color.g = json["color"][1];
    object.color.b = json["color"][2];
    object.color.a = json["color"][3];
    
    if (json.contains("mesh_file")) {
        object.mesh_file = json["mesh_file"];
    }
    
    return object;
}

shape_msgs::msg::SolidPrimitive SceneManager::createPrimitive(const std::string& type, 
                                                             const std::vector<double>& dimensions)
{
    shape_msgs::msg::SolidPrimitive primitive;
    
    if (type == "box") {
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = dimensions[0];  // 长度
        primitive.dimensions[1] = dimensions[1];  // 宽度
        primitive.dimensions[2] = dimensions[2];  // 高度
    } else if (type == "cylinder") {
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = dimensions[0];  // 高度
        primitive.dimensions[1] = dimensions[1];  // 半径
    } else if (type == "sphere") {
        primitive.type = primitive.SPHERE;
        primitive.dimensions.resize(1);
        primitive.dimensions[0] = dimensions[0];  // 半径
    }
    
    return primitive;
}

std::string SceneManager::getDefaultScenePath(){
    // 获取包的安装路径
    std::string default_scene_path;
    try {
        std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("jaka_manipulation");
        std::string scenes_dir = pkg_share_dir + "/config/scene";
        
        // 确保scenes目录存在
        std::filesystem::create_directories(scenes_dir);
        
        // 设置默认场景文件路径
        default_scene_path = scenes_dir + "/default_scene.json";
        
        RCLCPP_INFO(logger_, "场景文件目录设置为: %s", scenes_dir.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "无法获取包路径: %s", e.what());
        // 使用当前工作目录作为备用
        default_scene_path = "./config/scene/default_scene.json";
        std::filesystem::create_directories("./config/scene");
    }
    return default_scene_path;
}
} // namespace jaka_manipulation
