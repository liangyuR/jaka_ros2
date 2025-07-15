#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <memory>
#include <functional>
#include <chrono>

class ManipulationNode : public rclcpp::Node
{
public:
    ManipulationNode() : Node("manipulation_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("planning_group", "jaka_zu12");
        planning_group_ = this->get_parameter("planning_group").as_string();

        // 初始化MoveIt接口
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), planning_group_);
        
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // 创建服务
        add_table_service_ = this->create_service<std_srvs::srv::SetBool>(
            "add_table", std::bind(&ManipulationNode::add_table_callback, this, 
            std::placeholders::_1, std::placeholders::_2));

        add_box_service_ = this->create_service<std_srvs::srv::SetBool>(
            "add_box", std::bind(&ManipulationNode::add_box_callback, this, 
            std::placeholders::_1, std::placeholders::_2));

        clear_scene_service_ = this->create_service<std_srvs::srv::SetBool>(
            "clear_scene", std::bind(&ManipulationNode::clear_scene_callback, this, 
            std::placeholders::_1, std::placeholders::_2));

        attach_tool_service_ = this->create_service<std_srvs::srv::SetBool>(
            "attach_tool", std::bind(&ManipulationNode::attach_tool_callback, this, 
            std::placeholders::_1, std::placeholders::_2));

        detach_tool_service_ = this->create_service<std_srvs::srv::SetBool>(
            "detach_tool", std::bind(&ManipulationNode::detach_tool_callback, this, 
            std::placeholders::_1, std::placeholders::_2));

        // 等待MoveIt准备就绪
        auto timer = this->create_wall_timer(
            std::chrono::seconds(1), 
            std::bind(&ManipulationNode::initialization_timer, this));
        timer_ = timer;

        RCLCPP_INFO(this->get_logger(), "Manipulation node initialized");
    }

private:
    void initialization_timer()
    {
        static int count = 0;
        count++;
        
        if (count >= 3) {
            timer_->cancel();
            add_default_objects();
            RCLCPP_INFO(this->get_logger(), "Initialization complete. Services available:");
            RCLCPP_INFO(this->get_logger(), "  - /add_table");
            RCLCPP_INFO(this->get_logger(), "  - /add_box");
            RCLCPP_INFO(this->get_logger(), "  - /clear_scene");
            RCLCPP_INFO(this->get_logger(), "  - /attach_tool");
            RCLCPP_INFO(this->get_logger(), "  - /detach_tool");
        }
    }

    void add_default_objects()
    {
        // 添加默认的地面和桌子
        add_table(true);
        RCLCPP_INFO(this->get_logger(), "Added default table to scene");
    }

    void add_table_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        bool success = add_table(request->data);
        response->success = success;
        response->message = success ? "Table added successfully" : "Failed to add table";
    }

    void add_box_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        bool success = add_box(request->data);
        response->success = success;
        response->message = success ? "Box added successfully" : "Failed to add box";
    }

    void clear_scene_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        bool success = clear_scene();
        response->success = success;
        response->message = success ? "Scene cleared successfully" : "Failed to clear scene";
    }

    void attach_tool_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        bool success = attach_tool();
        response->success = success;
        response->message = success ? "Tool attached successfully" : "Failed to attach tool";
    }

    void detach_tool_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        bool success = detach_tool();
        response->success = success;
        response->message = success ? "Tool detached successfully" : "Failed to detach tool";
    }

    bool add_table(bool add = true)
    {
        if (!add) {
            std::vector<std::string> object_ids = {"table"};
            planning_scene_interface_->removeCollisionObjects(object_ids);
            return true;
        }

        // 创建桌子物体
        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = move_group_->getPlanningFrame();
        table.id = "table";

        // 定义桌子的形状（长方体）
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 1.2;  // 长度
        primitive.dimensions[1] = 0.8;  // 宽度
        primitive.dimensions[2] = 0.05; // 高度

        // 定义桌子的位置
        geometry_msgs::msg::Pose table_pose;
        table_pose.orientation.w = 1.0;
        table_pose.position.x = 0.6;
        table_pose.position.y = 0.0;
        table_pose.position.z = 0.4;  // 桌面高度

        table.primitives.push_back(primitive);
        table.primitive_poses.push_back(table_pose);
        table.operation = table.ADD;

        // 添加到场景
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(table);
        planning_scene_interface_->addCollisionObjects(collision_objects);

        // 等待应用
        rclcpp::sleep_for(std::chrono::milliseconds(100));

        RCLCPP_INFO(this->get_logger(), "Added table to planning scene");
        return true;
    }

    bool add_box(bool add = true)
    {
        if (!add) {
            std::vector<std::string> object_ids = {"box"};
            planning_scene_interface_->removeCollisionObjects(object_ids);
            return true;
        }

        // 创建盒子物体
        moveit_msgs::msg::CollisionObject box;
        box.header.frame_id = move_group_->getPlanningFrame();
        box.id = "box";

        // 定义盒子的形状
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.1;  // 长度
        primitive.dimensions[1] = 0.1;  // 宽度
        primitive.dimensions[2] = 0.1;  // 高度

        // 定义盒子的位置（在桌面上）
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.6;
        box_pose.position.y = 0.2;
        box_pose.position.z = 0.45 + 0.05;  // 桌面高度 + 盒子高度的一半

        box.primitives.push_back(primitive);
        box.primitive_poses.push_back(box_pose);
        box.operation = box.ADD;

        // 添加到场景
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(box);
        planning_scene_interface_->addCollisionObjects(collision_objects);

        // 等待应用
        rclcpp::sleep_for(std::chrono::milliseconds(100));

        RCLCPP_INFO(this->get_logger(), "Added box to planning scene");
        return true;
    }

    bool clear_scene()
    {
        // 获取所有物体ID
        std::vector<std::string> object_ids = planning_scene_interface_->getKnownObjectNames();
        
        if (!object_ids.empty()) {
            planning_scene_interface_->removeCollisionObjects(object_ids);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            RCLCPP_INFO(this->get_logger(), "Cleared all objects from planning scene");
        }
        
        return true;
    }

    bool attach_tool()
    {
        if (tool_attached_) {
            RCLCPP_INFO(this->get_logger(), "Tool is already attached");
            return true;
        }

        // 创建工具物体
        moveit_msgs::msg::CollisionObject tool;
        tool.header.frame_id = move_group_->getPlanningFrame();
        tool.id = "gripper_tool";

        // 定义工具底座
        shape_msgs::msg::SolidPrimitive tool_base;
        tool_base.type = tool_base.BOX;
        tool_base.dimensions.resize(3);
        tool_base.dimensions[0] = 0.05;  // 长度
        tool_base.dimensions[1] = 0.05;  // 宽度
        tool_base.dimensions[2] = 0.02;  // 高度

        // 工具底座位置 - 相对于Link_6
        geometry_msgs::msg::Pose tool_pose;
        tool_pose.orientation.w = 1.0;
        tool_pose.position.x = 0.0;
        tool_pose.position.y = 0.0;
        tool_pose.position.z = 0.02;

        tool.primitives.push_back(tool_base);
        tool.primitive_poses.push_back(tool_pose);
        tool.operation = tool.ADD;

        // 首先添加工具到场景
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(tool);
        planning_scene_interface_->addCollisionObjects(collision_objects);
        
        // 等待添加完成
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        
        // 更新工具状态
        tool_attached_ = true;
        
        // 设置末端效应器链接 (工具附加后的新TCP位置)
        move_group_->setEndEffectorLink("Link_6");

        RCLCPP_INFO(this->get_logger(), "Tool attached successfully");
        return true;
    }

    bool detach_tool()
    {
        if (!tool_attached_) {
            RCLCPP_INFO(this->get_logger(), "No tool is attached");
            return true;
        }

        // 从机械臂卸载工具
        planning_scene_interface_->removeCollisionObjects({"gripper_tool"});
        
        // 更新工具状态
        tool_attached_ = false;

        RCLCPP_INFO(this->get_logger(), "Tool detached successfully");
        return true;
    }

    // 成员变量
    std::string planning_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    
    // 服务
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr add_table_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr add_box_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr clear_scene_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr attach_tool_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr detach_tool_service_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 工具状态
    bool tool_attached_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}