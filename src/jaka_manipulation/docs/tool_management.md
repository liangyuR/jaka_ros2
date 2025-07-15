# JAKA 机械臂工具管理系统

## 概述

该工具管理系统允许您动态地给 JAKA 机械臂添加和移除工具，并在 MoveIt 中进行相应的配置。

## 组件结构

### 1. URDF 工具定义

- **文件**: `src/jaka_description/urdf/tools/gripper_tool.urdf.xacro`
- **作用**: 定义夹爪工具的物理结构，包括：
  - 工具底座
  - 左右夹爪
  - 工具中心点 (TCP)

### 2. 带工具的机械臂配置

- **文件**: `src/jaka_description/urdf/jaka_zu12_with_tool.urdf.xacro`
- **作用**: 包含原始机械臂和工具的完整配置

### 3. 工具管理节点

- **文件**: `src/jaka_manipulation/src/manipulation_node.cpp`
- **功能**: 提供工具管理的 ROS 2 服务

## 可用服务

### 基本物体管理

1. **添加桌子**
   ```bash
   ros2 service call /add_table std_srvs/srv/SetBool "data: true"
   ```

2. **添加盒子**
   ```bash
   ros2 service call /add_box std_srvs/srv/SetBool "data: true"
   ```

3. **清空场景**
   ```bash
   ros2 service call /clear_scene std_srvs/srv/SetBool "data: true"
   ```

### 工具管理

4. **附加工具**
   ```bash
   ros2 service call /attach_tool std_srvs/srv/SetBool "data: true"
   ```

5. **分离工具**
   ```bash
   ros2 service call /detach_tool std_srvs/srv/SetBool "data: true"
   ```

## 使用步骤

### 1. 启动系统

```bash
# 构建项目
cd ~/project/jaka_ros2
colcon build --packages-select jaka_manipulation

# 启动演示
ros2 launch jaka_manipulation manipulation_demo.launch.py
```

### 2. 测试工具管理

```bash
# 运行测试脚本
python3 src/jaka_manipulation/scripts/test_manipulation.py
```

### 3. 手动测试

```bash
# 附加工具
ros2 service call /attach_tool std_srvs/srv/SetBool "data: true"

# 查看工具是否出现在 RViz 中
# 工具会显示为灰色底座，连接到机械臂末端

# 分离工具
ros2 service call /detach_tool std_srvs/srv/SetBool "data: true"
```

## 工具特性

### 工具结构
- **底座**: 5cm x 5cm x 2cm 的立方体
- **夹爪**: 两个可动的夹爪，具有碰撞检测
- **TCP**: 工具中心点，位于底座上方 8cm 处

### 碰撞检测
- 工具会作为碰撞物体添加到 MoveIt 场景中
- 在路径规划时会避免与工具发生碰撞

### 状态管理
- 系统会跟踪工具的附加状态
- 防止重复附加或分离操作

## 扩展功能

### 添加新工具

1. 在 `src/jaka_description/urdf/tools/` 目录下创建新的工具定义
2. 在 `manipulation_node.cpp` 中添加对应的服务
3. 更新碰撞检测和 TCP 配置

### 工具控制

可以在 `manipulation_node.cpp` 中添加：
- 夹爪开合控制
- 工具状态监控
- 力反馈控制

## 故障排除

### 常见问题

1. **工具不显示**
   - 确保 RViz 中启用了 "Scene Geometry"
   - 检查工具是否正确添加到场景中

2. **服务调用失败**
   - 确保 `manipulation_node` 正在运行
   - 检查服务是否正确注册

3. **碰撞检测不工作**
   - 确保工具的碰撞几何体正确定义
   - 检查 MoveIt 场景更新是否成功

### 日志查看

```bash
# 查看manipulation节点日志
ros2 topic echo /rosout | grep manipulation_node
```

## 开发者信息

- **版本**: 1.0.0
- **兼容性**: ROS 2 Humble, MoveIt 2
- **维护**: 基于 JAKA SDK 2.2.2 