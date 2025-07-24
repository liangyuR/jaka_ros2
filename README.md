# JAKA ROS 2 驱动包

基于 **SDK 2.2.2** 版本的节卡（JAKA）协作机器人 ROS 2 驱动包，提供完整的机器人控制、路径规划和可视化功能。

## 系统要求

- **操作系统**: Ubuntu 22.04 (Jammy)
- **ROS 版本**: ROS 2 Humble
- **控制器版本**: 最低 1.7.1.46，推荐 1.7.2.16
- **硬件要求**: 
  - CPU: 64位 Intel i5/i7 或同等 AMD 处理器
  - 内存: 至少 8GB（推荐 16GB）
  - 存储: 至少 20GB 可用空间

## 项目架构

```
jaka_ros2/
├── src/
│   ├── jaka_driver/          # 核心驱动模块
│   ├── jaka_msgs/            # 消息和服务定义
│   ├── jaka_description/     # 机器人模型描述
│   ├── jaka_planner/         # MoveIt 2 运动规划
│   ├── jaka_manipulation/    # 高级操作任务
│   └── jaka_zu12_moveit_config/ # ZU12 型号 MoveIt 配置
├── build/                    # 构建文件
├── install/                  # 安装文件
└── log/                      # 构建日志
```

## 模块详细说明

### 1. jaka_driver - 核心驱动模块
**作用**: 项目的核心驱动模块，负责与 JAKA 机器人硬件的直接通信

**主要功能**:
- 通过 JAKA SDK 与机器人控制器建立连接
- 提供丰富的 ROS 2 服务接口（关节运动、直线运动、正逆运动学等）
- 实时发布机器人状态信息
- 处理机器人的安全和错误状态

**核心文件**:
- `jaka_driver.cpp` - 主要驱动程序（989行代码）
- `robot_start.launch.py` - 启动文件

### 2. jaka_msgs - 消息和服务定义
**作用**: 定义项目专用的消息和服务类型

**服务类型**:
- `Move` - 机器人运动控制
- `GetFK/GetIK` - 正逆运动学求解
- `SetIO/GetIO` - IO 端口控制
- `SetPayload` - 负载设置
- `ServoMove` - 伺服运动控制
- `ClearError` - 错误清除

**消息类型**:
- `RobotMsg` - 机器人状态消息

### 3. jaka_description - 机器人模型描述
**作用**: 包含机器人的物理模型和可视化资源

**主要内容**:
- URDF 文件：机器人的运动学和动力学模型
- 3D 网格文件：机器人各关节的 3D 模型
- 配置文件：机器人参数配置

**支持型号**: 当前包含 `jaka_zu12` 型号的描述文件

### 4. jaka_planner - 运动规划模块
**作用**: 提供基于 MoveIt 2 的运动规划功能

**主要功能**:
- 启动 MoveIt 2 服务器
- 处理轨迹规划和执行
- 与 `jaka_driver` 协同工作实现轨迹跟踪
- 支持碰撞检测和路径优化

**核心文件**:
- `moveit_server.cpp` - 实现 MoveIt 2 与 JAKA 硬件的桥接
- `moveit_server.launch.py` - MoveIt 2 服务器启动文件

### 5. jaka_manipulation - 操作任务模块
**作用**: 提供高级的机器人操作功能

**主要功能**:
- 自动充电相关的移动控制
- 感知和操作任务的集成
- 高级操作接口封装

**核心文件**:
- `manipulation_node.cpp` - 操作任务节点

### 6. jaka_zu12_moveit_config - 特定型号的MoveIt配置
**作用**: 专为 JAKA ZU12 型号提供的 MoveIt 2 配置

**主要内容**:
- 运动学求解器配置
- 规划器参数设置
- 碰撞检测配置
- RViz 可视化配置

## 安装和构建

### 1. 克隆仓库
```bash
# HTTPS 方式
git clone https://github.com/JakaCobot/jaka_ros2.git

# SSH 方式
git clone git@github.com:JakaCobot/jaka_ros2.git
```

### 2. 构建项目
```bash
cd jaka_ros2
colcon build
```

### 3. 设置环境
```bash
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 使用方式

### 基础驱动模式
启动 JAKA 驱动服务，替换 `<robot_ip>` 为实际机器人IP地址：
```bash
ros2 launch jaka_driver robot_start_launch.py ip:=<robot_ip>
```

### 服务调用示例

#### 关节运动
```bash
ros2 service call /jaka_driver/joint_move jaka_msgs/srv/Move \
  "{pose: [0.0, 1.57, -1.57, 1.57, 1.57, 0.0], has_ref: false, ref_joint: [0], mvvelo: 0.5, mvacc: 0.5, mvtime: 0.0, mvradii: 0.0, coord_mode: 0, index: 0}"
```

#### 直线运动
```bash
ros2 service call /jaka_driver/linear_move jaka_msgs/srv/Move \
  "{pose: [111.126, 282.111, 271.55, 3.142, 0.0, -0.698], has_ref: false, ref_joint: [0], mvvelo: 100, mvacc: 100, mvtime: 0.0, mvradii: 0.0, coord_mode: 0, index: 0}"
```

### MoveIt 2 规划模式
启动 MoveIt 2 服务器，替换 `<robot_ip>` 和 `<robot_model>` 为实际参数：
```bash
ros2 launch jaka_planner moveit_server.launch.py ip:=<robot_ip> model:=<robot_model>
```

### RViz 可视化
在新终端中启动 RViz 界面：
```bash
ros2 launch jaka_zu12_moveit_config demo.launch.py
```

## 工作流程

1. **硬件连接**: `jaka_driver` 通过 IP 地址连接到 JAKA 机器人控制器
2. **状态发布**: 驱动模块实时发布机器人的关节状态和系统状态
3. **运动控制**: 
   - 直接控制：通过 `jaka_driver` 的服务接口发送运动指令
   - 规划控制：通过 `jaka_planner` 使用 MoveIt 2 进行路径规划
4. **可视化**: 在 RViz 中显示机器人模型和运动轨迹
5. **高级操作**: 通过 `jaka_manipulation` 执行复杂的操作任务

## 支持的机器人型号

当前版本支持所有官方发布的 JAKA 六自由度协作机器人型号：
- JAKA A 系列
- JAKA C 系列  
- JAKA Pro 系列
- JAKA S 系列
- JAKA Zu 系列
- JAKA MiniCobo

## 详细文档

- [JAKA ROS 2 Documentation（英文版）](jaka_ros2_documentation.md)
- [JAKA ROS 2 Documentation（中文版）](jaka_ros2_documentation-中文版.md)
- [Release Notes](release_notes.md)

## 注意事项

- 确保 `jaka_driver` 和 MoveIt 2 不能同时运行
- 使用前请确认机器人控制器版本符合要求
- 建议在测试环境中先验证功能后再用于生产环境

# BUILD
vcpkg install abseil:x64-linux
just b
待完善