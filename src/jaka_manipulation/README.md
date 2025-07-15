# JAKA Manipulation Package

这个包提供了 JAKA 机械臂的高级操作功能，支持仿真和真实机器人模式。

## Launch 文件

### 1. 主要启动文件

#### `manipulation_planning.launch.py`
完整的运动规划启动文件，支持多种模式：

```bash
# 仿真模式
ros2 launch jaka_manipulation manipulation_planning.launch.py mode:=simulation

# 真实机器人模式
ros2 launch jaka_manipulation manipulation_planning.launch.py mode:=real robot_ip:=192.168.1.100

# 带调试的仿真模式
ros2 launch jaka_manipulation manipulation_planning.launch.py mode:=simulation debug:=true
```

#### `quick_start.launch.py`
快速启动文件，使用默认配置：

```bash
# 默认仿真模式
ros2 launch jaka_manipulation quick_start.launch.py

# 真实机器人模式
ros2 launch jaka_manipulation quick_start.launch.py mode:=real robot_ip:=192.168.1.100
```

#### `gazebo_simulation.launch.py`
Gazebo 仿真环境启动：

```bash
# 启动 Gazebo 仿真
ros2 launch jaka_manipulation gazebo_simulation.launch.py

# 指定机器人型号
ros2 launch jaka_manipulation gazebo_simulation.launch.py robot_model:=zu12
```

### 2. 参数说明

- `mode`: 运行模式 (`simulation` | `real` | `gazebo`)
- `robot_ip`: 机器人IP地址（仅真实模式需要）
- `robot_model`: 机器人型号（`zu12`, `zu3`, `s5` 等）
- `use_rviz`: 是否启动 RViz (`true` | `false`)
- `use_database`: 是否启动数据库 (`true` | `false`)
- `debug`: 是否启用调试模式 (`true` | `false`)

### 3. 典型使用场景

#### 开发和测试
```bash
# 在仿真环境中开发算法
ros2 launch jaka_manipulation manipulation_planning.launch.py mode:=simulation use_rviz:=true

# 在 Gazebo 中测试物理交互
ros2 launch jaka_manipulation gazebo_simulation.launch.py
```

#### 实际部署
```bash
# 连接真实机器人
ros2 launch jaka_manipulation manipulation_planning.launch.py mode:=real robot_ip:=<实际IP>
```

## 依赖项

确保已安装以下依赖：
- `jaka_driver`
- `jaka_planner`
- `jaka_zu12_moveit_config`
- `moveit_ros_planning_interface`
- `Qt6` (用于GUI界面)

## 故障排除

1. **连接错误**：检查机器人IP地址和网络连接
2. **启动失败**：确保所有依赖包已正确安装
3. **规划失败**：检查机器人模型和配置文件是否正确 