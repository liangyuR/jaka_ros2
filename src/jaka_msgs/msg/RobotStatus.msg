# 机器人详细状态信息
# 基于JAKA SDK的RobotStatus结构体

# 基础状态信息
int32 errcode                    # 错误码，0表示正常，其他表示异常
bool inpos                       # 机器人是否到位，true表示到位，false表示未到位
bool powered_on                  # 机器人是否上电，true表示上电，false表示未上电
bool enabled                     # 机器人是否使能，true表示使能，false表示未使能
float64 rapidrate                # 机器人运动倍率
bool protective_stop             # 是否检测到碰撞，true表示检测到碰撞，false表示未检测到
bool emergency_stop              # 是否急停，true表示急停，false表示未急停
bool connected                   # 是否已经连接, true: 连接, false: 未连接

# 位置信息
float64[] cartesiantran_position  # 机器人末端笛卡尔空间位置 [x, y, z, rx, ry, rz]
float64[] joint_position          # 机器人关节空间位置 [j1, j2, j3, j4, j5, j6]

# 坐标系信息
uint32 current_tool_id           # 当前使用的工具坐标系ID
uint32 current_user_id           # 当前使用的用户坐标系ID
bool on_soft_limit               # 是否在软限位位置，true表示触发限位保护，false表示未触发

# 运动状态
bool drag_status                 # 是否处于拖拽状态，true表示拖拽状态，false表示非拖拽状态
bool is_socket_connect           # SDK与控制器连接通道是否正常，true表示正常，false表示异常

# 数字IO信号
int32[] dout                     # 机器人控制柜数字输出信号
int32[] din                      # 机器人控制柜数字输入信号
float64[] ain                    # 机器人控制柜模拟输入信号
float64[] aout                   # 机器人控制柜模拟输出信号

# 工具端IO信号
int32[] tio_dout                 # 机器人末端工具数字输出信号
int32[] tio_din                  # 机器人末端工具数字输入信号
float64[] tio_ain                # 机器人末端工具模拟输入信号
int32[] tio_key                  # 机器人末端工具按键 [0]free;[1]point;[2]pause_resume

# 时间戳
builtin_interfaces/Time timestamp # 状态时间戳 