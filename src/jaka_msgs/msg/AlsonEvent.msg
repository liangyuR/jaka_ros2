# ALSON Camera Event Message
# 事件类型
uint8 EVENT_TYPE_CONNECTION_STATUS = 0
uint8 EVENT_TYPE_DATA_RECEIVED = 1
uint8 EVENT_TYPE_RECONNECT_STATUS = 2
uint8 EVENT_TYPE_HEARTBEAT_STATUS = 3
uint8 EVENT_TYPE_PROJECT_STATUS = 4

# 状态类型
uint8 STATUS_SUCCESS = 0
uint8 STATUS_FAILED = 1
uint8 STATUS_WAITING = 2
uint8 STATUS_GIVEUP = 3

# 消息头
std_msgs/Header header

# 事件类型
uint8 event_type

# 状态
uint8 status

# 消息内容
string message

# 额外数据（如接收到的数据、重连次数等）
string data

# 重连相关
int32 retry_count
int32 delay_seconds

# 连接状态
bool connected 