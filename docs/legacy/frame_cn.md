# GameUAV 架构框架

## 1. 设计目标

GameUAV 是一个通用多无人机协同系统。

核心原则：

```text
单架无人机内部：ROS 通信
无人机之间、无人机与地面站之间：TCP / UDP 通信
ROS 与 TCP/UDP 之间：gateway 负责转换
```

系统主干不应该绑定某个阶段性算法。ADV、MPC、规则策略、编队策略都应该作为可插拔策略插件存在。

最终目标：

```text
ROS 留在单机内部
跨机器只传受控网络消息
地面站只发语义命令
无人机本机 agent/gateway 再转换成本机 ROS 操作
本机 safety 始终能独立接管安全动作
```

## 2. 推荐目录结构

```text
gameuav/
  frame.md
  frame_cn.md

  ros_nodes/
    perception/
    state_estimation/
    planning/
    control/
    actuation/
    safety/
    mission/

  agent/
    uav_agent/
    launch_manager/
    ros_command_executor/
    health_monitor/
    log_streamer/

  comm/
    protocol/
    udp_link/
    tcp_link/
    serializer/
    heartbeat/
    qos/
    router/

  gateway/
    ros_to_net_gateway/
    net_to_ros_gateway/
    topic_mapping/
    message_adapter/

  gcs/
    backend/
    frontend/
    monitor/
    command/

  strategy/
    interface/
    plugins/
      adv/
      mpc/
      rule_based/
      formation/

  config/
    fleet/
    network/
    topics/
    frames/
    mission/
    modules/

  deploy/
  docs/
  tests/
  tools/
```

一句话概括：

```text
ros_nodes = 飞机内部 ROS 功能
agent = 飞机本机 ROS 进程和命令管理器
comm = TCP/UDP 协议和传输
gateway = ROS 与网络消息互转
gcs = 地面站
strategy = 可替换协同算法
config = 机队、网络、话题、坐标系、模块配置
```

## 3. ros_nodes：单机内部 ROS 功能层

`ros_nodes/` 放每架无人机本机运行的 ROS 功能模块。

这些模块只通过本机 ROS topic、service、action 协作，不应该直接负责跨机器 TCP/UDP 通信。

### 3.1 perception

```text
ros_nodes/perception/
```

感知模块。

典型内容：

```text
相机
雷达
深度图
点云
目标检测
障碍物检测
```

### 3.2 state_estimation

```text
ros_nodes/state_estimation/
```

状态估计模块。

典型内容：

```text
VINS
SLAM
GPS/IMU 融合
里程计
位置、速度、姿态估计
```

### 3.3 planning

```text
ros_nodes/planning/
```

规划模块。

典型内容：

```text
全局路径规划
局部避障
轨迹生成
EGO Planner
多机协同轨迹生成
```

### 3.4 control

```text
ros_nodes/control/
```

控制模块。

典型内容：

```text
位置控制
速度控制
姿态控制
轨迹跟踪
PX4 / ArduPilot / MAVROS 适配
```

### 3.5 actuation

```text
ros_nodes/actuation/
```

非飞控类执行器模块。

典型内容：

```text
LED 灯珠
蜂鸣器
舵机
夹爪
继电器
云台
```

边界：

```text
control/    放飞行控制：位置、速度、姿态、轨迹跟踪、飞控接口
actuation/  放外设执行器：LED、蜂鸣器、舵机、夹爪、继电器、云台
```

示例 topic：

```text
/actuation/led_cmd
/actuation/servo_cmd
/actuation/gripper_cmd
```

地面站控制链路仍然走 gateway：

```text
GCS
  -> TCP ActuatorCommand
  -> net_to_ros_gateway
  -> /actuation/led_cmd
  -> led_node
```

### 3.6 safety

```text
ros_nodes/safety/
```

安全模块。

典型内容：

```text
急停
失联悬停
低电量返航/降落
越界保护
碰撞风险处理
定位丢失保护
```

安全模块必须本机独立运行，不能依赖地面站实时在线。

### 3.7 mission

```text
ros_nodes/mission/
```

任务模块。

典型内容：

```text
任务状态机
任务开始/暂停/恢复/终止
目标点管理
任务完成判断
任务进度发布
```

## 4. agent：无人机本机运行管理层

`agent/` 是地面站远程管理无人机 ROS 系统的入口。

它和普通 ROS 节点不同：`uav_agent` 最好是一个普通后台进程，由 systemd 开机自启。它不应该依赖 ROS master 已经启动，否则当 `roscore` 没起来时，地面站就无法通过它恢复系统。

### 4.1 uav_agent

```text
agent/uav_agent/
```

每架无人机上常驻运行的 TCP 命令接收器。

职责：

```text
接收地面站命令
检查命令是否合法
根据白名单分发给 launch_manager 或 ros_command_executor
返回 ACK、执行状态、错误信息
```

它不是任意 shell 执行器，不允许地面站传任意命令。

错误设计：

```json
{
  "command": "roslaunch 任意包 任意launch"
}
```

推荐设计：

```json
{
  "message_type": "module_command",
  "action": "start",
  "module": "vins"
}
```

`vins` 对应什么启动命令，由无人机本机 `config/modules/` 白名单决定。

### 4.2 launch_manager

```text
agent/launch_manager/
```

负责启动、停止、重启和监控 ROS 进程。

典型模块：

```text
roscore
mavros
vins
px4ctrl
ego_planner
strategy_manager
gateway
```

实现建议：

```text
使用 subprocess.Popen 或 systemd 管理进程
每个 roslaunch 单独进程组
保存 PID、启动时间、日志路径、运行状态
停止时先 SIGINT，再 SIGTERM，最后必要时 SIGKILL
```

### 4.3 ros_command_executor

```text
agent/ros_command_executor/
```

负责执行受控 ROS 运行时命令。

这类命令不是启动进程，而是在 ROS 已经运行后，发布 topic 或调用 service。

典型操作：

```text
起飞
降落
悬停
发布目标点
切换模式
调用参数服务
```

### 4.4 health_monitor

```text
agent/health_monitor/
```

负责检查无人机本机健康状态。

检查内容：

```text
进程是否存在
roscore 是否可达
rosnode 是否存在
rostopic 是否存在
关键 topic 是否有新消息
CPU / 内存 / 磁盘 / 网络状态
```

### 4.5 log_streamer

```text
agent/log_streamer/
```

负责日志查看、日志上传、日志流式回传。

## 5. comm：TCP/UDP 通信层

`comm/` 是纯网络通信层。

它不应该包含 VINS、EGO、ADV、MPC 这类业务逻辑。

### 5.1 protocol

```text
comm/protocol/
```

定义网络消息协议。

每个网络包建议至少包含：

```text
protocol_version
message_type
source_id
target_id
sequence_id
timestamp
payload
checksum 或 signature
```

常见消息类型：

```text
Heartbeat
UavState
FleetState
MissionCommand
MissionAck
ModuleCommand
ModuleStatus
SafetyEvent
ControlOverride
LogChunk
```

### 5.2 udp_link

```text
comm/udp_link/
```

UDP 通信代码。

适合低延迟、高频、允许偶尔丢包的数据：

```text
心跳
位置
速度
姿态
邻机状态
链路质量
```

### 5.3 tcp_link

```text
comm/tcp_link/
```

TCP 通信代码。

适合必须可靠到达的数据：

```text
启动/停止模块
任务下发
参数修改
模式切换
日志上传
地图/文件传输
需要 ACK 的控制命令
```

### 5.4 serializer

```text
comm/serializer/
```

序列化和反序列化。

负责：

```text
程序对象 -> 网络字节流
网络字节流 -> 程序对象
```

推荐路线：

```text
第一阶段：JSON，方便调试
第二阶段：MessagePack 或 Protobuf
第三阶段：高频数据有必要时再做自定义二进制
```

### 5.5 heartbeat

```text
comm/heartbeat/
```

心跳和在线状态检测。

推荐默认值：

```text
每 0.5 秒发送一次 heartbeat
超过 1.5 秒未收到，进入 warning
超过 3.0 秒未收到，认为 lost
通知 safety 模块执行失联策略
```

### 5.6 qos

```text
comm/qos/
```

通信质量控制。

职责：

```text
限频
限流
消息优先级
超时
ACK / 重传
过期消息丢弃
延迟统计
丢包统计
```

### 5.7 router

```text
comm/router/
```

消息路由。

负责决定消息发给谁：

```text
发给某一架无人机
发给所有无人机
发给地面站
发给某个编队小组
忽略不是发给自己的消息
```

## 6. gateway：ROS 与网络消息转换层

`gateway/` 是整个架构的关键隔离层。

原则：

```text
不要把完整 ROS graph 暴露到跨机器网络
只允许 gateway 把受控 ROS 信息转换成网络消息
只允许 gateway 把受控网络命令转换成本机 ROS 操作
```

### 6.1 ros_to_net_gateway

```text
gateway/ros_to_net_gateway/
```

负责把本机 ROS 消息转换成网络消息。

示例：

```text
订阅 /vins_position
        ↓
转换成 UavState 网络包
        ↓
UDP 发给其他无人机和地面站
```

### 6.2 net_to_ros_gateway

```text
gateway/net_to_ros_gateway/
```

负责把网络消息转换成本机 ROS 消息、service 或 action。

示例：

```text
TCP 收到 MissionCommand
        ↓
转换成 geometry_msgs/PoseStamped
        ↓
发布到 /planning/goal
```

### 6.3 topic_mapping

```text
gateway/topic_mapping/
```

管理 ROS topic 与网络消息类型的映射。

示例：

```text
/vins_position              -> UavState
/mavros/battery             -> BatteryState
/px4ctrl/takeoff_land       <- TakeoffLand
/planning/goal              <- MissionCommand
/actuation/led_cmd          <- ActuatorCommand
```

### 6.4 message_adapter

```text
gateway/message_adapter/
```

负责不同消息格式之间的转换。

示例：

```text
nav_msgs/Odometry -> UavState
geometry_msgs/PointStamped -> UavState
MissionCommand -> geometry_msgs/PoseStamped
TakeoffCommand -> quadrotor_msgs/TakeoffLand
ActuatorCommand -> LED/servo/gripper command topic
```

## 7. gcs：地面站

`gcs/` 是地面控制站。

地面站应该发送语义命令，而不是直接跨网发布无人机内部 ROS topic。

### 7.1 backend

```text
gcs/backend/
```

地面站后端。

职责：

```text
管理无人机连接
维护机队状态
提供 API
转发命令
管理日志
保存任务记录
```

### 7.2 frontend

```text
gcs/frontend/
```

地面站前端界面。

职责：

```text
地图显示
无人机状态显示
任务编辑
告警展示
人工控制界面
```

### 7.3 monitor

```text
gcs/monitor/
```

监控模块。

监控内容：

```text
在线/离线
电量
位置
速度
飞行模式
任务状态
模块状态
网络质量
安全事件
```

### 7.4 command

```text
gcs/command/
```

命令下发模块。

支持命令：

```text
启动模块
停止模块
起飞
降落
返航
悬停
设置目标点
开始任务
暂停任务
恢复任务
终止任务
人工接管
```

## 8. strategy：可替换策略层

`strategy/` 放协同算法。

ADV、MPC 是插件，不是系统主干。

### 8.1 interface

```text
strategy/interface/
```

定义统一策略接口。

推荐输入：

```text
FleetState
MissionState
EnvironmentState
StrategyConfig
```

推荐输出：

```text
RoleAssignment
TrajectoryIntent
ControlCommand
MissionDecision
```

### 8.2 plugins

```text
strategy/plugins/adv/
strategy/plugins/mpc/
strategy/plugins/rule_based/
strategy/plugins/formation/
```

每个策略插件只依赖统一策略接口，不应该直接依赖 TCP/UDP，不应该直接散乱订阅所有 ROS topic。

策略由配置决定：

```yaml
strategy:
  active: adv
  plugins:
    adv:
      config: config/strategy/adv.yaml
    mpc:
      config: config/strategy/mpc.yaml
    rule_based:
      config: config/strategy/rule_based.yaml
```

系统其他部分只关心策略输出，不关心当前用的是 ADV、MPC 还是规则策略。

## 9. config：配置层

`config/` 放系统运行配置。

### 9.1 fleet

```text
config/fleet/
```

机队配置。

示例：

```yaml
uavs:
  uav0:
    id: 0
    ip: 20.0.0.188
    role: defender
  uav1:
    id: 1
    ip: 20.0.0.187
    role: defender
ground:
  ip: 20.0.0.172
```

### 9.2 network

```text
config/network/
```

网络配置。

内容：

```text
UDP 端口
TCP 端口
广播地址
心跳频率
超时时间
重连参数
ACK / 重传参数
```

### 9.3 topics

```text
config/topics/
```

ROS topic 配置。

内容：

```text
本机 odom topic
电池 topic
起飞/降落 topic
目标点 topic
控制指令 topic
任务状态 topic
```

### 9.4 frames

```text
config/frames/
```

坐标系配置。

内容：

```text
world
map
odom
base_link
每架无人机本地原点
local -> world 平移和 yaw
```

### 9.5 mission

```text
config/mission/
```

任务配置。

内容：

```text
航点
目标区域
禁飞区
安全边界
任务参数
```

### 9.6 modules

```text
config/modules/
```

`uav_agent` 可启动/停止模块白名单。

示例：

```yaml
modules:
  roscore:
    type: process
    command: roscore

  mavros:
    type: launch
    package: px4ctrl
    launch: mavros_px4_namespaced.launch

  vins:
    type: launch
    package: vins
    launch: fast_drone_250.launch

  px4ctrl:
    type: launch
    package: px4ctrl
    launch: run_ctrl.launch

  ego:
    type: launch
    package: ego_planner
    launch: single_run_in_exp.launch
```

## 10. 运行架构

### 10.1 单架无人机内部

```text
uav_agent
  |
  | 启动 / 停止 / 检查
  v
roscore
mavros
vins
px4ctrl
ego_planner
strategy_manager
gateway
safety
mission
```

本机 ROS 数据流：

```text
perception
  -> state_estimation
  -> mission
  -> strategy
  -> planning
  -> control
  -> flight controller
```

### 10.2 跨机器通信

```text
无人机本机 ROS topic
  -> ros_to_net_gateway
  -> comm udp/tcp
  -> 地面站或其他无人机

地面站或其他无人机
  -> comm udp/tcp
  -> net_to_ros_gateway
  -> 无人机本机 ROS topic/service/action
```

不做完整 ROS graph 跨机同步。

只传受控的系统级消息：

```text
UavState
Heartbeat
MissionCommand
MissionAck
ModuleCommand
ModuleStatus
SafetyEvent
TrajectoryIntent
ControlOverride
```

## 11. 地面站如何控制无人机 ROS 系统

地面站控制分两类。

### 11.1 进程级控制

用于启动/停止 ROS 模块。

示例：

```text
启动 roscore
启动 mavros
启动 vins
启动 px4ctrl
启动 ego_planner
停止 ego_planner
重启 vins
查询模块状态
查看模块日志
```

链路：

```text
GCS
  -> TCP ModuleCommand
  -> uav_agent
  -> launch_manager
  -> subprocess/systemd/roslaunch
  -> ModuleStatus ACK
  -> GCS
```

启动 VINS 示例：

```json
{
  "message_type": "module_command",
  "request_id": 1001,
  "target_id": "uav1",
  "action": "start",
  "module": "vins",
  "timestamp": 1710000000.0
}
```

无人机回包：

```json
{
  "message_type": "module_status",
  "request_id": 1001,
  "source_id": "uav1",
  "module": "vins",
  "status": "starting",
  "pid": 23152,
  "detail": "roslaunch vins fast_drone_250.launch"
}
```

### 11.2 ROS 运行时控制

用于对已经运行的 ROS 系统发指令。

示例：

```text
起飞
降落
悬停
设置目标点
暂停任务
恢复任务
终止任务
人工接管
```

链路：

```text
GCS
  -> TCP MissionCommand / ControlCommand
  -> UAV net_to_ros_gateway
  -> 本机 ROS topic/service/action
  -> mission/control/safety
```

起飞命令示例：

```json
{
  "message_type": "control_command",
  "request_id": 2001,
  "target_id": "uav1",
  "command": "takeoff",
  "height": 1.0,
  "timestamp": 1710000000.0
}
```

本机 ROS 输出：

```text
topic: /px4ctrl/takeoff_land
type: quadrotor_msgs/TakeoffLand
payload: takeoff_land_cmd = 1
```

目标点命令示例：

```json
{
  "message_type": "mission_command",
  "request_id": 2002,
  "target_id": "uav1",
  "command": "set_goal",
  "position": [1.0, 0.0, 0.8],
  "yaw_deg": 0.0
}
```

本机 ROS 输出：

```text
topic: /planning/goal
type: geometry_msgs/PoseStamped
```

## 12. uav_agent 设计规则

`uav_agent` 是受控的无人机本机运行管理器，不是通用 shell 后门。

必须遵守：

```text
1. ROS 启动前就能运行
2. 固定 TCP 控制端口
3. 只接受白名单模块和白名单命令
4. 不执行地面站传来的任意 shell 字符串
5. roslaunch 进程独立进程组管理
6. 保存 PID、启动时间、日志路径、状态
7. 每个 request_id 都必须 ACK
8. 停止模块时先温和停止，再强制停止
9. 能向地面站回传健康状态和日志
10. 关键模块异常时通知 gateway/safety
```

推荐启动顺序：

```text
1. uav_agent 由 systemd 开机启动
2. GCS -> start roscore
3. GCS -> start mavros
4. GCS -> start vins
5. GCS -> start px4ctrl
6. GCS -> start ego_planner
7. GCS -> start gateway / mission / safety
8. GCS -> takeoff 或 mission command
```

## 13. TCP / UDP 使用策略

UDP 用于：

```text
heartbeat
UavState
邻机位置/速度/姿态
高频遥测
链路质量
```

TCP 用于：

```text
模块启动/停止
任务命令
需要 ACK 的控制命令
参数修改
模式切换
日志上传
文件传输
配置同步
```

UDP 数据要求：

```text
必须有 sequence_id
必须有 timestamp
过期包直接丢弃
高频遥测不做重传
```

TCP 命令要求：

```text
必须有 request_id
必须返回 ACK
必须有超时
尽量设计为幂等命令
需要返回最终执行状态
```

## 14. 安全策略

安全决策必须在无人机本机完成。

无人机不能依赖地面站实时在线才能保持安全。

最低安全策略：

```text
GCS 失联 -> 按策略继续任务/悬停/返航
邻机失联 -> 多机协同降级
定位丢失 -> 悬停/降落
低电量 -> 返航/降落
越界 -> 停止任务并返航/降落
碰撞风险 -> 避障/悬停
人工急停 -> 立即执行安全动作
```

`safety` 模块应该订阅：

```text
本机健康状态
心跳状态
电池状态
定位质量
任务状态
通信状态
```

## 15. 从当前 muav 迁移

当前 `../muav` 可以按职责迁移。

建议映射：

```text
core/src/realflight_modules/realsense-ros
  -> ros_nodes/perception/

core/src/realflight_modules/VINS-Fusion
  -> ros_nodes/state_estimation/

core/src/planner/plan_manage 及 EGO planner 相关包
  -> ros_nodes/planning/

core/src/realflight_modules/px4ctrl
  -> ros_nodes/control/

core/src/groundctrl
  -> gcs/backend/, gcs/monitor/, gcs/command/

core/src/swarm_position_bridge
  -> gateway/ros_to_net_gateway/ 和 gateway/message_adapter/

core/src/planner/rosmsg_tcp_bridge
  -> 只作为参考，建议用 comm + gateway 重新实现

adv/src/adv
  -> strategy/plugins/adv/

mpc/src/mpc
  -> strategy/plugins/mpc/

core/shfiles/*.sh
  -> config/modules/ 和 agent/launch_manager/
```

推荐迁移顺序：

```text
1. 先定义 config/fleet、config/network、config/topics
2. 实现 uav_agent，可启动/停止/查询 roscore 和现有 launch
3. 实现 GCS -> uav_agent 的 TCP ModuleCommand
4. 实现 UDP heartbeat 和 UavState
5. 实现 ros_to_net_gateway，发布本机状态
6. 实现 net_to_ros_gateway，支持 takeoff、land、set_goal
7. 把 ADV/MPC 放到 strategy/interface 后面
8. 逐步减少 fkie_master_sync 和跨机 ROS topic 依赖
9. 补齐 lost-link、模块异常等 safety 策略
10. 增加仿真、集成、硬件在环测试
```

## 16. 第一阶段最小闭环

第一阶段不要一次性迁移所有模块。

最小可行目标：

```text
uav_agent
GCS command backend
TCP start/stop/status module command
UDP heartbeat
UDP UavState
net_to_ros_gateway 支持 takeoff/land/set_goal
ros_to_net_gateway 支持 odom/battery/module status
基础 lost-link safety 策略
```

完成这个闭环后，再迁移 ADV、MPC、EGO、VINS 等具体模块。

## 17. 最终原则

```text
地面站发送语义命令
无人机本机 agent/gateway 转换命令
ROS 只在单机内部使用
TCP/UDP 是唯一跨机器通信层
安全逻辑留在无人机本机
ADV/MPC 是插件，不是主干
```
