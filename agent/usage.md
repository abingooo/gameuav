# uav_agent Usage

本文档说明第一版 `uav_agent` 的使用方式。

当前 `uav_agent` 的定位是：无人机本机常驻运行的进程管理器。它通过 TCP 接收地面站命令，并且只能启动、停止、查询白名单里的模块。

它不是远程 shell，不允许执行任意命令。

## 1. 目录位置

核心文件：

```text
agent/
  uav_agent/
    server.py              # TCP agent 服务端
  launch_manager/
    manager.py             # 本机进程/roslaunch 管理器

comm/protocol/
  agent_protocol.py        # agent TCP JSON 协议

gcs/command/
  agentctl.py              # 本地命令行客户端，用来模拟地面站

config/modules/
  uav_agent.yaml           # agent 可控制模块白名单

config/ros_commands/
  ros_command_executor.yaml # agent 可执行 ROS runtime command 白名单
```

## 2. 启动 uav_agent

在工程根目录执行：

```bash
cd /home/uav/Desktop/uav_project/gameuav
python3 -m agent.uav_agent.server --host 0.0.0.0 --port 8765 --uav-id uav1
```

本机调试时可以只监听 `127.0.0.1`：

```bash
python3 -m agent.uav_agent.server --host 127.0.0.1 --port 8765 --uav-id uav1
```

常用参数：

```text
--host              监听地址，默认 0.0.0.0
--port              TCP 端口，默认 8765
--uav-id            当前无人机 ID，默认 uav1
--config            模块白名单配置，默认 config/modules/uav_agent.yaml
--ros-command-config ROS 命令白名单配置，默认 config/ros_commands/ros_command_executor.yaml
--log-dir           被 agent 启动的模块日志目录，默认 logs/agent
--ros-home          ROS_HOME，默认 /tmp/gameuav_ros_home
--ros-log-dir       ROS_LOG_DIR，默认 /tmp/gameuav_ros_logs
--auth-token        TCP 命令 token，默认读取 GAMEUAV_AGENT_TOKEN，否则为 uavuavuavuav
--allowed-source    允许的 source_id，可传多次；不传表示不限制
--verbose           输出调试日志
```

示例：

```bash
python3 -m agent.uav_agent.server \
  --host 0.0.0.0 \
  --port 8765 \
  --uav-id uav1 \
  --log-dir logs/agent \
  --ros-home /tmp/gameuav_ros_home \
  --ros-log-dir /tmp/gameuav_ros_logs \
  --auth-token uavuavuavuav
```

## 3. 使用 agentctl 发送命令

`agentctl.py` 是当前第一版的命令行客户端，用来模拟地面站。

基本格式：

```bash
python3 tools/agentctl.py ACTION [MODULE] [--arg key:=value]
```

指定远端 agent：

```bash
python3 tools/agentctl.py ACTION [MODULE] \
  --host 192.168.1.101 \
  --port 8765 \
  --source-id gcs \
  --target-id uav1 \
  --auth-token uavuavuavuav
```

允许的动作：

```text
list      列出 agent 白名单模块
health    查询 agent/ROS master/模块整体健康状态
status    查询模块状态
start     启动模块
stop      停止模块
restart   重启模块
```

## 4. 常用命令

列出可控模块：

```bash
python3 tools/agentctl.py list
```

查询 `roscore` 状态：

```bash
python3 tools/agentctl.py status roscore
```

查询整体健康状态：

```bash
python3 tools/agentctl.py health
```

启动 `roscore`：

```bash
python3 tools/agentctl.py start roscore
```

启动 `mavros`：

```bash
python3 tools/agentctl.py start mavros
```

停止 `mavros`：

```bash
python3 tools/agentctl.py stop mavros
```

重启 `mavros`：

```bash
python3 tools/agentctl.py restart mavros
```

启动 RealSense：

```bash
python3 tools/agentctl.py start realsense
```

指定 RealSense 序列号：

```bash
python3 tools/agentctl.py start realsense --arg serial_no:=832112071797
```

启动 VINS：

```bash
python3 tools/agentctl.py start vins
```

启动 EGO：

```bash
python3 tools/agentctl.py start ego
```

启动 px4ctrl：

```bash
python3 tools/agentctl.py start px4ctrl
```

启动 tiplight：

```bash
python3 tools/agentctl.py start tiplight
```

## 5. 推荐启动顺序

单机真实飞行链路建议按这个顺序：

```bash
python3 tools/agentctl.py start roscore
python3 tools/agentctl.py start mavros
python3 tools/agentctl.py start realsense --arg serial_no:=832112071797
python3 tools/agentctl.py start vins
python3 tools/agentctl.py start px4ctrl
python3 tools/agentctl.py start ego
```

如果想用组合 launch：

```bash
python3 tools/agentctl.py start state_estimation --arg camera_serial_no:=832112071797
python3 tools/agentctl.py start px4ctrl
python3 tools/agentctl.py start ego
```

或者启动更完整的真实飞行组合：

```bash
python3 tools/agentctl.py start realflight --arg camera_serial_no:=832112071797
```

## 6. 当前白名单模块

白名单配置在：

```text
config/modules/uav_agent.yaml
```

当前包含：

```text
roscore
mavros
realsense
state_estimation
vins
ego
px4ctrl
flight_control
realflight
tiplight
safe_takeoff
safe_land
```

只有这里列出的模块才能被 agent 启动、停止、重启和查询。

模块可设置 `autostart: true`。agent 进程启动时会自动启动这些模块；未设置该字段的模块仍保持按需启动。

## 7. 传递 launch 参数

地面站不能传任意参数，只能传白名单里允许的参数。

示例：`mavros` 允许传 `fcu_url` 和 `configure_stream_rates`。

```bash
python3 tools/agentctl.py start mavros \
  --arg fcu_url:=/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00:57600 \
  --arg configure_stream_rates:=true
```

示例：`px4ctrl` 允许传 `no_rc` 和 `enable_auto_arm`。

```bash
python3 tools/agentctl.py start px4ctrl \
  --arg no_rc:=false \
  --arg enable_auto_arm:=true
```

如果传了未授权参数，agent 会拒绝执行。

例如：

```bash
python3 tools/agentctl.py start mavros --arg unsafe:=1
```

会返回错误：

```text
argument is not allowed: unsafe
```

## 8. 日志位置

被 agent 启动的模块日志默认放在：

```text
logs/agent/
```

每个模块会生成类似：

```text
logs/agent/mavros_20260520_142506.log
logs/agent/vins_20260520_143012.log
```

如果启动失败，先看对应模块日志。

## 9. ROS runtime command

`uav_agent` 除了能管理模块进程，也能执行受控 ROS 运行时命令。

这部分由 `agent/ros_command_executor/` 实现。它的定位不是远程 shell，而是“ROS 白名单命令执行器”：

```text
地面站只发送 command 名称和允许的参数
uav_agent 本机读取白名单配置
uav_agent 按配置转换成 rostopic/rosnode 等受控 ROS 操作
```

也就是说，地面站不能指定任意 topic、任意 msg type、任意 shell 命令。

命令格式：

```bash
python3 tools/agentctl.py ros COMMAND [--arg key:=value] --auth-token uavuavuavuav
```

当前白名单配置：

```text
config/ros_commands/ros_command_executor.yaml
```

如果修改了这个文件，需要重启 `uav_agent` 才会加载新配置：

```bash
sudo systemctl restart gameuav-agent.service
```

当前默认启用：

```text
health
command_list
topic_list
node_list
set_goal
tiplight
```

当前默认禁用：

```text
takeoff
land
arm
mode
```

命令含义：

| 命令 | 是否默认启用 | 是否需要 ROS master | 作用 |
| --- | --- | --- | --- |
| `health` | 是 | 否 | 检查 ROS master 是否可达 |
| `command_list` | 是 | 否 | 查询当前 agent 暴露的 ROS 命令白名单 |
| `topic_list` | 是 | 是 | 执行 `rostopic list` |
| `node_list` | 是 | 是 | 执行 `rosnode list` |
| `set_goal` | 是 | 是 | 向 `/planning/goal` 发布 `geometry_msgs/PoseStamped` |
| `ego_position` | 是 | 是 | 向 `/control/ego_position` 发布目标，由 EGO 规划执行 |
| `position` | 是 | 是 | 向 `/control/position` 发布直接位置目标，不经过 EGO 避障 |
| `speed` | 是 | 是 | 向 `/control/speed` 发布短时速度命令，需要持续刷新 |
| `control_stop` | 是 | 是 | 向 `/control/stop` 发布悬停请求 |
| `tiplight` | 是 | 是 | 向 `/actuation/tiplight_cmd` 发布 `std_msgs/String` |
| `safe_takeoff` | 是 | 是 | 起飞安全检查，默认 dry-run，不发布起飞命令 |
| `safe_land` | 是 | 是 | 降落安全检查，默认 dry-run，不发布降落命令 |
| `takeoff` | 否 | 是 | 起飞命令，占位，安全审核后再启用 |
| `land` | 否 | 是 | 降落命令，占位，安全审核后再启用 |
| `arm` | 否 | 是 | 解锁命令，占位，后续接 MAVROS service |
| `mode` | 否 | 是 | 模式切换，占位，后续接 MAVROS service |

查询 ROS master：

```bash
python3 tools/agentctl.py ros health --auth-token uavuavuavuav
```

列出 ROS runtime command 白名单：

```bash
python3 tools/agentctl.py ros command_list --auth-token uavuavuavuav
```

列出 ROS topic：

```bash
python3 tools/agentctl.py ros topic_list --auth-token uavuavuavuav
```

列出 ROS node：

```bash
python3 tools/agentctl.py ros node_list --auth-token uavuavuavuav
```

发布规划目标点：

```bash
python3 tools/agentctl.py ros set_goal \
  --arg x:=1.0 \
  --arg y:=0.0 \
  --arg z:=1.2 \
  --arg yaw:=0.0 \
  --auth-token uavuavuavuav
```

这个命令会发布：

```text
topic: /planning/goal
type:  geometry_msgs/PoseStamped
```

`yaw` 会转换成 `geometry_msgs/PoseStamped.pose.orientation` 的平面四元数：

```bash
python3 tools/agentctl.py ros set_goal \
  --arg x:=1.0 \
  --arg y:=0.0 \
  --arg z:=1.2 \
  --arg yaw:=1.57 \
  --auth-token uavuavuavuav
```

发布灯光命令：

```bash
python3 tools/agentctl.py ros tiplight --arg data:=red --auth-token uavuavuavuav
```

允许的灯光参数由白名单配置控制，当前为：

```text
off
red
green
blue
yellow
white
blink
```

非法命令会被拒绝：

```bash
python3 tools/agentctl.py ros unknown --auth-token uavuavuavuav
```

默认禁用的危险命令也会被拒绝：

```bash
python3 tools/agentctl.py ros takeoff --auth-token uavuavuavuav
```

预期返回：

```text
ros command is disabled: takeoff
```

安全起降 dry-run：

```bash
python3 tools/agentctl.py ros safe_takeoff --auth-token uavuavuavuav
python3 tools/agentctl.py ros safe_land --auth-token uavuavuavuav
```

这两个命令当前只做安全检查，不会真正发布 `/px4ctrl/takeoff_land`。

`safe_takeoff` 当前检查：

```text
1. ROS master 可达
2. /mavros 节点存在
3. /px4ctrl 节点存在
4. /mavros/state topic 存在，并且 connected=true
5. /mavros/state 显示 armed=false
6. /mavros/extended_state topic 存在，并且 landed_state=1
7. /mavros/local_position/pose 有消息
8. /vins_fusion/imu_propagate 有消息
9. /px4ctrl/takeoff_land topic 存在并且有 /px4ctrl 订阅者
```

`safe_land` 当前检查：

```text
1. ROS master 可达
2. /mavros 节点存在
3. /px4ctrl 节点存在
4. /mavros/state topic 存在，并且 connected=true
5. /mavros/extended_state topic 存在，并且 landed_state=2
6. /mavros/local_position/pose 有消息
7. /vins_fusion/imu_propagate 有消息
8. /px4ctrl/takeoff_land topic 存在
```

如果检查失败，返回里会有 `checks` 列表，每一项都会说明通过或失败原因。

真实执行开关在白名单配置里：

```yaml
safe_takeoff:
  dry_run: true
  allow_execute: false
```

真实启用必须同时满足：

```text
1. 配置 allow_execute: true
2. 调用时传 dry_run:=false
3. 所有 safety checks 通过
```

示例：

```bash
python3 tools/agentctl.py ros safe_takeoff \
  --arg dry_run:=false \
  --auth-token uavuavuavuav
```

注意：当前配置里 `allow_execute: false`，所以上面命令也只会返回拒绝，不会起飞。

新增 ROS 命令的流程：

```text
1. 在 config/ros_commands/ros_command_executor.yaml 中新增 command
2. 明确 type，目前支持 builtin / publish / safety_command
3. 明确 topic、msg_type、message 模板
4. 明确 args 白名单、类型、默认值、范围或枚举值
5. 先保持 enabled: false
6. 本机确认消息类型和 topic 正确后再启用
7. 重启 gameuav-agent.service
8. 用 agentctl.py ros command_list 确认配置已加载
```

发布类命令示例：

```yaml
set_goal:
  enabled: true
  type: publish
  topic: /planning/goal
  msg_type: geometry_msgs/PoseStamped
  args:
    x:
      type: float
      required: true
      min: -1000.0
      max: 1000.0
  message:
    pose:
      position:
        x: "{x}"
```

安全原则：

```text
1. 不允许地面站传 shell 命令
2. 不允许地面站临时指定 topic/msg_type
3. 不允许未声明参数
4. 起飞、降落、解锁、模式切换必须默认禁用
5. 危险动作启用前必须增加状态检查和互锁条件
```

## 10. systemd 开机自启

部署文件在：

```text
deploy/systemd/
  gameuav-agent.service
  gameuav-agent.env
  install_gameuav_agent_service.sh
```

安装：

```bash
cd /home/uav/Desktop/uav_project/gameuav
sudo deploy/systemd/install_gameuav_agent_service.sh
```

安装脚本会：

```text
1. 写入 /etc/systemd/system/gameuav-agent.service
2. 写入 /etc/gameuav/agent.env
3. systemctl daemon-reload
4. systemctl enable gameuav-agent.service
```

启动或重启服务：

```bash
sudo systemctl restart gameuav-agent.service
```

查看状态：

```bash
systemctl status gameuav-agent.service
```

查看日志：

```bash
journalctl -u gameuav-agent.service -f
```

systemd 环境配置在：

```text
/etc/gameuav/agent.env
```

如果修改了 `deploy/systemd/gameuav-agent.service` 或新增了 env 字段，需要重新安装：

```bash
sudo deploy/systemd/install_gameuav_agent_service.sh
sudo systemctl restart gameuav-agent.service
```

默认内容来自：

```text
deploy/systemd/gameuav-agent.env
```

当前默认 token：

```text
GAMEUAV_AGENT_TOKEN=uavuavuavuav
```

修改 token 后需要重启：

```bash
sudo systemctl restart gameuav-agent.service
```

## 11. 停止逻辑

`stop` 会按顺序尝试：

```text
SIGINT
SIGTERM
SIGKILL
```

也就是说，它会先尽量让 `roslaunch` 正常退出，超时后再强制停止。

## 12. TCP 协议格式

TCP 上每个消息是一行 JSON，以 `\n` 结尾。

请求示例：

```json
{
  "protocol_version": "gameuav.agent.v1",
  "message_type": "module_command",
  "source_id": "gcs",
  "target_id": "uav1",
  "sequence_id": "1001",
  "timestamp": 1710000000.0,
  "payload": {
    "request_id": "1001",
    "auth_token": "uavuavuavuav",
    "action": "start",
    "module": "vins"
  },
  "checksum": "crc32"
}
```

响应示例：

```json
{
  "protocol_version": "gameuav.agent.v1",
  "message_type": "module_status",
  "source_id": "uav1",
  "target_id": "gcs",
  "sequence_id": "1001",
  "timestamp": 1710000001.0,
  "payload": {
    "request_id": "1001",
    "ok": true,
    "action": "start",
    "module": "vins",
    "status": {
      "module": "vins",
      "status": "running",
      "pid": 23152,
      "type": "launch",
      "log_path": "logs/agent/vins_20260520_143012.log"
    }
  },
  "checksum": "crc32"
}
```

错误响应示例：

```json
{
  "protocol_version": "gameuav.agent.v1",
  "message_type": "error",
  "source_id": "uav1",
  "target_id": "gcs",
  "sequence_id": "1001",
  "timestamp": 1710000001.0,
  "payload": {
    "request_id": "1001",
    "ok": false,
    "code": "ModuleRuntimeError",
    "detail": "module is not whitelisted: shell"
  },
  "checksum": "crc32"
}
```

ROS runtime command 请求示例：

```json
{
  "protocol_version": "gameuav.agent.v1",
  "message_type": "ros_command",
  "source_id": "gcs",
  "target_id": "uav1",
  "sequence_id": "1002",
  "timestamp": 1710000000.0,
  "payload": {
    "request_id": "1002",
    "auth_token": "uavuavuavuav",
    "command": "set_goal",
    "args": {
      "x": "1.0",
      "y": "0.0",
      "z": "1.2"
    }
  },
  "checksum": "crc32"
}
```

## 13. token 鉴权

第一版使用固定 token 鉴权。

agent 默认 token：

```text
uavuavuavuav
```

服务端配置方式：

```bash
python3 -m agent.uav_agent.server --auth-token uavuavuavuav
```

或者通过环境变量：

```bash
export GAMEUAV_AGENT_TOKEN=uavuavuavuav
python3 -m agent.uav_agent.server
```

客户端配置方式：

```bash
python3 tools/agentctl.py health --auth-token uavuavuavuav
```

如果 token 错误，会返回：

```text
invalid auth token
```

注意：固定 token 只适合第一版内网调试。后续更规范的做法是 HMAC 签名、TLS 或 VPN。

## 14. 安全边界

当前第一版已经具备这些安全限制：

```text
1. 不执行地面站传来的任意 shell 字符串
2. 只能操作 config/modules/uav_agent.yaml 白名单模块
3. launch 参数必须在 allowed_args 中声明
4. 每个模块使用独立进程组
5. 每个请求都有响应
6. 错误命令会返回 error，不会静默执行
7. TCP 命令必须带正确 token
8. ROS runtime command 必须在 config/ros_commands/ros_command_executor.yaml 中声明
9. 起飞、降落、解锁、模式切换默认禁用
10. 发布类 ROS 命令只能使用本机白名单里固定的 topic/msg_type/message 模板
```

仍建议后续补充：

```text
1. HMAC 签名
2. TLS 或局域网 VPN
3. 更完整的 systemd sandbox 限制
4. heartbeat/health_monitor
5. 日志流式传输
6. 起飞、降落、悬停、模式切换的安全互锁协议
7. ROS service/action 类型命令支持
```

## 15. 快速自测

启动 agent：

```bash
cd /home/uav/Desktop/uav_project/gameuav
python3 -m agent.uav_agent.server --host 127.0.0.1 --port 8765 --uav-id uav1
```

另开一个终端：

```bash
cd /home/uav/Desktop/uav_project/gameuav
python3 tools/agentctl.py list
python3 tools/agentctl.py health
python3 tools/agentctl.py status roscore
python3 tools/agentctl.py ros health
python3 tools/agentctl.py ros command_list
```

测试非法模块：

```bash
python3 tools/agentctl.py start shell
```

预期返回：

```text
module is not whitelisted: shell
```

运行单元测试：

```bash
python3 -m unittest tests.test_agent_protocol tests.test_launch_manager tests.test_uav_agent_server tests.test_ros_command_executor
```
