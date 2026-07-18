# uav_agent

`uav_agent` 是无人机本机进程管理器。它应该在 ROS 启动前运行，负责接收地面站 TCP 命令，并启动、停止、查询本机 ROS 模块。

它不是远程 shell。地面站只能操作 `config/modules/uav_agent.yaml` 白名单里的模块。

## 启动 agent

```bash
cd /home/uav/Desktop/uav_project/gameuav
python3 -m agent.uav_agent.server --host 0.0.0.0 --port 8765 --uav-id uav1
```

默认 token 为：

```text
uavuavuavuav
```

也可以显式指定：

```bash
python3 -m agent.uav_agent.server --host 0.0.0.0 --port 8765 --uav-id uav1 --auth-token uavuavuavuav
```

## 命令示例

列出可控模块：

```bash
python3 tools/agentctl.py list
```

启动 ROS master：

```bash
python3 tools/agentctl.py start roscore
```

启动 MAVROS：

```bash
python3 tools/agentctl.py start mavros
```

启动组合状态估计链路：

```bash
python3 tools/agentctl.py start state_estimation --arg camera_serial_no:=832112071797
```

查询状态：

```bash
python3 tools/agentctl.py status mavros
```

查询健康状态：

```bash
python3 tools/agentctl.py health
```

停止模块：

```bash
python3 tools/agentctl.py stop mavros
```

## 协议格式

TCP 上每个包是一行 JSON，以 `\n` 结尾。

基础包结构：

```json
{
  "protocol_version": "gameuav.agent.v1",
  "message_type": "module_command",
  "source_id": "gcs",
  "target_id": "uav1",
  "sequence_id": "uuid-or-counter",
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

允许的动作：

```text
start
stop
restart
status
list
health
```

## systemd 开机自启

安装：

```bash
cd /home/uav/Desktop/uav_project/gameuav
sudo deploy/systemd/install_gameuav_agent_service.sh
sudo systemctl restart gameuav-agent.service
```

查看状态：

```bash
systemctl status gameuav-agent.service
```

配置文件：

```text
/etc/gameuav/agent.env
```

## 安全规则

- 模块名必须存在于 `config/modules/uav_agent.yaml`。
- launch 参数必须在对应模块的 `allowed_args` 里。
- 每个模块用独立进程组启动。
- 停止流程是 `SIGINT`、`SIGTERM`、必要时 `SIGKILL`。
- 日志写到 `logs/agent/`。
- TCP 命令必须带正确 token。
