# Comm Layer Usage

`comm/` 是 GameUAV 的跨机器通信基础层，不依赖 ROS。

## 1. 消息类型

统一网络协议在：

```text
comm/protocol/network_protocol.py
```

核心消息：

```text
heartbeat  UDP best-effort 在线心跳
state      UDP best-effort 状态广播
alert      UDP best-effort 告警
command    TCP reliable 命令
ack        TCP reliable 命令确认
result     TCP reliable 命令结果
error      TCP reliable 错误
```

## 2. UDP

用于高频、允许丢包的数据：

```text
位置
速度
姿态
电量
心跳
告警
```

发送状态：

```python
from comm.udp_link.link import UdpLink

link = UdpLink("0.0.0.0", 0)
link.send_state("uav1", "192.168.1.10", 9001, state={"battery": 0.8})
```

心跳检测：

```python
from comm.heartbeat.monitor import HeartbeatMonitor

monitor = HeartbeatMonitor(timeout_sec=3.0)
monitor.update("uav1")
monitor.is_online("uav1")
```

## 3. TCP

用于可靠命令：

```text
模块控制
任务下发
模式切换
参数修改
```

TCP command 采用：

```text
command -> ack -> result
```

客户端：

```python
from comm.tcp_link.client import TcpCommandClient

client = TcpCommandClient("192.168.1.101", 9100, source_id="gcs", target_id="uav1")
response = client.send_command("set_goal", {"x": 1, "y": 0, "z": 1.2})
```

服务端：

```python
from comm.tcp_link.server import serve_forever

def handle(command, args, message):
    return {"command": command, "args": args}

serve_forever("0.0.0.0", 9100, "uav1", handle)
```

## 4. 测试

```bash
uv run python -m unittest tests.test_comm_network
```
