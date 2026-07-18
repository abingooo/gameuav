# Gateway Usage

`gateway/` 负责 ROS 和网络协议之间的转换。

核心原则：

```text
ROS topic 只在无人机本机内部使用
跨机器网络只传协议化消息
```

## 1. ros_to_net_gateway

方向：

```text
ROS topic -> UDP state
```

第一版订阅：

```text
/vins_fusion/imu_propagate
/mavros/battery
/mavros/state
```

启动示例：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

python3 -m gateway.ros_to_net_gateway.node \
  --uav-id uav1 \
  --udp-host 192.168.1.10 \
  --udp-port 9001 \
  --rate-hz 5
```

其中 `192.168.1.10:9001` 是地面站或监听端。

## 2. net_to_ros_gateway

方向：

```text
TCP command -> ROS topic / safe agent command
```

启动示例：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

python3 -m gateway.net_to_ros_gateway.node \
  --host 0.0.0.0 \
  --port 9100 \
  --uav-id uav1
```

第一版支持网络命令：

```text
set_goal
ego_position / ego-position
position
speed
stop
tiplight
takeoff -> safe_takeoff
land    -> safe_land
```

## 3. 映射配置

参考：

```text
config/topics/ros_interfaces.yaml
```

其中 `gateway_mapping` 描述哪些 ROS topic 可以进出网络。

## 4. 测试

```bash
uv run python -m unittest tests.test_gateway
```

## 5. systemd 部署

部署文件：

```text
deploy/systemd/gameuav-gateway.env
deploy/systemd/gameuav-ros-to-net-gateway.service
deploy/systemd/gameuav-net-to-ros-gateway.service
deploy/systemd/install_gameuav_gateway_services.sh
```

安装：

```bash
cd /home/uav/Desktop/uav_project/gameuav
sudo deploy/systemd/install_gameuav_gateway_services.sh
```

配置文件会安装到：

```text
/etc/gameuav/gateway.env
```

关键配置：

```text
GAMEUAV_GATEWAY_UDP_HOST=地面站IP
GAMEUAV_GATEWAY_UDP_PORT=9001
GAMEUAV_GATEWAY_TCP_HOST=0.0.0.0
GAMEUAV_GATEWAY_TCP_PORT=9100
GAMEUAV_AGENT_HOST=127.0.0.1
GAMEUAV_AGENT_PORT=8765
```

当前在无人机本机调试时，`GAMEUAV_GATEWAY_UDP_HOST` 可以先设为 `127.0.0.1`。

最终部署时：

```text
GAMEUAV_GATEWAY_UDP_HOST=地面站服务器IP
```

启动：

```bash
sudo systemctl restart gameuav-ros-to-net-gateway.service
sudo systemctl restart gameuav-net-to-ros-gateway.service
```

查看：

```bash
systemctl status gameuav-ros-to-net-gateway.service --no-pager
systemctl status gameuav-net-to-ros-gateway.service --no-pager
```

停止：

```bash
sudo systemctl stop gameuav-ros-to-net-gateway.service
sudo systemctl stop gameuav-net-to-ros-gateway.service
```

注意：

```text
1. ros_to_net_gateway 需要 ROS master 和相关 topic 存在，否则会反复重启或等待 ROS。
2. net_to_ros_gateway 需要 ROS 环境，但不要求所有业务 topic 已存在。
3. takeoff/land 通过本机 uav_agent 的 safe_takeoff/safe_land 执行，不走裸起降 topic。
```
