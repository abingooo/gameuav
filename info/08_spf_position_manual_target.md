# SPF Position 手工目标发送

本文说明如何向 `/control/spf_position` 手工发布目标，用于复现 SPF 已经生成
目标点之后的控制链路：

```text
/control/spf_position (geometry_msgs/PoseStamped)
    -> gameuav_control_interface
    -> /control/position_cmd (quadrotor_msgs/PositionCommand)
    -> px4ctrl
```

手工发布只验证上述下游链路。它绕过 RGB 图像、VLM 推理、SPF 动作转换和
EGO，因此不能用于证明 SPF 的完整感知、推理或避障能力。

## 1. 执行条件

`/control/spf_position` 只有同时满足以下条件才会产生控制输出：

- `gameuav_control_interface` 和 `px4ctrl` 已启动。
- MAVROS 状态新鲜，且 `connected=true`、`armed=true`。
- 已通过 `/spf/enable` 显式打开 SPF 执行门。
- VINS 和 PX4 姿态数据新鲜，并通过 control interface 姿态检查。

当前飞行器处于 `disarm` 时，打开执行门和发送目标都会被拒绝。这是预期行为。

> 仅在飞行器已经稳定悬停、定位正常且试验区域清空后打开执行门。手工发布
> `/control/spf_position` 不经过 EGO，没有路径避障。

## 2. 准备终端

```bash
cd /home/uav/Desktop/uav_project/gameuav
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

确认订阅节点和飞行状态：

```bash
rostopic info /control/spf_position
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /vins_fusion/imu_propagate/pose/pose
rostopic echo -n 1 /control/interface_status
```

`/control/spf_position` 的订阅者中应包含 `gameuav_control_interface`。发送绝对
目标前，必须记录当前 VINS 位置和朝向。

## 3. 打开 SPF 执行门

必须在 PX4 已解锁且 MAVROS 已连接之后执行：

```bash
rostopic pub -1 /spf/enable std_msgs/Bool "data: true"
```

随后检查状态：

```bash
rostopic echo -n 1 /control/interface_status
```

只有状态中的 `spf_execution_enabled` 为 `true` 才能继续。若先在 disarm 状态
发布 `true`，执行门会保持关闭，解锁后必须重新发布一次。

## 4. 推荐方式：发送世界系绝对目标

SPF bridge 实际发布的是 VINS 世界系中的绝对目标。将下面的
`TARGET_X/TARGET_Y/TARGET_Z` 替换为距离当前 VINS 位置较近的目标值：

```bash
rostopic pub -1 /control/spf_position geometry_msgs/PoseStamped \
"header:
  stamp: now
  frame_id: 'world'
pose:
  position: {x: TARGET_X, y: TARGET_Y, z: TARGET_Z}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

上例中的单位四元数表示世界系 `yaw=0`，并不表示保持当前朝向。若要保持当前
航向，应把 `orientation` 替换为当前 VINS 位姿的四元数。

例如，仅当当前 VINS 位姿接近 `(0.0, 0.0, 1.0)` 且当前世界 yaw 接近 `0` 时，
下面的示例才表示沿世界 `x` 方向移动 `0.2 m`：

```bash
rostopic pub -1 /control/spf_position geometry_msgs/PoseStamped \
"header: {stamp: now, frame_id: 'world'}
pose:
  position: {x: 0.2, y: 0.0, z: 1.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

不要在未读取当前 VINS 坐标时直接使用这个示例值。

## 5. 可选方式：发送机体系相对目标

control interface 也接受相对当前位置的机体系目标：

- `x`：向前。
- `y`：向左。
- `z`：向上。

下面表示相对当前位置向前 `0.2 m`，高度不变：

```bash
rostopic pub -1 /control/spf_position geometry_msgs/PoseStamped \
"header: {stamp: now, frame_id: 'body'}
pose:
  position: {x: 0.2, y: 0.0, z: 0.0}
  orientation: {x: CURRENT_QX, y: CURRENT_QY, z: CURRENT_QZ, w: CURRENT_QW}"
```

必须把 `CURRENT_QX` 至 `CURRENT_QW` 替换为当前 VINS 四元数，才能保持当前
航向。机体系只影响位置增量；`orientation` 仍被解释为世界系绝对 yaw。

## 6. 坐标与限幅规则

| `frame_id` | 位置含义 |
|---|---|
| `world`, `map`, `odom`, `local`, `enu` 或空字符串 | 直接作为同一 VINS 世界系绝对坐标使用，不执行 TF 转换 |
| `body`, `body_enu`, `base_link`, `base`, `ego`, `local_body` | 相对当前位置的 FLU 增量：前、左、上，再按当前 VINS yaw 转到世界系 |

注意：

- control interface 会把目标高度限制到 `0.05-3.0 m`。
- 机体系位置增量的三维长度上限默认为 `3.0 m`。
- 世界系目标没有 XY 距离限幅，错误的绝对坐标可能产生大幅移动。
- 手工直发不会经过 SPF bridge 的水平 `1.5 m`、垂直 `0.3 m` 限幅。
- 手工直发不会经过 EGO，也不会根据障碍物修正目标。
- 默认 `spf_position_timeout=0` 表示没有固定时间超时，但到达释放仍然生效。一条被
  接受的目标会持续发送到满足到达稳定条件，不需要使用 `rostopic pub -r` 循环发布。

## 7. 观察下游输出

在发送目标前分别打开两个终端：

```bash
rostopic echo /control/interface_status
```

```bash
rostopic echo /control/position_cmd
```

目标被接受后，接口状态应进入 `spf_position`，并以约 `50 Hz` 在
`/control/position_cmd` 上持续输出目标位置、零速度和零加速度命令。

默认到达判定要求以下条件连续满足 `0.5 s`：

- XY 误差 `<=0.25 m`。
- Z 误差 `<=0.20 m`。
- yaw 误差 `<=10 deg`。
- 三维线速度 `<=0.25 m/s`。

稳定后状态应变为 `spf_hover_wait`，`/control/position_cmd` 停止产生新消息。
PX4Ctrl 的位置命令超时为 `0.5 s`，因此会从 `CMD_CTRL` 退回 `AUTO_HOVER`。
这时不会恢复缓存的 EGO 轨迹；新的 `/control/spf_position` 才会再次进入
`spf_position` 并恢复命令流。手工单次目标没有自动下一轮，到达后会一直保持悬停。

常见拒绝原因：

- `SPF execution gate is closed`：没有成功发布 `/spf/enable=true`。
- `PX4 is not armed`：飞控未解锁。
- `MAVROS state is unavailable/stale`：MAVROS 未连接或状态超时。
- `VINS/PX4 attitude unavailable/stale`：定位或姿态参考没有更新。
- `invalid frame`：`frame_id` 不在支持列表中。

## 8. 撤销目标

关闭 SPF 执行门即可撤销活动目标并释放 SPF 位置命令：

```bash
rostopic pub -1 /spf/enable std_msgs/Bool "data: false"
```

控制门面会进入 `spf_hover_wait` 并停止 `/control/position_cmd`，PX4Ctrl 在命令超时后
转入 `AUTO_HOVER`。不要为了立即释放控制而继续发送 `/control/stop`：该话题是控制层
驻点请求，会重新产生一个短时 direct-position 命令。需要终止动力时仍应使用遥控器或
飞控的 disarm/安全处置流程。

## 9. 正常 SPF 任务入口

正常 SPF 实验不应手工构造 `/control/spf_position`。当前正式 SPF 链路应打开
执行门后向 `/spf/user_command` 发送自然语言任务，由作者链路生成目标并发布到
`/control/ego_position`，再交给 EGO：

```bash
python3 tools/agentctl.py ros spf_task_enable \
  --arg data:=true \
  --auth-token uavuavuavuav

python3 tools/agentctl.py ros spf_user_command \
  --arg 'data:=fly toward the target chair' \
  --auth-token uavuavuavuav
```

手工目标适用于隔离检查
`control_interface -> /control/position_cmd -> px4ctrl`；正常任务入口才覆盖 SPF
感知、模型推理、动作到目标点的转换以及统一的 EGO 执行链路。

`/spf/user_command` 是单次推理入口，到达后不会自动请求新目标。需要连续执行同一条
语义任务时使用 `/spf/task/start`；局部点满足同一组到达条件后，TaskLoop 进入轮间
延时，上一条 EGO 局部轨迹结束后，TaskLoop 请求下一轮 SPF。作者 SPF 没有任务级
`final`/`done` 输出，最终成功仍由
操作员通过 `/spf/task/control` 的 `complete`/`success` 确认。到达释放属于
GameUAV/PX4Ctrl 控制适配，不能作为作者模型新增能力或语义任务完成证据。连续任务
进入 `SUCCESS/TIMEOUT/ERROR/ABORTED` 后会自动发布 `/spf/enable=false`，撤销活动点
并使迟到的 worker 结果失效；开始下一项任务前需要重新执行第 3 节的 enable 操作。

## 10. 实现位置

- [`control_interface_node.py`](../ros_nodes/control/gameuav_control_interface/scripts/control_interface_node.py#L214)：SPF 目标接收和执行门。
- [`control_interface_node.py`](../ros_nodes/control/gameuav_control_interface/scripts/control_interface_node.py#L351)：持续生成及到达后停止 `PositionCommand`。
- [`control_interface_node.py`](../ros_nodes/control/gameuav_control_interface/scripts/control_interface_node.py#L595)：世界系和机体系转换。
- [`see_point_fly_bridge.py`](../ros_nodes/mission/see_point_fly_bridge/scripts/see_point_fly_bridge.py#L373)：SPF 动作转换为世界系目标。
- [`bringup_flight_control.launch`](../launch/bringup_flight_control.launch#L33)：control interface 到 px4ctrl 的话题连接。
