# 本机 ROS 接口规范

本文档用于冻结 `gameuav` 单机内部 ROS 接口。

ROS 只在单架无人机的机载计算机内部使用。不要把无人机编号写进公共 ROS topic。跨机器身份由 TCP/UDP gateway 消息中的 `uav_id` 或 `target_id` 表达。

## 基本规则

- 公共 ROS topic 是模块之间的稳定契约。
- gateway 只能订阅或发布本文档列出的公共接口。
- 内部 topic 是模块实现细节，GCS、gateway、其他无关模块不要直接依赖。
- 跨 ROS 包边界的 topic 在代码和配置中尽量使用绝对路径。
- EGO Planner 的 `drone_id` 只作为算法内部轨迹编号保留，不是 ROS 命名空间，也不是网络身份。

## 公共 Topic

| Topic | 类型 | 方向 | 所属模块 | 作用 |
|---|---|---:|---|---|
| `/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | 输出 | RealSense | VINS 左目红外图 |
| `/camera/infra2/image_rect_raw` | `sensor_msgs/Image` | 输出 | RealSense | VINS 右目红外图 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 输出 | RealSense | EGO 局部地图深度图 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 输出 | RealSense | 可选彩色图 |
| `/camera/*/camera_info` | `sensor_msgs/CameraInfo` | 输出 | RealSense | 相机内参 |
| `/mavros/state` | `mavros_msgs/State` | 输出 | MAVROS | PX4 连接和模式状态 |
| `/mavros/extended_state` | `mavros_msgs/ExtendedState` | 输出 | MAVROS | 降落/飞行状态 |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | 输出 | MAVROS | PX4 本地位置，供安全检查 |
| `/mavros/statustext/recv` | `mavros_msgs/StatusText` | 输出 | MAVROS | PX4 状态文本 |
| `/mavros/imu/data` | `sensor_msgs/Imu` | 输出 | MAVROS | VINS 和 px4ctrl 使用的 IMU 输入 |
| `/mavros/imu/data_raw` | `sensor_msgs/Imu` | 输出 | MAVROS | 可选原始 IMU 话题；当前 PX4/MAVROS 实测可能无数据 |
| `/mavros/battery` | `sensor_msgs/BatteryState` | 输出 | MAVROS | 电池状态 |
| `/mavros/rc/in` | `mavros_msgs/RCIn` | 输出 | MAVROS | 遥控器输入 |
| `/vins_fusion/imu_propagate` | `nav_msgs/Odometry` | 输出 | VINS | 高频里程计，供规划和控制使用 |
| `/vins_fusion/odometry` | `nav_msgs/Odometry` | 输出 | VINS | 优化后的里程计 |
| `/vins_fusion/path` | `nav_msgs/Path` | 输出 | VINS | 可视化轨迹 |
| `/vins_fusion/extrinsic` | `nav_msgs/Odometry` | 输出 | VINS | EGO 地图使用的外参估计 |
| `/vins_fusion/image_track` | `sensor_msgs/Image` | 输出 | VINS | 特征跟踪可视化 |
| `/vins_fusion/point_cloud` | `sensor_msgs/PointCloud` | 输出 | VINS | 特征点云 |
| `/vins_restart` | `std_msgs/Bool` | 输入 | VINS | 重置 VINS |
| `/vins_imu_switch` | `std_msgs/Bool` | 输入 | VINS | 开关 IMU |
| `/vins_cam_switch` | `std_msgs/Bool` | 输入 | VINS | 开关双目相机 |
| `/planning/goal` | `geometry_msgs/PoseStamped` | 输入 | EGO Planner | 任务目标点输入 |
| `/planning/goal_yaw_deg` | `std_msgs/Float64` | 输入 | EGO Planner | 可选目标 yaw，单位度 |
| `/position_cmd` | `quadrotor_msgs/PositionCommand` | 输出 | EGO Planner | 给 px4ctrl 的位置控制指令 |
| `/control/ego_position` | `geometry_msgs/PoseStamped` | 输入 | control interface | 机体系/世界系目标，转换为 `/planning/goal` 交给 EGO 规划 |
| `/control/position` | `geometry_msgs/PoseStamped` | 输入 | control interface | 直接位置目标，不经过 EGO 避障规划 |
| `/control/speed` | `geometry_msgs/TwistStamped` | 输入 | control interface | 短时速度命令，积分成 px4ctrl 位置指令 |
| `/control/stop` | `std_msgs/Empty` | 输入 | control interface | 按当前里程计位置悬停 |
| `/control/interface_status` | `std_msgs/String` | 输出 | control interface | JSON 状态，包含当前模式和拒绝原因 |
| `/control/ego_position_cmd` | `quadrotor_msgs/PositionCommand` | 输入 | control interface | realflight/egoctrl 中 EGO 轨迹命令的重映射输入 |
| `/control/spf_position` | `geometry_msgs/PoseStamped` | 输入 | control interface | SPF 持续直控位置目标，不经过 EGO |
| `/control/position_cmd` | `quadrotor_msgs/PositionCommand` | 输出 | control interface | realflight/egoctrl 中给 px4ctrl 的仲裁后指令 |
| `/px4ctrl/takeoff_land` | `quadrotor_msgs/TakeoffLand` | 输入 | px4ctrl | 起飞/降落命令 |
| `/px4ctrl/hover_yaw_cmd` | `std_msgs/Float64` | 输入 | px4ctrl | 悬停 yaw 命令，也可由 EGO 发布 |
| `/traj_start_trigger` | `geometry_msgs/PoseStamped` | 输出 | px4ctrl | 预设轨迹模式触发 |
| `/debugPx4ctrl` | `quadrotor_msgs/Px4ctrlDebug` | 输出 | px4ctrl | 控制器调试状态 |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | 输入 | MAVROS | px4ctrl 发给 PX4 的姿态/油门 setpoint |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | 输入 | MAVROS | px4ctrl 发给 PX4 的本地位置 setpoint |
| `/actuation/tiplight_cmd` | `std_msgs/String` | 输入 | tiplight | LED 指令 |
| `/status/tiplight` | `std_msgs/String` | 输出 | tiplight | LED 串口/状态反馈 |

## 公共 Service

| Service | 类型 | 调用方 | 作用 |
|---|---|---|---|
| `/mavros/set_mode` | `mavros_msgs/SetMode` | px4ctrl | 切换 PX4 模式 |
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | px4ctrl | 解锁/上锁 |
| `/mavros/cmd/command` | `mavros_msgs/CommandLong` | px4ctrl | PX4 long command，例如重启 |

## EGO Planner 边界

EGO Planner 对外只暴露以下接口：

| 方向 | 公共 topic |
|---|---|
| 目标点输入 | `/planning/goal` |
| 可选 yaw 输入 | `/planning/goal_yaw_deg` |
| 控制指令输出 | `/position_cmd` |

以下是内部接口，不给 gateway/GCS 直接使用。`drone_0` 前缀保留是为了兼容 EGO Planner 原始代码里的内部轨迹编号，不表示网络无人机身份。

| 内部 topic | 说明 |
|---|---|
| `/drone_0_planning/bspline` | planner 到 traj_server 的 B-spline |
| `/drone_0_planning/data_display` | 规划可视化/调试 |
| `/broadcast_bspline` | EGO 内部 B-spline 广播 |
| `/drone_0_planning/swarm_trajs` | EGO 原始 swarm trajectory 缓冲 |
| `/drone_*` | EGO 原始仿真/多机兼容接口 |

## 控制门面

UAV 侧控制门面暴露三层控制接口：

| 命令 | ROS 输入 | 行为 |
|---|---|---|
| `ego_position` | `/control/ego_position` | 把机体系或世界系位姿转换成 `/planning/goal`，由 EGO 继续负责避障和轨迹生成。 |
| `position` | `/control/position` | 直接生成 `PositionCommand` 给 px4ctrl，不做障碍物规划。 |
| SPF position | `/control/spf_position` | SPF 专用持续位置目标，直接生成 `PositionCommand` 给 px4ctrl，直到新目标或其他直控命令接管。 |
| `speed` | `/control/speed` | 把限幅后的速度短时积分成 `PositionCommand`，适合手动点动，必须持续刷新。 |

在 `realflight`/`egoctrl` 中，EGO 的 `/position_cmd` 会重映射到
`/control/ego_position_cmd`，px4ctrl 订阅 `/control/position_cmd`。控制门面默认透传 EGO；
当收到 direct `position` 或 `speed` 时临时接管，超时后回到 EGO 透传。SPF 使用
独立的 `/control/spf_position` 持续接管路径，不会把目标发布给 EGO。

## Gateway 映射

gateway 应该收发语义消息，再转换成本机 ROS 接口：

| 网络消息 | 本机 ROS 接口 |
|---|---|
| `MissionCommand(set_goal)` | 发布 `/planning/goal` |
| `MissionCommand(ego_position)` | 发布 `/control/ego_position` |
| `FlightCommand(position)` | 发布 `/control/position` |
| `FlightCommand(speed)` | 发布 `/control/speed` |
| `FlightCommand(stop)` | 发布 `/control/stop` |
| `MissionCommand(set_yaw)` | 发布 `/planning/goal_yaw_deg` |
| `FlightCommand(takeoff/land)` | 发布 `/px4ctrl/takeoff_land` |
| `ActuatorCommand(tiplight)` | 发布 `/actuation/tiplight_cmd` |
| `UavState` | 订阅 `/vins_fusion/imu_propagate`、`/mavros/state`、`/mavros/battery` |
| `ControllerState` | 订阅 `/debugPx4ctrl`、`/control/interface_status` |
