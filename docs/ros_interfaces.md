# Local ROS Interfaces

This document freezes the single-UAV ROS interface for `gameuav`.

ROS is local to one onboard computer. Do not encode UAV identity in ROS topic
names. Cross-machine identity belongs to TCP/UDP gateway messages as `uav_id`
or `target_id`.

## Rules

- Public ROS topics are stable contracts between onboard modules.
- Gateway code may subscribe or publish only public topics listed here.
- Internal topics are implementation details of a module and should not be used
  by GCS, gateway, or unrelated modules.
- Topic names are absolute in code/config when they cross package boundaries.
- EGO Planner keeps `drone_id` only as an internal trajectory id for algorithm
  compatibility. It is not a ROS namespace or network identity.

## Public Topics

| Topic | Type | Direction | Owner | Purpose |
|---|---|---:|---|---|
| `/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | out | RealSense | Left infrared image for VINS |
| `/camera/infra2/image_rect_raw` | `sensor_msgs/Image` | out | RealSense | Right infrared image for VINS |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | out | RealSense | Depth image for local mapping |
| `/camera/color/image_raw` | `sensor_msgs/Image` | out | RealSense | Optional color image |
| `/camera/*/camera_info` | `sensor_msgs/CameraInfo` | out | RealSense | Camera calibration |
| `/mavros/state` | `mavros_msgs/State` | out | MAVROS | PX4 connection and mode state |
| `/mavros/extended_state` | `mavros_msgs/ExtendedState` | out | MAVROS | Landed/flying state |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | out | MAVROS | PX4 local pose used for safety checks |
| `/mavros/statustext/recv` | `mavros_msgs/StatusText` | out | MAVROS | PX4 status text |
| `/mavros/imu/data` | `sensor_msgs/Imu` | out | MAVROS | IMU input for VINS and px4ctrl |
| `/mavros/imu/data_raw` | `sensor_msgs/Imu` | out | MAVROS | Optional raw IMU topic; may be unavailable on current PX4/MAVROS setup |
| `/mavros/battery` | `sensor_msgs/BatteryState` | out | MAVROS | Battery state |
| `/mavros/rc/in` | `mavros_msgs/RCIn` | out | MAVROS | RC input when RC is enabled |
| `/vins_fusion/imu_propagate` | `nav_msgs/Odometry` | out | VINS | High-rate odometry for planning/control |
| `/vins_fusion/odometry` | `nav_msgs/Odometry` | out | VINS | Optimized odometry |
| `/vins_fusion/path` | `nav_msgs/Path` | out | VINS | Visualization path |
| `/vins_fusion/extrinsic` | `nav_msgs/Odometry` | out | VINS | Camera/IMU extrinsic estimate for EGO map |
| `/vins_fusion/image_track` | `sensor_msgs/Image` | out | VINS | Feature tracking visualization |
| `/vins_fusion/point_cloud` | `sensor_msgs/PointCloud` | out | VINS | Tracked feature cloud |
| `/vins_restart` | `std_msgs/Bool` | in | VINS | Reset VINS estimator |
| `/vins_imu_switch` | `std_msgs/Bool` | in | VINS | Enable/disable IMU usage |
| `/vins_cam_switch` | `std_msgs/Bool` | in | VINS | Enable/disable stereo camera usage |
| `/planning/goal` | `geometry_msgs/PoseStamped` | in | EGO Planner | Mission goal input |
| `/planning/goal_yaw_deg` | `std_msgs/Float64` | in | EGO Planner | Optional goal yaw in degrees |
| `/position_cmd` | `quadrotor_msgs/PositionCommand` | out | EGO Planner | Position command for px4ctrl |
| `/control/ego_position` | `geometry_msgs/PoseStamped` | in | control interface | Body/world goal converted to `/planning/goal` for EGO planning |
| `/control/position` | `geometry_msgs/PoseStamped` | in | control interface | Direct position target, bypassing EGO obstacle planning |
| `/control/speed` | `geometry_msgs/TwistStamped` | in | control interface | Short-lived speed command, integrated into px4ctrl position commands |
| `/control/stop` | `std_msgs/Empty` | in | control interface | Hold current odometry position |
| `/control/interface_status` | `std_msgs/String` | out | control interface | JSON status for command mode and rejection details |
| `/control/ego_position_cmd` | `quadrotor_msgs/PositionCommand` | in | control interface | Remapped EGO trajectory command input in realflight/egoctrl |
| `/control/spf_position` | `geometry_msgs/PoseStamped` | in | control interface | SPF direct position target; bypasses EGO and is released after the configured arrival condition settles |
| `/control/position_cmd` | `quadrotor_msgs/PositionCommand` | out | control interface | Muxed command output to px4ctrl in realflight/egoctrl |
| `/px4ctrl/takeoff_land` | `quadrotor_msgs/TakeoffLand` | in | px4ctrl | Takeoff/land command |
| `/px4ctrl/hover_yaw_cmd` | `std_msgs/Float64` | in | px4ctrl | Hover yaw command, also published by EGO |
| `/traj_start_trigger` | `geometry_msgs/PoseStamped` | out | px4ctrl | Trigger for preset EGO trajectory mode |
| `/debugPx4ctrl` | `quadrotor_msgs/Px4ctrlDebug` | out | px4ctrl | Controller debug state |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | in | MAVROS | Attitude/thrust setpoint from px4ctrl |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | in | MAVROS | Local position setpoint from px4ctrl |
| `/actuation/tiplight_cmd` | `std_msgs/String` | in | tiplight | LED command |
| `/status/tiplight` | `std_msgs/String` | out | tiplight | LED serial/status feedback |

## Public Services

| Service | Type | Caller | Purpose |
|---|---|---|---|
| `/mavros/set_mode` | `mavros_msgs/SetMode` | px4ctrl | Switch PX4 mode |
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | px4ctrl | Arm/disarm PX4 |
| `/mavros/cmd/command` | `mavros_msgs/CommandLong` | px4ctrl | PX4 long commands, including reboot |

## EGO Planner Boundary

EGO Planner public interface is intentionally small:

| Direction | Public topic |
|---|---|
| Goal input | `/planning/goal` |
| Optional yaw input | `/planning/goal_yaw_deg` |
| Control output | `/position_cmd` |

The following topics are internal and must not be used by gateway/GCS. The
`drone_0` prefix is kept because EGO Planner's original code uses it as an
internal trajectory id. It is not the network UAV identity.

| Internal topic | Reason |
|---|---|
| `/drone_0_planning/bspline` | Planner-to-trajectory-server B-spline |
| `/drone_0_planning/data_display` | Planner visualization/debug |
| `/broadcast_bspline` | EGO internal B-spline broadcast |
| `/drone_0_planning/swarm_trajs` | Legacy EGO swarm trajectory buffer |
| `/drone_*` | Legacy EGO simulation/swarm compatibility only |

## Control Interface

The UAV-side control facade exposes three command levels:

| Command | ROS input | Behavior |
|---|---|---|
| `ego_position` | `/control/ego_position` | Converts a body-frame or world-frame pose into `/planning/goal`; EGO Planner keeps obstacle-aware trajectory generation. |
| `position` | `/control/position` | Directly drives px4ctrl through `PositionCommand`; it does not perform obstacle planning. |
| SPF position | `/control/spf_position` | SPF position target converted directly to px4ctrl `PositionCommand` until superseded or the configured arrival condition settles. |
| `speed` | `/control/speed` | Integrates bounded velocity into a short-lived `PositionCommand`; refresh it continuously for manual jogging. |

In `realflight`/`egoctrl`, EGO's `/position_cmd` output is remapped to
`/control/ego_position_cmd`, and px4ctrl subscribes to `/control/position_cmd`.
The control interface passes EGO commands through unless a direct `position` or
`speed` command is active. SPF uses the persistent `/control/spf_position` path
and does not publish its target to EGO.

With the real-flight defaults, an SPF target is considered settled only while
XY error is at most `0.25 m`, Z error is at most `0.20 m`, yaw error is at most
`10 deg`, and three-dimensional linear speed is at most `0.25 m/s` continuously
for `0.5 s`. The control interface then enters `spf_hover_wait`, clears cached
SPF/EGO motion commands, and stops publishing `/control/position_cmd`. PX4Ctrl's
configured `0.5 s` command timeout consequently changes `CMD_CTRL` to
`AUTO_HOVER`. A newly accepted SPF target resumes command publication and
returns PX4Ctrl to `CMD_CTRL`.

A one-shot `/spf/user_command` or a manual `/control/spf_position` therefore
leaves the vehicle in hover after arrival. A continuous `/spf/task/start` loop
requests the next SPF cycle after its inter-cycle delay. The upstream SPF model
does not emit a task-level `final` or `done` result, so local-goal arrival is not
semantic task success. Arrival release is GameUAV/PX4Ctrl integration behavior,
not an added SPF model capability. Every terminal continuous-task result closes
the shared `/spf/enable` gate, invalidating the active point and any late worker
response; explicitly enable SPF again before starting another task.

## Gateway Mapping

Gateway should map network messages to semantic local ROS interfaces:

| Network message | ROS interface |
|---|---|
| `MissionCommand(set_goal)` | publish `/planning/goal` |
| `MissionCommand(ego_position)` | publish `/control/ego_position` |
| `FlightCommand(position)` | publish `/control/position` |
| `FlightCommand(speed)` | publish `/control/speed` |
| `FlightCommand(stop)` | publish `/control/stop` |
| `MissionCommand(set_yaw)` | publish `/planning/goal_yaw_deg` |
| `FlightCommand(takeoff/land)` | publish `/px4ctrl/takeoff_land` |
| `ActuatorCommand(tiplight)` | publish `/actuation/tiplight_cmd` |
| `UavState` | subscribe `/vins_fusion/imu_propagate`, `/mavros/state`, `/mavros/battery` |
| `ControllerState` | subscribe `/debugPx4ctrl`, `/control/interface_status` |
