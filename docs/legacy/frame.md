# GameUAV Architecture Frame

## 1. Design Goal

GameUAV is a general multi-UAV coordination system.

The core principle is:

```text
Inside one UAV: ROS communication
Between UAVs and GCS: TCP/UDP communication
Between ROS and network: gateway translation
```

The system should not depend on any temporary algorithm such as ADV or MPC.
ADV, MPC, rule-based control, and formation control are strategy plugins.

## 2. Top-Level Structure

```text
gameuav/
  frame.md

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

## 3. Module Responsibilities

### 3.1 ros_nodes

`ros_nodes/` contains the UAV-local ROS modules.

These modules communicate through local ROS topics, services, and actions only.
They should not directly open TCP/UDP sockets for multi-machine coordination.

```text
ros_nodes/perception/
```

Camera, lidar, point cloud processing, target detection, obstacle detection.

```text
ros_nodes/state_estimation/
```

VINS, SLAM, GPS/IMU fusion, odometry, pose, velocity, attitude estimation.

```text
ros_nodes/planning/
```

Global planning, local planning, obstacle avoidance, trajectory generation.

```text
ros_nodes/control/
```

Position control, velocity control, attitude control, trajectory tracking,
flight-controller adapters.

```text
ros_nodes/actuation/
```

Non-flight actuators.

Examples:

```text
LED
buzzer
servo
gripper
relay
gimbal
```

Flight control stays in `control/`. Payload and auxiliary actuators stay in
`actuation/`.

```text
ros_nodes/safety/
```

Emergency stop, lost-link handling, low-battery handling, boundary protection,
collision-risk handling, hover/return/land fallback.

```text
ros_nodes/mission/
```

Mission state machine, mission command handling, task progress, mission pause,
resume, abort, and completion judgment.

### 3.2 agent

`agent/` is the UAV-local system management layer.

It is the entry point used by the ground station to manage a UAV's ROS system.
It should run as a normal background process, preferably started by systemd.

The agent should not require ROS master to be alive. Otherwise it cannot start
`roscore` or recover a broken ROS system.

```text
agent/uav_agent/
```

Long-running TCP command receiver on every UAV.

Responsibilities:

```text
receive GCS commands
authenticate/check commands
dispatch commands to launch_manager or ros_command_executor
return command ack/status/result
```

```text
agent/launch_manager/
```

Starts, stops, restarts, and monitors local ROS processes.

Examples:

```text
roscore
mavros
vins
px4ctrl
ego_planner
strategy plugin process
```

It should use a whitelist from `config/modules/`.
The GCS must not send arbitrary shell commands.

```text
agent/ros_command_executor/
```

Executes controlled ROS runtime commands after ROS is available.

Examples:

```text
publish takeoff command
publish land command
publish mission goal
call mode-switch service
call parameter service
```

```text
agent/health_monitor/
```

Checks local system status.

Examples:

```text
process alive
roscore reachable
required rosnode exists
required rostopic exists
topic has recent messages
CPU/memory/disk/network status
```

```text
agent/log_streamer/
```

Streams or uploads logs to the GCS.

### 3.3 comm

`comm/` is the pure network communication layer.

It should be reusable and should not contain business logic such as VINS,
EGO, ADV, or MPC.

```text
comm/protocol/
```

Defines network-level message contracts.

Every network packet should include at least:

```text
protocol_version
message_type
source_id
target_id
sequence_id
timestamp
payload
checksum or signature
```

Common message types:

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

```text
comm/udp_link/
```

UDP transport for high-frequency and loss-tolerant data.

Use cases:

```text
heartbeat
UAV position
velocity
attitude
neighbor state
link-quality report
```

```text
comm/tcp_link/
```

TCP transport for reliable data.

Use cases:

```text
start/stop module
takeoff/land command
mission upload
parameter update
mode switch
log upload
map or file transfer
```

```text
comm/serializer/
```

Converts internal message objects to bytes and bytes back to objects.

Candidate formats:

```text
JSON for early debugging
MessagePack for compact runtime messages
Protobuf for stable cross-language contracts
FlatBuffers for zero-copy or high-performance use
custom binary only when necessary
```

Recommended migration path:

```text
phase 1: JSON
phase 2: Protobuf or MessagePack
phase 3: optimized binary only for high-rate data if needed
```

```text
comm/heartbeat/
```

Online/offline detection.

Recommended default:

```text
send heartbeat every 0.5 s
warn if no heartbeat for 1.5 s
declare lost if no heartbeat for 3.0 s
notify safety module after lost-link detection
```

```text
comm/qos/
```

Communication quality control.

Responsibilities:

```text
rate limiting
priority queue
timeout
ack/retry
drop stale message
latency statistics
packet loss statistics
```

```text
comm/router/
```

Routes network messages.

Examples:

```text
send to one UAV
send to all UAVs
send to GCS
broadcast to team
ignore messages not targeting this UAV
```

### 3.4 gateway

`gateway/` converts between local ROS messages and network messages.

This is the key isolation layer.

ROS topics should not be exposed directly across machines. The gateway decides
which ROS information is allowed to leave the UAV and which network commands
are allowed to enter ROS.

```text
gateway/ros_to_net_gateway/
```

Subscribes to local ROS topics and publishes network messages.

Example:

```text
/vins_position
  -> UavState network packet
  -> UDP to GCS and other UAVs
```

```text
gateway/net_to_ros_gateway/
```

Receives network messages and publishes/calls local ROS interfaces.

Example:

```text
MissionCommand from TCP
  -> /mission/command
```

```text
gateway/topic_mapping/
```

Configuration and code for mapping ROS topics to network messages.

Example mapping:

```text
/vins_position              -> UavState
/mavros/battery             -> BatteryState
/px4ctrl/takeoff_land       <- TakeoffLand command
/planning/goal              <- Mission goal command
/actuation/led_cmd          <- ActuatorCommand
```

```text
gateway/message_adapter/
```

Message conversion code.

Examples:

```text
nav_msgs/Odometry -> UavState
geometry_msgs/PointStamped -> UavState
UavState -> local tracking topic
MissionCommand -> geometry_msgs/PoseStamped
TakeoffCommand -> quadrotor_msgs/TakeoffLand
ActuatorCommand -> LED/servo/gripper command topic
```

### 3.5 gcs

`gcs/` is the ground control station.

The GCS should send semantic commands. It should not directly publish arbitrary
UAV-local ROS topics over the network.

```text
gcs/backend/
```

Fleet connection management, command API, database/log API, state aggregation.

```text
gcs/frontend/
```

Map UI, vehicle table, mission editor, alerts, manual control UI.

```text
gcs/monitor/
```

Monitors:

```text
online/offline
battery
pose
velocity
flight mode
mission state
module state
network quality
safety events
```

```text
gcs/command/
```

Builds and sends commands:

```text
start module
stop module
takeoff
land
return
hover
set goal
start mission
pause mission
resume mission
abort mission
manual override
```

### 3.6 strategy

`strategy/` contains replaceable coordination algorithms.

ADV and MPC are not platform foundations. They are plugins.

```text
strategy/interface/
```

Defines the common strategy interface.

Recommended input:

```text
FleetState
MissionState
EnvironmentState
StrategyConfig
```

Recommended output:

```text
RoleAssignment
TrajectoryIntent
ControlCommand
MissionDecision
```

```text
strategy/plugins/adv/
strategy/plugins/mpc/
strategy/plugins/rule_based/
strategy/plugins/formation/
```

Each plugin should depend on the common strategy interface, not on raw network
transport or arbitrary ROS topics.

### 3.7 config

`config/` contains all runtime configuration.

```text
config/fleet/
```

UAV ID, name, role, IP, ports, capabilities.

Example:

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

```text
config/network/
```

UDP/TCP ports, broadcast address, retry timeout, heartbeat timeout.

```text
config/topics/
```

ROS topic names for each UAV.

```text
config/frames/
```

Coordinate frames and transformations.

Examples:

```text
world
map
odom
base_link
local origin of each UAV
local-to-world yaw and translation
```

```text
config/mission/
```

Mission parameters, waypoints, safety boundary, no-fly zone, task-specific
parameters.

```text
config/modules/
```

Whitelist of modules that `uav_agent` is allowed to start/stop.

Example:

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

## 4. Runtime Architecture

### 4.1 UAV Local Runtime

```text
uav_agent
  |
  | start/stop/check
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

UAV-local data flow:

```text
perception
  -> state_estimation
  -> mission
  -> strategy
  -> planning
  -> control
  -> actuation when payload output is needed
  -> vehicle/flight controller
```

### 4.2 Cross-Machine Runtime

```text
UAV ROS topics
  -> ros_to_net_gateway
  -> comm udp/tcp
  -> GCS or other UAV

GCS or other UAV
  -> comm udp/tcp
  -> net_to_ros_gateway
  -> UAV ROS topics/services
```

Do not synchronize the full ROS graph across machines.
Only export controlled, system-level messages.

## 5. GCS Control Model

The GCS controls a UAV through `uav_agent` and `gateway`.

There are two command categories.

### 5.1 Process-Level Commands

These commands manage ROS processes.

Examples:

```text
start roscore
start mavros
start vins
start px4ctrl
start ego
stop ego
restart vins
query module status
stream module logs
```

Path:

```text
GCS
  -> TCP ModuleCommand
  -> uav_agent
  -> launch_manager
  -> subprocess/systemd/roslaunch
  -> ModuleStatus ack
  -> GCS
```

Example command:

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

Example response:

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

### 5.2 ROS Runtime Commands

These commands operate on an already running ROS system.

Examples:

```text
takeoff
land
hover
set goal
pause mission
resume mission
abort mission
manual override
```

Path:

```text
GCS
  -> TCP MissionCommand or ControlCommand
  -> UAV net_to_ros_gateway
  -> local ROS topic/service/action
  -> controller/mission/safety
```

Example takeoff command:

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

Local ROS output:

```text
topic: /px4ctrl/takeoff_land
type: quadrotor_msgs/TakeoffLand
payload: takeoff_land_cmd = 1
```

Example mission goal:

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

Local ROS output:

```text
topic: /planning/goal
type: geometry_msgs/PoseStamped
```

## 6. uav_agent Implementation Rules

`uav_agent` is not an arbitrary shell executor.

It is a controlled local UAV management agent.

Required rules:

```text
1. Run before ROS starts.
2. Listen on a fixed TCP control port.
3. Accept only whitelisted module names and command types.
4. Never execute arbitrary command strings from GCS.
5. Start launch processes in their own process groups.
6. Store pid, start time, log path, and current status.
7. Return ack for every request_id.
8. Support graceful stop before force stop.
9. Expose health and logs to GCS.
10. Notify safety/gateway when critical module status changes.
```

Recommended internal model:

```text
uav_agent
  command_server
  auth/checker
  launch_manager
  ros_command_executor
  health_monitor
  log_streamer
```

Recommended start order:

```text
1. uav_agent starts by systemd
2. GCS sends start roscore
3. GCS sends start mavros
4. GCS sends start vins
5. GCS sends start px4ctrl
6. GCS sends start ego_planner
7. GCS sends start gateway and mission modules
8. GCS sends takeoff or mission command
```

## 7. Communication Policy

Use UDP for:

```text
heartbeat
UavState
neighbor pose/velocity/attitude
high-frequency telemetry
link quality
```

Use TCP for:

```text
module command
mission command
control command requiring ack
parameter update
mode switch
log upload
file transfer
configuration sync
```

For UDP data:

```text
include sequence_id
include timestamp
drop stale packets
do not wait for retransmission for high-rate telemetry
```

For TCP commands:

```text
include request_id
return ack
return final status when command completes or fails
apply timeout
make commands idempotent where possible
```

## 8. Safety Policy

Safety must be local to the UAV.

The UAV must not depend on the GCS to remain safe.

Minimum local safety actions:

```text
lost GCS link -> continue mission or hover according to policy
lost neighbor link -> degrade multi-UAV coordination
lost localization -> hover/land
low battery -> return/land
out of bounds -> stop mission and return/land
collision risk -> avoid/hover
manual emergency stop -> immediate safe action
```

The `safety` module should subscribe to local health, heartbeat status, battery,
localization quality, and mission state.

## 9. Strategy Plugin Policy

ADV and MPC should be implemented as plugins.

They must not become hard dependencies of the platform.

Strategy plugin input:

```text
FleetState
MissionState
EnvironmentState
StrategyConfig
```

Strategy plugin output:

```text
RoleAssignment
TrajectoryIntent
ControlCommand
MissionDecision
```

The strategy manager chooses the active plugin from configuration:

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

The rest of the system should not know whether the active strategy is ADV, MPC,
or rule-based.

## 10. Migration From Current muav

The current `../muav` project can be migrated by role.

Suggested mapping:

```text
core/src/realflight_modules/realsense-ros
  -> ros_nodes/perception/

core/src/realflight_modules/VINS-Fusion
  -> ros_nodes/state_estimation/

core/src/planner/plan_manage and related EGO planner packages
  -> ros_nodes/planning/

core/src/realflight_modules/px4ctrl
  -> ros_nodes/control/

core/src/groundctrl
  -> gcs/backend/, gcs/monitor/, gcs/command/

core/src/swarm_position_bridge
  -> gateway/ros_to_net_gateway/ and message_adapter/

core/src/planner/rosmsg_tcp_bridge
  -> reference only; replace with comm/ and gateway/ design

adv/src/adv
  -> strategy/plugins/adv/

mpc/src/mpc
  -> strategy/plugins/mpc/

core/shfiles/*.sh
  -> config/modules/ and agent/launch_manager/

core/src/utils/quadrotor_msgs, uav_utils, pose_utils, cmake_utils,
catkin_simple, DecompROS/decomp_ros_msgs, DecompROS/decomp_ros_utils
  -> ros_nodes/common/ and linked into root src/
```

Recommended migration order:

```text
1. Define config/fleet, config/network, config/topics.
2. Implement uav_agent with start/stop/status for roscore and current launch files.
3. Implement TCP ModuleCommand path from GCS to uav_agent.
4. Implement heartbeat and UavState over UDP.
5. Implement ros_to_net_gateway for local state telemetry.
6. Implement net_to_ros_gateway for takeoff, land, and set_goal.
7. Move ADV/MPC behind strategy/interface.
8. Reduce or remove fkie_master_sync and direct cross-machine ROS publishing.
9. Add safety rules for lost link and module failure.
10. Add integration tests and simulation replay.
```

## 11. Minimal First Milestone

The first working version should include only:

```text
uav_agent
GCS command backend
TCP start/stop/status module command
UDP heartbeat
UDP UavState
net_to_ros_gateway for takeoff/land/set_goal
ros_to_net_gateway for odom/battery/module status
basic safety lost-link policy
```

This is enough to prove the architecture before migrating all modules.

## 12. Core Summary

```text
ros_nodes = UAV-local ROS functions
agent = UAV-local ROS process and command manager
comm = TCP/UDP protocol and transport
gateway = ROS/network translation
gcs = ground control station
strategy = replaceable coordination algorithms
config = fleet/network/topic/frame/module configuration
deploy/docs/tests/tools = engineering support
```

The final rule:

```text
GCS sends semantic commands.
UAV agent and gateway convert commands locally.
ROS stays local to each UAV.
TCP/UDP is the only cross-machine communication layer.
Safety decisions stay local on each UAV.
```
