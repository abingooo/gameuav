# See-Point-Fly GameUAV Adapter

This directory keeps third-party SPF code separate from GameUAV integration code.

Layout:

- `upstream/`: clone of `https://github.com/Hu-chih-yao/see-point-fly.git`.
- `worker/`: isolated process boundary for SPF inference.
- `adapter/config_tello.yaml`: local endpoint, model, and author-mode selection.
- `UPSTREAM_SOURCE.json`: exact author repository, branch, and commit record.
- `REPRODUCTION_STATUS.md`: paper task boundary, real-world five-category
  evidence matrix, and current verification status.

The upstream snapshot is synchronized without local source modifications to
`main` commit `5621bcf43e9826d60df014541dd0498e743a92bd`.

The active comparison manifest under `../smpf/experiments/` contains the 11
real-world tasks. The 23-task DRL manifest is retained only as an author-source
reference; Search is outside the current real-world-only scope.

The ROS bridge runs in the GameUAV ROS Python 3.8 environment. The SPF worker is
intended to run in its own `uv` environment because upstream SPF requires a newer
Python than ROS Noetic.

Current integration policy:

- SPF does not publish MAVROS attitude/thrust setpoints directly.
- SPF output is treated as a relative action suggestion.
- The ROS bridge clamps the suggestion and publishes `/control/ego_position`.
- The control interface forwards that target to `/planning/goal`; EGO owns
  trajectory generation and its output reaches px4ctrl through
  `/control/ego_position_cmd` and `/control/position_cmd`. The SPF startup
  entry is `egoctrl_nomap`, which fixes EGO to `free_space`: B-spline and
  dynamic constraints remain active, but depth/point-cloud obstacle mapping is
  not subscribed or used. SMPF instead uses the mapped `egoctrl` entry.
- While XY error is at most `0.25 m`, Z error at most `0.20 m`, yaw error at
  most `10 deg`, and three-dimensional speed at most `0.25 m/s` continuously
  for `0.5 s`, the target is settled. The task loop then requests the next SPF
  action after its inter-cycle delay. EGO finishes each local trajectory and
  px4ctrl returns to hover when its trajectory command stream ends.
- The bridge rejects every real goal publication unless the shared SPF session
  gate is explicitly enabled and MAVROS state is fresh, connected, and armed.
  The continuous task executor uses the same gate and additionally requires an
  already armed, hovering vehicle.
- Endpoint occupancy projection is disabled by default so the SPF baseline does
  not borrow EGO perception or planning. Enabling it creates an explicit local
  safety variant, not the author baseline.

## Author Reproduction Boundary

The author repository exposes one iterative waypoint loop through three platform
entries: `sim`, `airsim`, and `tello`. It does not contain separate navigation,
search, follow, reasoning, or long-horizon task modules. Those experiment
categories are exercised by giving the same loop different natural-language
instructions. GameUAV does not add task-specific policies on top of that loop.

The author Tello implementation supports `adaptive_mode` and `obstacle_mode`.
The GameUAV worker reads `operational_mode` from
`adapter/config_tello.yaml`, matching the author's configuration behavior.
`SPF_OPERATIONAL_MODE` may override it for one worker process. Restart the
worker after changing the mode; `/health` and inference metadata report the
effective mode.

The PX4 adapter preserves the author's relative `ActionPoint` convention and
faces the selected direction while moving. It replaces the Tello SDK actuation
layer with a bounded EGO target path shared with SMPF. Task
timeouts, operator completion, and endpoint occupancy projection are local
safety/integration behavior, not claims about the author implementation.

Bringup:

```bash
# Terminal 1: SPF worker boundary.
cd /home/uav/Desktop/uav_project/gameuav
cd strategy/see_point_fly/upstream
HTTPS_PROXY=http://127.0.0.1:7890 HTTP_PROXY=http://127.0.0.1:7890 uv sync

cd /home/uav/Desktop/uav_project/gameuav
OPENAI_API_KEY=... OPENAI_BASE_URL=https://openrouter.ai/api/v1 \
  HTTPS_PROXY=http://127.0.0.1:7890 HTTP_PROXY=http://127.0.0.1:7890 \
  uv run --project strategy/see_point_fly/upstream \
  python strategy/see_point_fly/worker/spf_worker.py --host 127.0.0.1 --port 9310

# Terminal 2: ROS bridge.
source /opt/ros/noetic/setup.bash
source devel/setup.bash
ROS_HOME=/tmp/gameuav_ros_home ROS_LOG_DIR=/tmp/gameuav_ros_logs \
  roslaunch launch/bringup_see_point_fly.launch
```

Command topics:

```bash
rostopic pub -1 /spf/enable std_msgs/Bool "data: true"
rostopic pub -1 /spf/user_command std_msgs/String "data: 'fly toward the red target'"
rostopic echo /spf/status
```

Continuous SPF direct-control task loop:

The task executor is launched with the bridge but starts in `DISABLED`. It never
arms, takes off, or lands. Abort, disable, MAVROS disconnect, and PX4 disarm
replace the active SPF target with a current-position hold through
`/control/stop`. The bridge and control interface perform the direct command
conversion. A task start is
rejected unless MAVROS is connected, PX4 is already armed, odometry is fresh,
and the vehicle is hovering inside the configured altitude and speed bounds.
Use the ground-station preview inference for disarmed tabletop checks; preview
does not publish `/control/ego_position`.

```bash
# Enable both the direct-control bridge and task executor for this flight session.
rostopic pub -1 /spf/enable std_msgs/Bool "data: true"

# Start one persistent semantic task. After arrival release, the executor waits
# for its inter-cycle delay and then requests another SPF action.
rostopic pub -1 /spf/task/start std_msgs/String \
  "data: 'fly to the chair'"

rostopic echo /spf/task/status

# Stop requesting new goals and replace the active target with a position hold.
rostopic pub -1 /spf/task/control std_msgs/String "data: 'abort'"

# Mark a task successful after the experiment operator verifies completion.
rostopic pub -1 /spf/task/control std_msgs/String "data: 'complete'"
```

Task states are `DISABLED`, `IDLE`, `WAITING_GOAL`, `WAITING_ARRIVAL`,
`WAITING_NEXT`, `SUCCESS`, `TIMEOUT`, `ABORTED`, and `ERROR`. The first migrated
version intentionally keeps the paper's operator-confirmed success criterion;
automatic semantic completion detection is not inferred from position-target arrival.

A one-shot `/spf/user_command` remains in `AUTO_HOVER` after EGO completes its
local trajectory because it does not request another goal. A
continuous `/spf/task/start` loop later requests the next inference; its new goal
returns PX4Ctrl to `CMD_CTRL`. The author implementation emits relative action
points, not a task-level `final` or `done` result. This arrival-release behavior
belongs to the GameUAV/PX4Ctrl adapter and is not an added SPF model capability.
Every terminal continuous-task result closes `/spf/enable`, which invalidates
the active point and late worker responses. Re-enable SPF before starting the
next task.

GCS / agent module startup:

```bash
python3 tools/agentctl.py start see_point_fly_worker --auth-token uavuavuavuav
python3 tools/agentctl.py start see_point_fly --auth-token uavuavuavuav
```

`see_point_fly_worker` inherits API credentials from the `gameuav-agent`
environment. For OpenRouter/OpenAI-compatible mode, set:

```bash
OPENAI_API_KEY=...
OPENAI_BASE_URL=https://oauth-us.tokenshub.site
SPF_OPENAI_WIRE_API=responses
```

The bridge will reject commands until it has fresh image and odometry messages.
It will also reject worker errors, too-close targets, stale input, and
rate-limited commands.

SPF action convention:

- `dx`: right, meters
- `dy`: forward, meters
- `dz`: up, meters
- `yaw_only`: target is too close; the bridge holds position and commands only yaw
