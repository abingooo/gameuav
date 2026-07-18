# See-Point-Fly GameUAV Adapter

This directory keeps third-party SPF code separate from GameUAV integration code.

Layout:

- `upstream/`: clone of `https://github.com/Hu-chih-yao/see-point-fly.git`.
- `worker/`: isolated process boundary for SPF inference.
- `adapter/config.yaml`: bridge safety and topic defaults.

The ROS bridge runs in the GameUAV ROS Python 3.8 environment. The SPF worker is
intended to run in its own `uv` environment because upstream SPF requires a newer
Python than ROS Noetic.

Current integration policy:

- SPF does not publish MAVROS attitude/thrust setpoints directly.
- SPF output is treated as a relative action suggestion.
- The ROS bridge clamps the suggestion and publishes `/control/spf_position`.
- The control interface converts that target to `PositionCommand` on
  `/control/position_cmd`, which px4ctrl consumes directly; EGO is bypassed.
- Endpoint occupancy projection remains enabled, but this direct path does not
  provide EGO trajectory planning or path obstacle avoidance.

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
arms, takes off, lands, or cancels the active position target. The bridge and
control interface perform the direct command conversion. Start it only after the
vehicle is hovering and all normal flight safety checks have passed.

```bash
# Enable the task executor for this flight session.
rostopic pub -1 /spf/task/enable std_msgs/Bool "data: true"

# Start one persistent semantic task. The executor requests another SPF action
# only after the previous direct position target is reached and the vehicle settles.
rostopic pub -1 /spf/task/start std_msgs/String \
  "data: 'fly to the chair'"

rostopic echo /spf/task/status

# Stop requesting new goals. This keeps the current direct position target.
rostopic pub -1 /spf/task/control std_msgs/String "data: 'abort'"

# Mark a task successful after the experiment operator verifies completion.
rostopic pub -1 /spf/task/control std_msgs/String "data: 'complete'"
```

Task states are `DISABLED`, `IDLE`, `WAITING_GOAL`, `WAITING_ARRIVAL`,
`WAITING_NEXT`, `SUCCESS`, `TIMEOUT`, `ABORTED`, and `ERROR`. The first migrated
version intentionally keeps the paper's operator-confirmed success criterion;
automatic semantic completion detection is not inferred from position-target arrival.

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
