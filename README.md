# GameUAV

GameUAV is the UAV-side runtime workspace for onboard perception, state
estimation, planning, control, actuation, strategy execution, and network
gateways.

The ground-station web application is maintained and deployed separately. This
repository must not host or restart a UAV-local GCS backend.

## Repository Layout

- `agent/`: local module manager and health endpoint.
- `comm/`: shared protocol and transport code.
- `gateway/`: UAV-side command and camera gateways.
- `ros_nodes/`: ROS packages for perception, estimation, planning, and control.
- `launch/`: onboard ROS bringup files.
- `strategy/see_point_fly/`: See-Point-Fly baseline and GameUAV adapter.
- `strategy/smpf/`: See-Model-Plan-Fly upstream source import and integration work.
- `config/`: module, network, topic, frame, and mission configuration.
- `deploy/`: systemd and udev deployment templates.
- `tools/`: local administration and diagnostics.
- `tests/`: non-flight unit and integration tests.

## Runtime Boundary

- UAV (`uav0`) agent: `20.0.0.188:8765`
- UAV (`uav0`) command gateway: `20.0.0.188:9100`
- UAV (`uav0`) camera stream: `http://20.0.0.188:9200`
- Ground-station GCS: `http://20.0.0.172:8000/`

Control execution remains on the UAV. Ground-station and Windows clients do not
run ROS directly.

## Development

Generated Catkin workspaces, virtual environments, runtime logs, model
benchmarks, and local credentials are intentionally excluded from Git. See
[`docs/git_workflow.md`](docs/git_workflow.md) for branch, commit, review, and
large-file conventions.

The current machine-specific operating context is documented in
[`AGENTS.md`](AGENTS.md).
