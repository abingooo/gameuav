# GameUAV Codex Context - UAV Side

This workspace is the UAV-side runtime repository.

Current machine:

- Host role: UAV onboard computer
- Workspace: `/home/uav/Desktop/uav_project/gameuav`
- UAV IP used by the ground station: `20.0.0.187`
- Ground station IP: `20.0.0.172`

Important architecture rule:

- This repository no longer contains the GCS web UI/backend as an active UAV-side component.
- Do not recreate or restart a UAV-local `gameuav-gcs.service`.
- The GCS runs on the ground station at `http://20.0.0.172:8000/`.
- The UAV side should keep ROS, control, perception, agent, and gateways here.

UAV-side responsibilities:

- `agent/`: local management agent, starts/stops whitelisted modules.
- `comm/`: shared protocol and transport code.
- `gateway/`: UAV-side network gateways.
- `gateway/camera_stream_gateway/`: ROS image topics to HTTP MJPEG on TCP `9200`.
- `gateway/net_to_ros_gateway/`: TCP command gateway on TCP `9100`.
- `ros_nodes/`: ROS packages for state estimation, planning, control, actuation.
- `launch/`: ROS launch files for onboard stacks.
- `strategy/`: upper-level algorithms such as See-Point-Fly.
- `tools/agentctl.py`: local CLI replacement for the old `gcs/command/agentctl.py`.

Ground-station responsibilities:

- The ground station code is deployed at `/home/abin/Desktop/gameuav_station` on `20.0.0.172`.
- It contains `gcs/`, `comm/`, `gateway/`, and config needed by the web backend.
- The browser should access `http://20.0.0.172:8000/`, not `20.0.0.187:8000`.

Current service split:

- UAV:
  - `gameuav-agent.service`: enabled, active
  - `gameuav-camera-stream-gateway.service`: enabled, active
  - `gameuav-gcs.service`: removed from UAV side
- Ground station:
  - `gameuav-station-gcs.service`: enabled, active

Key network endpoints:

- UAV agent: `20.0.0.187:8765`
- UAV net-to-ROS gateway: `20.0.0.187:9100`
- UAV camera stream gateway: `http://20.0.0.187:9200`
- Ground station web GCS: `http://20.0.0.172:8000`

Common UAV-side commands:

```bash
systemctl status gameuav-agent.service
systemctl status gameuav-camera-stream-gateway.service
python3 tools/agentctl.py health --auth-token uavuavuavuav
python3 tools/agentctl.py start egoctrl --auth-token uavuavuavuav
python3 tools/agentctl.py start rgb1_camera --auth-token uavuavuavuav
```

Development guidance:

- If editing camera streaming code, edit `gateway/camera_stream_gateway/*`.
- Do not import `gcs.backend.*` from UAV-side code.
- If a shared client is needed for agent TCP commands, use `comm.agent_client`.
- If the ground station needs updates, sync the relevant code to `gameuav-station`.
- Keep control execution on the UAV side; Windows or ground-station clients should not run ROS directly.

Known useful SSH alias:

```bash
ssh gameuav-station
```

