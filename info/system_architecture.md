# GameUAV UAV-Side System Architecture

The canonical diagram sources live beside this file as Mermaid `.mmd` files.

## Top-Level Split

```mermaid
flowchart LR
  GCS["Ground-station GCS\n20.0.0.172:8000"] --> Agent["UAV agent\n20.0.0.187:8765"]
  GCS --> NetGW["net_to_ros_gateway\n20.0.0.187:9100"]
  GCS --> CamGW["camera_stream_gateway\n20.0.0.187:9200"]
  Agent --> ROS["ROS runtime"]
  NetGW --> ROS
  ROS --> CamGW
  Perception["RealSense / RGB1"] --> Estimation["MAVROS + VINS"]
  Estimation --> Planning["EGO / SPF"]
  Planning --> Control["px4ctrl"]
  Estimation --> Control
  Control --> PX4["PX4 FMU"]
```

## Files

- `00_system_overview.mmd`
- `01_launch_bringup.mmd`
- `02_agent_comm_gateway.mmd`
- `03_perception_state_estimation.mmd`
- `04_planning_mission.mmd`
- `05_control_takeoff.mmd`

