# GameUAV UAV-Side Architecture Diagrams

These diagrams describe the UAV-side runtime in this repository. The ground-station GCS is shown only as an external system; it is not an active UAV-local component.

Render the `.mmd` files with any Mermaid renderer, or paste them into a Markdown editor that supports Mermaid.

- `00_system_overview.mmd`: top-level UAV-side system split.
- `01_launch_bringup.mmd`: launch/module composition.
- `02_agent_comm_gateway.mmd`: agent, command, protocol, and gateways.
- `03_perception_state_estimation.mmd`: cameras, MAVROS IMU, VINS, and image streaming.
- `04_planning_mission.mmd`: EGO planner and See-Point-Fly mission path.
- `05_control_takeoff.mmd`: px4ctrl VINS-only control and takeoff/land flow.
- `06_spf_smpf_six_tasks.md`: code-level SPF/SMPF implementation, completion semantics,
  and verification boundaries for Navigation, Obstacle Avoidance, Long-Horizon,
  Reasoning, Search, and Follow.
