# GameUAV Codex Context - Ground Station Side

This workspace is the ground-station deployment of GameUAV.

Current machine:

- Host role: ground station
- Workspace: `/home/abin/Desktop/gameuav_station`
- Ground station IP: `20.0.0.172`
- Target UAV: `uav0`
- UAV IP: `20.0.0.188`
- `20.0.0.187` is the separate `uav1` endpoint.

Important architecture rule:

- This machine runs the GCS web backend and frontend.
- This machine should not run ROS control nodes directly.
- ROS, PX4Ctrl, VINS, EGO, SPF bridge, camera ROS publishers, and control execution stay on the UAV.

Ground-station responsibilities:

- `gcs/backend/`: FastAPI backend for the web UI.
- `gcs/frontend/`: browser UI.
- `comm/`: shared protocol and agent TCP client.
- `gateway/`: shared camera client definitions used by the GCS backend.
- `.env`: runtime configuration pointing the GCS to the UAV.

Current service:

- `gameuav-station-gcs.service`: enabled, active

Important endpoints:

- Local GCS web: `http://20.0.0.172:8000/`
- UAV agent: `20.0.0.188:8765`
- UAV net-to-ROS gateway: `20.0.0.188:9100`
- UAV camera stream gateway: `http://20.0.0.188:9200`

Expected `.env` direction:

```bash
GAMEUAV_AGENT_HOST=20.0.0.188
GAMEUAV_AGENT_PORT=8765
GAMEUAV_GATEWAY_TCP_HOST=20.0.0.188
GAMEUAV_GATEWAY_TCP_PORT=9100
GAMEUAV_CAMERA_BASE_URL=http://20.0.0.188:9200
GAMEUAV_TARGET_UAV_ID=uav0
GAMEUAV_AGENT_TOKEN=uavuavuavuav
```

Common ground-station commands:

```bash
systemctl status gameuav-station-gcs.service
sudo systemctl restart gameuav-station-gcs.service
curl http://127.0.0.1:8000/api/agent/health
curl http://127.0.0.1:8000/api/gateway/health
curl http://127.0.0.1:8000/api/cameras/settings
```

Development guidance:

- Web UI edits go under `gcs/frontend/`.
- Web backend edits go under `gcs/backend/`.
- Camera shared code is imported from `gateway/camera_stream_gateway/*`.
- Do not assume ROS exists locally on the ground station.
- If UAV-side code changes are needed, edit the UAV repository at `/home/uav/Desktop/uav_project/gameuav` and then sync the required shared files back here.
