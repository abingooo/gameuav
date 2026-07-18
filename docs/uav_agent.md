# uav_agent

`uav_agent` is the local process manager for a UAV. It is designed to run
before ROS starts and accept controlled TCP commands from a ground station.

It is not a shell executor. The GCS can only request actions against modules
listed in `config/modules/uav_agent.yaml`.

## Start Agent

```bash
cd /home/uav/Desktop/uav_project/gameuav
python3 -m agent.uav_agent.server --host 0.0.0.0 --port 8765 --uav-id uav1
```

Default token:

```text
uavuavuavuav
```

Explicit token:

```bash
python3 -m agent.uav_agent.server --host 0.0.0.0 --port 8765 --uav-id uav1 --auth-token uavuavuavuav
```

## Command Examples

List modules:

```bash
python3 tools/agentctl.py list
```

Start ROS master:

```bash
python3 tools/agentctl.py start roscore
```

Start MAVROS:

```bash
python3 tools/agentctl.py start mavros
```

Start VINS with the combined state-estimation bringup:

```bash
python3 tools/agentctl.py start state_estimation --arg camera_serial_no:=832112071797
```

Query status:

```bash
python3 tools/agentctl.py status mavros
```

Query health:

```bash
python3 tools/agentctl.py health
```

Stop a module:

```bash
python3 tools/agentctl.py stop mavros
```

## Protocol

Each TCP packet is one JSON object followed by `\n`.

Envelope fields:

```json
{
  "protocol_version": "gameuav.agent.v1",
  "message_type": "module_command",
  "source_id": "gcs",
  "target_id": "uav1",
  "sequence_id": "uuid-or-counter",
  "timestamp": 1710000000.0,
  "payload": {
    "request_id": "1001",
    "auth_token": "uavuavuavuav",
    "action": "start",
    "module": "vins"
  },
  "checksum": "crc32"
}
```

Allowed actions:

```text
start
stop
restart
status
list
health
```

## systemd Autostart

Install:

```bash
cd /home/uav/Desktop/uav_project/gameuav
sudo deploy/systemd/install_gameuav_agent_service.sh
sudo systemctl restart gameuav-agent.service
```

Check status:

```bash
systemctl status gameuav-agent.service
```

Environment file:

```text
/etc/gameuav/agent.env
```

## Safety Rules

- Module names must exist in `config/modules/uav_agent.yaml`.
- Launch arguments must be whitelisted per module.
- Commands run in independent process groups.
- Stop uses `SIGINT`, then `SIGTERM`, then `SIGKILL`.
- Logs are written under `logs/agent/`.
- TCP commands must include the correct token.
