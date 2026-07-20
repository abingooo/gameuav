# SPF Reproduction Status

Last audited: 2026-07-20

## Author Sources

- Released code: `https://github.com/Hu-chih-yao/see-point-fly.git`
- Released code commit: `5621bcf43e9826d60df014541dd0498e743a92bd`
- Supplementary material: `https://github.com/yuna0x0/spf-suppl.git`
- Supplementary material commit: `1308b59c6986aed9938572075d8dc22363047d30`
- Paper: PMLR v305, *See, Point, Fly: A Learning-Free VLM Framework for
  Universal Unmanned Aerial Navigation*

All 59 files tracked by the released-code commit are present under `upstream/`
and match the author checkout byte for byte. Generated environments and caches
are not part of that comparison.

## What The Author Method Implements

The author implementation contains one persistent RGB-and-language to relative
`ActionPoint` loop. Navigation, obstacle avoidance, long horizon, reasoning,
search, and follow are experiment categories expressed through different
natural-language prompts. The released code does not contain six task-specific
policies, an object tracker, a search state machine, or a long-horizon stage
manager.

The GameUAV worker passes the operator command unchanged to the author's
`TelloActionProjector.get_vlm_points()` method. The primary local baseline uses
the author's `adaptive_mode`. `obstacle_mode` is a separate released mode and
must be reported as a separate variant if evaluated.

## Paper Category Evidence Matrix

| Category | Paper environment | Published tasks | Local route | Evidence still required |
| --- | --- | ---: | --- | --- |
| Navigation | Simulation and real world | 5 + 1 | Same persistent author loop | Controlled-flight repetitions |
| Obstacle avoidance | Simulation and real world | 5 + 2 | Same loop; no EGO map or trajectory planner | Collision-scored controlled flights |
| Long horizon | Simulation and real world | 5 + 2 | Same instruction is repeated after each settled action | Operator-scored multi-stage flights |
| Reasoning | Simulation and real world | 3 + 4 | Same prompt is passed unchanged to the VLM | Operator-scored task completion |
| Search | Simulation only | 5 | Author DRL simulator entry only | Out of local scope (real-world-only comparison) |
| Follow | Real world only | 2 | Same repeated image-action loop; no added tracker | Moving-target repetitions and final-distance checks |

The local experiment scope is explicitly real-world-only. Because the paper
does not evaluate Search in the real world, Search is not a pending local task
and no PX4 Search trial will be introduced. The exact simulator manifest remains
available only for source traceability.

## Exact Experiment Protocol

The exact 11 active real-world prompts are recorded in
`../smpf/experiments/spf_realworld_tasks.json`. They cover Navigation, Obstacle
Avoidance, Long Horizon, Reasoning, and Follow. The exact 23 simulation prompts
remain in `../smpf/experiments/spf_simulation_tasks.json` as an inactive source
reference. Both manifests preserve the Table 1 wording without adding a prompt.

Each active task requires five repetitions. A collision is a failure. At
completion, the requested task must be satisfied or the target must be clearly
visible in the final egocentric view and lie within `1 m`. Local position-goal
arrival is not semantic success; the operator must apply the paper criterion.
The author SPF output has no task-level `final` or `done` signal.

## Local Platform Boundary

The author Tello implementation converts the relative action into sequential
timed RC yaw, pitch, and throttle commands. GameUAV replaces only this actuation
layer:

```text
RGB1 + exact prompt
  -> author TelloActionProjector
  -> relative ActionPoint (right, forward, up)
  -> world-frame bounded position target
  -> /control/spf_position
  -> /control/position_cmd
  -> px4ctrl
```

EGO is bypassed. Endpoint projection against the EGO occupancy cloud is disabled
in the launch files and in the bridge's direct-construction default. Current
platform bounds are `1.5 m` horizontal step, `0.3 m` vertical step, and
`0.4-1.5 m` target altitude. These PX4 integration bounds and the switch from
timed RC primitives to a position setpoint mean local results are a faithful
author-policy port, not a numerical reproduction of the Tello dynamics.

The GameUAV control adapter defines local-goal arrival as XY error `<=0.25 m`,
Z error `<=0.20 m`, yaw error `<=10 deg`, and three-dimensional speed
`<=0.25 m/s`, all held continuously for `0.5 s`. It then clears cached SPF/EGO
motion commands and stops `/control/position_cmd`; after PX4Ctrl's configured
`0.5 s` command timeout, `CMD_CTRL` returns to `AUTO_HOVER`. A one-shot command
or manual target remains hovering. Only the continuous `/spf/task/start` loop
requests another inference after its inter-cycle delay, and a new target returns
PX4Ctrl to `CMD_CTRL`. This is GameUAV/PX4Ctrl integration behavior, not an
author-policy capability or semantic completion signal. Any terminal continuous
task result closes `/spf/enable`, invalidating the active point and late worker
responses; a subsequent task requires explicit re-enable.

Every SPF goal publication requires the shared `/spf/enable` session gate plus
fresh, connected, armed MAVROS state. The continuous loop subscribes to the same
gate and additionally requires an already armed hover. A disarmed tabletop
inference must use the preview worker path, which cannot publish a ROS position
target. Abort, gate disable, MAVROS disconnect, or PX4 disarm commands a
current-position hold so a later re-arm cannot continue an old SPF target.

For the primary SPF/SMPF comparison, both methods use `gemini-3.5-flash` as the
visual model. This is a controlled local comparison, not the paper's model
configuration: the supplementary material reports Gemini 2.0 Flash, while the
current released repository defaults to Gemini 2.5 Flash when no model override
is provided.

Camera input is not controlled between methods: SPF keeps `/rgb1/image_raw`,
while SMPF keeps RealSense `/camera/color/image_raw`. This deliberate input
difference is a hard experimental limitation and every comparison must report
it; results cannot be attributed to the planning methods alone.

## Current Verification Level

- Author source integrity: verified.
- Exact 11 active real-world prompts and five-category coverage: verified
  offline. The other 23 published prompts are retained as inactive references.
- Command preservation through the ROS bridge and worker boundary: verified by
  tests.
- Primary mode: live worker reports `adaptive_mode`.
- EGO occupancy projection: live ROS parameter is `false`.
- Direct SPF to px4ctrl topic routing: live ROS graph verified.
- Arrival release: configured for the thresholds above and covered by adapter
  tests; the resulting PX4Ctrl state transition still requires real-flight
  verification.
- Armed-state publication gate and disabled tabletop execution: verified live;
  a disarmed task prompt was rejected before worker inference and produced no
  `/control/spf_position` message.
- Common dynamic attitude gate: implemented for both methods. Stale PX4/VINS
  attitude or roll/pitch disagreement above `15 deg` clears cached control and
  requires a new command after recovery. The operator-approved threshold permits
  the observed `12.28 deg` stationary disagreement while retaining a gross-error
  guard.
- Semantic success for the five active real-world categories: not yet verified.
- Real flight: not authorized by these checks; PX4 remained disarmed and the
  continuous SPF task executor remained disabled during the audit.

Passing an offline action or reaching a local position target must not be
reported as completing Navigation, Obstacle Avoidance, Long Horizon, Reasoning,
or Follow.
