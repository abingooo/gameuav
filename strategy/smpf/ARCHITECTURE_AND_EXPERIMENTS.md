# SMPF Architecture and SPF Comparison Protocol

This document records the evidence-backed design boundary for SMPF
(See-Model-Plan-Fly). It separates the imported `muavold` prototype from the
GameUAV implementation that will be used in experiments.

## 1. Upstream Findings

The imported source is the `dev` branch at commit
`9c0121ae60722d5e1db7d99380cb2cd734aab48a`.

### `langguide`: mature prototype path

The implemented flow is:

```text
language command
-> VLM target boxes
-> remote SAM segmentation
-> RGB-D sphere model
-> LLM guidepoints
-> fixed camera-axis permutation
-> /toplan/single_plan_point
-> old EGO waypoint_done feedback
```

It is not directly flight-ready in GameUAV:

- Camera intrinsics are hard-coded and do not match the live color camera.
- The imported node assumes aligned depth without ensuring that the driver
  actually produces it.
- Color detections are indexed into unaligned depth in the available fallback.
- Camera-to-world conversion ignores the VINS attitude quaternion.
- Prompted sphere and segment constraints are not checked in code.
- Search and follow task branches are placeholders; every type runs the same
  one-shot navigation pipeline.
- Planning has no timeout, cancellation, input freshness check, or concurrency
  guard.
- `/toplan/*` belongs to the historical planner contract. GameUAV uses
  `/planning/goal` through the control interface.

### `guide_module`: refactor prototype

The refactor introduces task classes, a SAM client, depth statistics, and full
quaternion rotation, but its contracts are internally inconsistent:

- `control_task` is empty and `search_task` never commands a search motion.
- `navigate_task` calls an undefined method, reads the LLM response at the wrong
  level, passes a path to a single-point clamp, and has waypoint publication
  commented out.
- `follow_task` compares a world position to a camera-relative position and
  continues instead of completing when it reaches the target.
- Geometry outputs FLU `[forward, left, up]`, while the planning prompt declares
  optical `[right, down, forward]` coordinates.
- Geometry returns `sphere_center/radius`, while the prompt requires
  `center/safety_radius`.
- The refactored radius formula underestimates the visible bounding sphere.
- RGB and depth frames are cached independently without synchronization or age
  checks.

The refactor is useful as an ownership boundary, not as executable behavior.

## 2. Authoritative GameUAV Coordinate Chain

SMPF will use these explicit transforms:

```text
aligned color pixel + color CameraInfo
-> point in color optical frame (right, down, forward)
-> inverse RealSense depth_to_color extrinsic
-> depth/infra1 optical frame
-> /vins_fusion/extrinsic (body_T_cam0)
-> body FLU frame (forward, left, up)
-> /vins_fusion/imu_propagate pose
-> world ENU frame
```

The live system reports different color and depth intrinsics. No hard-coded
calibration value is acceptable for SMPF. GameUAV now starts RealSense with
`align_depth=true` and `enable_sync=true`; the bridge synchronizes color,
aligned depth, and color `CameraInfo`, and rejects stale or mismatched frames.
The real-flight VINS configuration fixes the calibrated body-camera transforms.
SMPF also rejects any body-camera translation above `0.75 m` and reports the
calibration error instead of planning with a physically impossible transform.

## 3. Implemented SMPF Pipeline

```text
instruction + synchronized RGB-D + calibrated transforms
-> gemini-3.5-flash semantic target grounding
-> required target SAM mask; conservative VLM-box fallback for optional obstacles
-> robust aligned-depth estimate
-> uncertainty-inflated object sphere in body/world coordinates
-> persistent world-frame semantic scene memory
-> deterministic 3-D approach-corridor obstacle relevance
-> task-specific goal generation:
   - Navigation/Obstacle/Reasoning/Long Horizon: gpt-5.2 guidepoints
   - Follow: one free 3-D point `0.15 m` outside the target safety sphere, no LLM call
-> deterministic schema/path validation for guidepoints or point-only validation for Follow
-> target standoff-band, progress, and visibility checks
-> deterministic target-facing yaw for every accepted waypoint
-> sphere visibility graph + A* repair when both LLM paths are rejected
-> transform validated guidepoints or Follow goal to world
-> /control/ego_position
-> /planning/goal -> EGO -> traj_server -> control_interface -> px4ctrl
-> odometry/velocity/settle feedback
-> re-observe, advance task stage, recover, or stop
```

The LLM proposes intent and route shape for the static/semantic planning modes;
Follow skips it entirely. It is never the authority for numeric safety.
Deterministic validation rejects invalid output, while EGO remains the authority
for collision-aware trajectory generation against live depth data.

### Implemented changes over the imported prototype

1. Calibrated semantic-metric memory: repeated object observations are fused in
   the world frame with conservative volumes, stable object IDs, and explicit
   age/confidence. Short English labels are normalized to their semantic head,
   then associated with a strict center-distance gate; target radii cannot merge
   adjacent same-class objects.
2. Verified guidepoint planning: every point and continuous segment is checked
   independently of the LLM response text.
3. EGO execution feedback: goals advance only after position, altitude, speed,
   and settle-time gates pass; timeouts and cancellation are explicit.
4. Active re-observation: Search, Follow, and Long Horizon modes can issue a
   bounded seven-view yaw sweep, but only after both execution gates are open.
5. Task-stage memory: Long Horizon instructions are decomposed into an ordered,
   schema-checked list. A stage advances only after the prior verified path
   reaches its odometry/velocity/settle gate. The completed target `object_id`
   is retained, and selecting it again is rejected in favor of a new search
   view.
6. Explicit obstacle modeling: scene grounding separates the destination from
   instruction-named obstacles. The destination must independently pass SAM and
   aligned-depth modeling. An optional obstacle whose SAM mask is empty uses its
   complete VLM box and aligned depth as a conservative sphere instead of
   aborting the task or silently dropping the obstacle.
7. Experiment evidence: the bridge writes redacted JSONL events containing
   model call counts, latency, path length, clearance, goal publication, and
   terminal state.
8. Layered execution gates: launch permission, a runtime enable message, armed
   PX4 state, fresh VINS, minimum altitude, and bounded enable-time speed must
   all pass. A disarm event closes execution and requests a stop.
   The shared control interface additionally compares VINS and PX4 roll/pitch;
   a stale attitude or more than `15 deg` disagreement invalidates cached motion
   and blocks both SPF and SMPF until a new command arrives after recovery.
9. Deterministic path repair: after two rejected LLM proposals, SMPF samples
   clearance-inflated points around modeled spheres, builds a segment-verified
   visibility graph, and runs A*. For target tasks it tries an ordered set of
   bounded target-standoff candidates instead of relying on one observer-side
   ray, which avoids false infeasibility near a floor, table, or overlapping
   obstacle. The repaired polyline must pass the same body- and world-frame
   verifier before it can reach EGO.
10. Calibration plausibility gate: rigid rotations and sensor translation
    magnitudes are verified before perception can become planning input. During
    live testing this gate caught an online VINS extrinsic drift from the
    calibrated `0.238 m` baseline to `2.619 m`; fixing the calibrated transform
    restored a `1.84 m` forward estimate for a target measured at `1.605 m`
    depth.
11. Goal-conditioned terminal verification: a collision-free route is not
    sufficient by itself. The final guidepoint must remain between `0.15 m` and
    `1.00 m` outside the uncertainty-inflated target sphere, make measurable
    progress, and retain a segment-verified view of the target. The contract is
    checked in body and world coordinates for both LLM and A* paths.
12. Evidence-based Follow termination: each completed EGO tracking goal ends
    with a fresh target observation. The task succeeds only inside the verified
    target band; exhausting `max_cycles` outside that band is `TIMEOUT`, not
    success.
13. Target-facing execution: yaw is derived geometrically from each world
    waypoint to the current target, not invented by the language model. The yaw
    command is refreshed below px4ctrl's timeout only while the execution gate
    and waypoint state are active. Arrival requires yaw error within `10 deg` in
    addition to position, speed, and settle-time checks.
14. Corridor-relevant obstacle planning: the VLM may ground named and visually
    plausible approach obstacles, but it cannot decide the metric planning set.
    Each remembered obstacle sphere is measured against the continuous segment
    from the UAV to the target standoff point. Only spheres with surface
    clearance at or below `0.25 m` enter LLM/A* planning; all detections remain
    in memory and EGO continues to use the complete live depth map.
15. Direct 3-D Follow goal: SMPF uses the current VLM/SAM/RGB-D target sphere to
    select one free point on the shell `0.15 m` outside that safety sphere and
    submits exactly one world-frame goal to EGO. The nearest observer-side point
    is preferred; another shell point is selected if the nearest point is
    occupied or outside bounds. SMPF does not ask the LLM for a Follow path and
    does not treat the straight line to that point as a trajectory. Body/world
    bounds and goal occupancy are checked before EGO receives the point; EGO
    owns route generation and live depth-map avoidance. A `0.50 m` cap exists
    only as the `smpf_bounded_follow_goal` ablation.
16. Post-grounding freshness: Follow uses the VLM result only as a label, then
    atomically snapshots newer RGB-D, latest odometry, and valid extrinsics.
    It requires frame age `<=1.0 s`, RGB-D/odom skew `<=0.08 s`, and at least one
    full-frame SAM mask. Multiple masks are resolved deterministically by
    selecting the largest reported pixel area, with the first mask winning a
    tie. The new depth/pose produces the sole `0.15 m` EGO goal;
    completion and publication both recheck frame age and fail closed with stop.
    Logs separate grounding and metric stamps, age, skew, and relocalization.
    Static tasks keep their existing timing; Long Horizon refreshes its image
    after initial stage decomposition.

These mechanisms are implemented and unit/dry-run tested. Search remains in the
implementation but is outside the real-world experiment scope. Multi-stage
progression, Follow re-observation, and EGO waypoint execution have not yet been
validated in flight.

Before the direct-goal redesign, a live Follow dry-run using
`gemini-3.5-flash` and `gpt-5.5` detected the
black-shirt target, retained target visibility, and clipped the verified path
to exactly `0.50 m` without publishing a control goal or planning-yaw command.
It is not evidence of flight-ready Follow: the observation was `58.53 s` old
when planning completed (`5.27 s` VLM, `1.47 s` SAM, `51.72 s` LLM). Dynamic
tasks therefore retain a separate latency/observation-freshness gate.

Still before that redesign, after selecting `gpt-5.2` with explicit `low`
reasoning effort, the same live
Follow dry-run again retained target visibility and produced a verified
`0.50 m` prefix with zero control-topic messages. The active-model measurement
was still `57.68 s` end to end (`10.55 s` for two VLM attempts, `1.67 s` SAM,
and `45.45 s` LLM), leaving the observation `57.74 s` old at plan completion.
The model switch therefore does not close the dynamic-task freshness gate.

The active direct-goal redesign then removed Follow path generation entirely.
In final live dry-run `5e3ea4b09ae5`, SMPF selected candidate `1` of `9` because
the nearest shell point was unavailable, and produced one `1.926 m` world-frame
EGO goal. Its final distance was `0.150 m` outside the target safety sphere
(`1.143 m` from the estimated center), and its target sightline was verified.
`guidepoints_m` was empty, the LLM call count and latency were both zero, and
deterministic point selection took `0.002 s`. End-to-end observation age fell
from `57.74 s` to `8.88 s`, now dominated by one VLM attempt (`7.38 s`) and SAM
(`1.41 s`). Both monitored control topics remained silent and the execution
gate remained closed.

### Testable method hypotheses

- `H1`: stable target identity reduces Long Horizon failures caused by revisiting
  the first same-class object. Compare `smpf_full` with
  `smpf_no_target_identity` on the two published `and the next` tasks.
- `H2`: deterministic graph repair reduces planning errors and API retries while
  preserving positive verified clearance. Compare `smpf_full` with
  `smpf_llm_only` on Obstacle Avoidance and Long Horizon tasks; report the
  selected approach-candidate trigger and graph expansion count.
- `H3`: goal-conditioned terminal checks reduce false task completions in which
  a route is collision-free but stops too far away, too close, or behind an
  occluding object. Compare `smpf_full` with `smpf_no_goal_contract` and report
  terminal surface distance and visibility.
- `H4`: target-facing yaw reduces otherwise successful position arrivals whose
  final camera view loses the target. Report final yaw error and final target
  visibility for Navigation, Reasoning, Long Horizon, and Follow.
- `H5`: metric corridor filtering reduces model calls, false infeasibility, and
  path length caused by off-route semantic objects without increasing EGO
  collision events. Compare `smpf_full` with `smpf_no_corridor_filter` and
  report candidate, selected, and filtered obstacle counts.
- `H6`: submitting the complete 3-D standoff goal to EGO reduces VLM-induced
  stop/re-observe latency without increasing target loss or collision rate.
  Compare `smpf_full` with `smpf_bounded_follow_goal` on both Follow tasks;
  report observation age, goal distance, target reacquisition rate, EGO path
  length, collision events, and final target visibility.
- Neither mechanism is credited as a flight-performance gain until the fixed
  five-repetition protocol produces operator-verified outcomes.

## 4. SPF Baseline Boundary

The synchronized author repository is `main` commit
`5621bcf43e9826d60df014541dd0498e743a92bd`. The author exposes one iterative
image-to-relative-action loop; the six categories are prompts evaluated through
that same loop, not separate task policies. The active local comparison uses
only the five categories evaluated by the paper in the real world.

For the primary comparison:

- SPF remains in author `adaptive_mode` and publishes bounded world-frame
  targets through EGO `free_space`; EGO supplies B-spline smoothing and
  dynamics constraints but does not use scene obstacle mapping.
- SPF and SMPF both use `gemini-3.5-flash` for visual decisions in the primary
  comparison. SMPF additionally uses `gpt-5.2` only for structured stage and
  guidepoint planning.
- SPF bridge endpoint projection remains disabled. SMPF uses EGO `mapped`,
  while SPF uses EGO `free_space` without depth/point-cloud occupancy fusion.
- SPF bridge goal publication requires the shared explicit session gate plus
  fresh, connected, armed MAVROS state; its continuous loop uses the same gate
  and additionally requires an already armed hover. Disarmed tabletop checks
  use worker preview only.
- SMPF uses its declared model-plus-EGO pipeline.
- Both methods receive the same task wording, start pose, physical scene, trial
  timeout, and operator success decision.
- Camera input remains intentionally different: SPF uses `/rgb1/image_raw`,
  while SMPF uses RealSense `/camera/color/image_raw`. Every result must report
  this as a hard limitation and must not be interpreted as a camera-controlled
  or pure-planner comparison.
- Model backend, image resolution, retries, API calls, latency, path length,
  collision events, and completion time are logged as experimental variables.

An SPF `obstacle_mode` ablation may be reported separately. Its released code
asks the VLM to adjust a 2D point around obstacles but does not geometrically
check the returned obstacle boxes or path.

## 5. Paper Evaluation Protocol

The SPF supplementary material defines success as completing the requested task
without collision, or ending with the target clearly visible and within 1 meter
in real-world trials. A collision or a final view without the target is failure.
Each task is repeated five times.

The 11 published real-world prompts are:

| Category | Prompt |
| --- | --- |
| Navigation | Fly to the chair (long distance) |
| Obstacle Avoidance | Fly to the person without hitting the cone |
| Obstacle Avoidance | Fly to the person without hitting the door |
| Long Horizon | Fly to the chairs and the next |
| Long Horizon | Fly to the cone and the next |
| Reasoning | It's raining, head to the comfiest chair that looks like it'll keep you dry! |
| Reasoning | Fly to the person who needs help |
| Reasoning | I'm thirsty, find something that can help me. |
| Reasoning | Fly to the person in the dark area |
| Follow | Fly toward the body of the person with red cone |
| Follow | Fly toward the person with green shirt |

The paper evaluates Search only in simulation. It is outside this
real-world-only comparison and is not a pending local task. The 23 simulation
prompts remain fixed in `experiments/spf_simulation_tasks.json` solely for
author-source traceability; no generic physical-room Search task will be added.

## 6. Implementation Gates

The current gate state is:

1. `PASS`: 189 offline geometry, schema, memory, state, gateway, and logging
   tests.
2. `PASS`: live synchronized RGB-D and sensor-extrinsic chain; measured timestamp
   delta 0 ms in the read-only probe. A deliberately observed `2.619 m` VINS
   extrinsic drift was rejected, then the fixed `0.238 m` calibration passed
   after a managed stack restart.
3. `PASS`: live VLM, SAM, LLM, strict schema, and continuous path validation.
4. `PASS`: two consecutive live ROS dry-runs against the tabletop wooden stool.
   The first accepted a verifier-clean LLM path. The second exercised both the
   conservative laptop-box fallback and deterministic
   `target_approach_candidate_2` repair. Both retained target visibility and a
   `0.15-1.00 m` terminal surface standoff. A 120-second listener observed zero
   messages on `/control/ego_position` and `/planning/goal_yaw_deg`; PX4 remained
   disarmed and the execution gate remained closed.
5. `PASS`: active Follow direct-goal dry-run against the black-shirt target.
   SMPF selected one free `0.15 m` target-sphere shell point, submitted no LLM
   request or guidepoint path, declared EGO as trajectory owner, and left both
   monitored control topics silent. The requested goal was `1.926 m` from the
   UAV, `1.143 m` from the target center, exactly `0.150 m` outside its safety
   sphere, and retained a verified target sightline; the execution gate remained
   closed.
6. `FAIL`: long-idle VINS attitude stability. After a managed restart, VINS and
   PX4 roll/pitch initially agreed within `0.46 deg`, but the stationary VINS
   estimate later reached `roll=10.78 deg` while PX4 remained at
   `roll=-1.50 deg` (`12.28 deg` disagreement). The current RealSense image
   retained level room geometry, so this was estimator drift rather than a
   physically tilted camera. The original `5 deg` attitude guard rejected
   takeoff and invalidated live motion when this occurred. The operator-approved
   threshold is now `15 deg`, so this measured disagreement is permitted while
   larger errors still block motion. The guard does not repair VINS.
7. `PENDING`: dynamic-task latency governance: the direct-goal redesign reduced
   observation age from `57.74 s` to `8.88 s`, but the remaining VLM latency
   must be evaluated with a moving target before Follow execution is enabled.
8. `PENDING`: `/control/ego_position` reached `/planning/goal` with px4ctrl
   isolated, but EGO rejected the current tabletop VINS origin as occupied and
   produced no B-spline. Repeat from a controlled hover with a free map start.
9. `PENDING`: tethered/controlled flight trial with one navigation task.
10. `PENDING`: five-category, 11-task real-world experiment after collision,
   timeout, abort, Follow, and multi-stage logging are independently verified in
   motion.

## 7. Reproducible Comparison Artifacts

- `experiments/comparison_profile.json` fixes the local primary methods, models,
  routes, common controls, paper-vs-local distinction, and the decision to keep
  the method-specific camera inputs as a mandatory reported limitation.
- `experiments/spf_realworld_tasks.json` fixes the 11 real-world prompts and
  five repetitions from the supplementary material.
- `experiments/spf_simulation_tasks.json` preserves all 23 DRL prompts as an
  inactive author-source reference; it is not part of the local experiment.
- `experiments/record_outcome.py` appends operator-verified outcomes using a
  fixed failure taxonomy.
- `experiments/summarize_outcomes.py` reports method/category success rates,
  failure reasons, mean duration, path length, and API calls by method variant.
- `experiments/smpf_ablation_variants.json` fixes the primary comparison and
  five single-mechanism SMPF ablations.
- Runtime inference evidence is written to `runtime/smpf_trials.jsonl`, which
  is intentionally excluded from Git.
