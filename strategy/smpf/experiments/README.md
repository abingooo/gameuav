# SPF/SMPF Comparison Runbook

This directory fixes the author task wording and the local comparison protocol.
It does not authorize arming or flight.

## Experiment Scopes

The primary local comparison uses all 11 author real-world tasks, five
repetitions per method:

```text
11 tasks x 5 repetitions x 2 methods = 110 trials
```

Those tasks cover Navigation, Obstacle Avoidance, Long Horizon, Reasoning, and
Follow. This is the complete active experiment scope.

The formal record identity is `(environment, method, task_id, repetition)`.
For every `task_id x repetition`, the recorder generates one canonical pair ID,
for example `real_world:nav_chair_long:r01`. Exactly one SPF record and one SMPF
record must use that pair ID. The outcome writer locks the JSONL file and
rejects a second record with the same identity before appending it.

The paper evaluates Search only in the DRL Simulator and provides no real-world
Search evaluation. Search is therefore out of scope for this real-world-only
comparison. `spf_simulation_tasks.json` is retained only as an author-source
reference; it is not a pending local experiment and must not be replaced with a
physical-room Search task.

## Fixed Method Profiles

Primary SPF:

- author code commit `5621bcf43e9826d60df014541dd0498e743a92bd`
- `adaptive_mode`
- `gemini-3.5-flash`
- EGO occupancy projection disabled
- bounded direct position target to px4ctrl

Primary SMPF:

- `muavold` `dev` source commit `9c0121ae60722d5e1db7d99380cb2cd734aab48a`
- `gemini-3.5-flash` visual grounding
- `gpt-5.2`, reasoning effort `low`, for planning where applicable
- verified 3-D goal or guidepoints routed through EGO

`gemini-3.5-flash` is the controlled local model choice. It is not the paper's
reported Gemini 2.0 Flash configuration. SPF `obstacle_mode` is a separate
released-code ablation and must not replace `spf_adaptive` inside the primary
aggregate.

## Before Every Trial

1. Select one task by ID from the appropriate manifest. Do not edit its prompt.
2. Assign stable `scene_id` and `start_pose_id` labels, a trial timeout, and one
   randomized method order for the pair. Use those same values for both records.
3. Check that the live method mode, model IDs, API backend/protocol, and image
   topic exactly match `comparison_profile.json`; pass
   `--confirm-fixed-profile` only after this check. The recorder copies the
   fixed profile into the record and also requires the actual RGB width/height.
4. Ensure only the selected method owns `/control/position_cmd` for the trial.
5. Record the fixed camera-input difference: SPF uses `/rgb1/image_raw`, while
   SMPF uses RealSense `/camera/color/image_raw`. This difference is retained by
   design and must be reported as a hard limitation. Do not describe the result
   as a camera-controlled or pure-planner comparison.
6. Verify collision observation, final egocentric target visibility, and final
   target distance can be scored.
7. Keep preview and execution separate. Disarmed SPF checks use the worker/GCS
   preview path, which does not publish `/control/spf_position`.

SPF receives the manifest prompt as one unchanged string. SMPF receives the same
prompt as `instruction`, with this fixed category mapping:

| Category | SMPF mode |
| --- | --- |
| Navigation | `navigate` |
| Obstacle Avoidance | `obstacle` |
| Long Horizon | `long_horizon` |
| Reasoning | `reasoning` |
| Follow | `follow` |

The mapping selects the proposed method branch; it does not alter the initial
task wording.

## Real-Flight Commissioning Order

Commissioning flights are not counted in the 110 scored trials.

1. Perform one unified stack restart so the current launch arguments and agent
   command whitelist are loaded. Keep both method execution gates closed.
2. Verify fresh MAVROS/VINS state, both declared camera topics, EGO readiness,
   model metadata, and the operator abort path before arming. Hold the vehicle
   stationary through the preflight window and require the common control
   status to report `attitude_guard_ok=true`; any VINS/PX4 roll or pitch
   disagreement above `15 deg` is a hard no-go, not a warning to waive.
3. Run the exact Navigation prompt once with SPF and once with SMPF, one method
   enabled at a time. Abort and close the prior method gate before switching.
4. After both Navigation paths complete safely, commission Obstacle Avoidance,
   Reasoning, Long Horizon, and Follow in that order.
5. Begin randomized, scored repetitions only after collision observation,
   final-view scoring, timeout, abort, and outcome recording have all been
   exercised successfully.

The UAV agent exposes `spf_task_enable`, `spf_task_start`, and
`spf_task_control` for SPF. `spf_task_enable` controls the shared `/spf/enable`
gate used by both the direct bridge and continuous task executor. It can open
only while PX4 is already connected and armed. The agent also exposes
`smpf_execution_enable`, `smpf_task_command`, and `smpf_task_control` for SMPF.
SMPF runtime enable succeeds only when its launch-time `execution_enabled`
permission was set for that stack start.

## Success Evidence

Use the author supplementary protocol:

- collision is failure;
- the final target must be visible;
- the requested task must be complete, or the final target distance must satisfy
  the manifest threshold;
- threshold: at most `1 m` in the real world;
- local waypoint arrival alone is not success.

Record a task-completion success:

```bash
python3 strategy/smpf/experiments/record_outcome.py \
  --method spf \
  --variant spf_adaptive \
  --environment real_world \
  --task-id nav_chair_long \
  --repetition 1 \
  --method-order smpf_then_spf \
  --scene-id room-a-layout-01 \
  --start-pose-id floor-mark-a \
  --trial-timeout-sec 300 \
  --image-width 640 \
  --image-height 480 \
  --confirm-fixed-profile \
  --outcome success \
  --task-completed \
  --target-visible
```

Add `--duration-sec`, `--path-length-m`, and `--api-calls` only with measured
values. For a target-distance success, omit `--task-completed` and provide
`--target-visible --final-target-distance-m VALUE`. The recorder rejects unknown
task IDs, non-primary variants, profile fields that are not confirmed, success
records without paper-protocol evidence, and duplicate trial identities. Use
the actual resolution observed for each method; the formal summary rejects a
method whose resolution changes across its scored trials.

Summarize the real-world scope:

```bash
python3 strategy/smpf/experiments/summarize_outcomes.py \
  runtime/spf_smpf_outcomes.jsonl \
  --tasks strategy/smpf/experiments/spf_realworld_tasks.json
```

The summary always reports `expected=110`, the raw `observed` record count,
every `missing` identity, and every duplicate identity. `coverage.complete` is
true only for 110 unique, protocol-valid records with 55 consistent pairs and
no protocol errors. The command exits with status `2` while the formal dataset
is incomplete or invalid, after printing the audit report.

Do not include `spf_simulation_tasks.json` or Search outcomes in this
real-world-only comparison.
