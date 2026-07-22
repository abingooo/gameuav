# VINS Online Calibration - 2026-07-21

## Scope

This calibration supersedes the 2026-07-19 result because the operator changed
the camera/IMU mechanical layout. The earlier camera position was farther from
the IMU, so agreement with the old absolute extrinsic translation was not an
acceptance criterion.

The vehicle remained disarmed and landed. Only MAVROS, RealSense, and the VINS
calibration or fixed-validation node were active. PX4Ctrl, EGO, the control
interface, SPF, and SMPF remained stopped.

Live input rates were approximately:

- `/camera/infra1/image_rect_raw`: `30 Hz`
- `/camera/infra2/image_rect_raw`: `30 Hz`
- `/mavros/imu/data`: `197-200 Hz`

## Accepted Result

The online result was frozen with:

```text
td = +0.0017191652713265113 s
```

```yaml
body_T_cam0:
  [-0.016096545061, -0.015700903812,  0.999747159464,  0.070551123279]
  [-0.999664263908, -0.020051620227, -0.016410118574,  0.004686868613]
  [ 0.020304204058, -0.999675654473, -0.015372870643,  0.093588578726]
  [ 0.0,             0.0,             0.0,             1.0]

body_T_cam1:
  [-0.015918100875, -0.015750094252,  0.999749242858,  0.069390988952]
  [-0.999668878175, -0.019966880043, -0.016231380422, -0.045520639980]
  [ 0.020217518976, -0.999676576815, -0.015427044242,  0.094321017712]
  [ 0.0,             0.0,             0.0,             1.0]
```

Physical and internal consistency checks:

- cam0 translation norm from the IMU frame: `117.296 mm`
- stereo baseline: `50.226 mm`
- stereo relative rotation: `0.0117 deg`
- post-motion 20-second `td` span: `0.000072 ms`
- post-motion translation span: at most `0.000263 mm` per component

## Fixed-Candidate Validation

The result was copied to an isolated configuration with
`estimate_extrinsic=0` and `estimate_td=0` before validation.

Static 60-second result:

- final displacement: `0.008221 m`
- maximum displacement: `0.015595 m`
- roll error: median `0.4834 deg`, maximum `0.6218 deg`
- pitch error: median `0.5066 deg`, maximum `0.7498 deg`

Deliberate return-to-origin result:

- maximum measured excursion: `0.637248 m`
- return error from the recorded start: approximately `0.006 m`
- no estimator restart or failure was detected

## Status

The accepted transforms and time offset are fixed in
`fast_drone_250.yaml` and `fast_drone_250_calibrated_candidate.yaml`.
The online-only profile uses the same result as its next initial guess, while
keeping both online-estimation switches enabled. Production flight keeps
`estimate_extrinsic=0` and `estimate_td=0`.

Raw run artifacts are retained under the ignored runtime directories:

- `runtime/vins_online_calibration/`
- `runtime/vins_candidate_validation/20260721/`
