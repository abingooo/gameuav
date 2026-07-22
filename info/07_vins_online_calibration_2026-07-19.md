# VINS Online Calibration - 2026-07-19

> Historical result only. The camera/IMU mechanical layout changed afterward,
> so production was recalibrated on 2026-07-21. See
> `09_vins_online_calibration_2026-07-21.md` for the active result.

## Scope

This calibration was performed with the vehicle disarmed and propellers removed.
Only MAVROS, RealSense, and VINS were started. PX4Ctrl, EGO, SPF, and SMPF stayed
stopped.

Live inputs:

- `/camera/infra1/image_rect_raw`: 29.993 Hz
- `/camera/infra2/image_rect_raw`: 29.993 Hz
- Stereo timestamp difference: 0 ms
- `/mavros/imu/data`: approximately 197 Hz after stream setup
- RealSense serial: `254622075677`

The live infrared `camera_info` reports rectified images with zero distortion,
`fx=fy=386.6515197753906`, `cx=322.40374755859375`, and
`cy=244.408203125`. Production VINS now uses those rectified intrinsics.

## Result

The estimator optimized around the previous Kalibr transform, using combined
translation and roll/pitch/yaw excitation. The accepted fixed time offset is:

```text
td = +0.012404321603 s
```

The accepted fixed transforms are stored in
`ros_nodes/state_estimation/VINS-Fusion/config/fast_drone_250.yaml`. The
resulting stereo relationship has:

- baseline: `49.368615 mm`
- relative rotation: `0.032027 deg`
- difference from live RealSense baseline: `0.588 mm`

## Validation

The online result was copied into a fixed configuration with both online
estimation switches disabled and run without any controller.

- fixed validation duration: approximately 7 minutes
- final 90-second static displacement: `0.0215 m`
- dedicated 60-second static maximum displacement: `0.0196 m`
- VINS/PX4 roll error: median `0.0686 deg`, max `0.1328 deg`
- VINS/PX4 pitch error: median `0.0422 deg`, max `0.0786 deg`

The previous approximately 13-14 degree roll/pitch disagreement was not present
with the calibrated candidate.

## Status

The result is frozen in the production VINS configuration with
`estimate_extrinsic=0` and `estimate_td=0`. The online calibration profile and
fixed candidate profile remain available for repeatability.

A deliberate hand-carried 0.5 m return-to-origin validation was requested but
no qualifying motion was detected during the validation window. Complete that
test with propellers removed before the next powered flight.
