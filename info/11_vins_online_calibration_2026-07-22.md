# VINS Online Calibration - 2026-07-22 16:06

## Scope

This sensor-only run supersedes the earlier calibrations from the same day. The
vehicle was disarmed and its propellers were removed. Only ROSCore, MAVROS,
RealSense, and the isolated VINS online-calibration node were active.

Live inputs were approximately `30 Hz` for both rectified infrared streams and
`150-205 Hz` for `/mavros/imu/data`. The motion sequence included three-axis
translation, independent roll/pitch/yaw excitation, combined translation and
rotation, and a final three-dimensional figure-eight verification pass.

## Fixed Result

```text
td = +0.00054337072533990364 s
```

```yaml
body_T_cam0:
  [-0.003602924896, -0.030703411698,  0.999522045501,  0.071946941852]
  [-0.999847221687, -0.016985598197, -0.004125862236,  0.013810903501]
  [ 0.017104157901, -0.999384205380, -0.030637523048,  0.094022047498]
  [ 0, 0, 0, 1]

body_T_cam1:
  [-0.003333703689, -0.031187964505,  0.999507977602,  0.072087817428]
  [-0.999845683368, -0.017135817214, -0.003869524395, -0.036177162873]
  [ 0.017248068597, -0.999366636745, -0.031126025946,  0.094920415955]
  [ 0, 0, 0, 1]
```

The resulting stereo baseline is `49.996337 mm`. During the final 20-second
static window, translation changes were sub-micrometer and `td` remained near
`0.543 ms`.

## Rejected Result

This result was deployed temporarily without a separate fixed-parameter dynamic
validation. During the 2026-07-23 flight, VINS altitude diverged from `0.33 m`
to `4.81 m` in approximately 10 seconds and continued diverging after disarm.
The result is rejected and must not be used for flight.

Production, fixed-candidate, and future online-calibration initial values were
restored to the validated 2026-07-21 result documented in
`09_vins_online_calibration_2026-07-21.md`.
