# Prototype model and architecture

The C++ core is independent of ROS. `FrameSource` separates storage from
computation: `DirectorySource` and `RosbagSource` supply a grayscale image and
its integer nanosecond timestamp. A three-stage oneTBB pipeline performs ordered
reading, parallel detection, and ordered collection with bounded images in
flight. Each worker owns its AprilTag family and detector to avoid sharing
mutable lookup tables. The historical Kalibr backend is the default; AprilTag 3
is selected explicitly with `--detector apriltag3`. The historical detector is
bundled without ROS 1, with per-thread counters.
Both backends operate on distorted grayscale input and share Kalibr2 filters and
corner refinement. Pixels are released after detection; only corners and
IMU samples enter the solver. The cache contains no pixels.

## Memory and parallelism

By default, the maximum number of images in flight is `threads`. If
`--image-memory-mib` is set, extraction uses:

```text
min(threads, image_budget / (16 × width × height + 2 MiB))
```

This optional conservative reservation covers pixel/encoded buffers, not total
RSS. Bag indexes and chunks, codec and detector workspaces, observations, IMU
samples, and factorization require additional memory. If a configured budget
cannot fit one image, extraction is rejected before loading; PNG/JPEG dimensions
are checked before decoding.
Input reading/decompression remains sequential, while detection is parallel.
Image buffers are bounded, but batch calibration memory still grows with
recording duration. No GPU backend is currently implemented.

## Conventions and equations

The world frame W is fixed to the AprilGrid. `T_w_i(t)` is the IMU pose in W.
`T_cam_imu = T_c_i` maps IMU coordinates into camera coordinates:
`p_c = R_c_i p_i + t_c_i`. An observation uses the time convention
`t_imu = t_camera + timeshift_cam_imu`. Timestamp origins are subtracted before
conversion to seconds to preserve precision for large timestamps.

Predictions are:

```text
pixel = projection(T_c_i · inverse(T_w_i(t_camera + Δt)) · target_point)
gyro  = vee(R_w_iᵀ · dR_w_i/dt) + gyro_bias
accel = R_w_iᵀ · (d²p_w_i/dt² − gravity) + accel_bias
```

A uniform cubic R3 spline represents translation and a cumulative SO(3) spline
represents rotation. Angular velocity and acceleration use analytic time
derivatives. Ceres AutoDiff computes parameter derivatives over local spline
blocks. Quaternions use a three-dimensional manifold; gravity has a fixed norm
of 9.80665 m/s² through a spherical manifold.

Each camera factor reserves a fixed superset of knots spanning the allowed time
offset. Its active support can cross spline intervals during optimization
without referencing parameters outside the factor.

Initialization estimates PnP poses from corners, searches over time offset,
aligns angular velocities using SVD, and estimates gravity direction and
accelerometer bias by least squares. Pose differences are used only for
initialization; optimized factors use spline derivatives.

IMU residual weights are `sqrt(1/update_rate)/noise_density`, assuming nominal
sampling at `update_rate`. Subsampling alone is not anti-alias filtering. Units
and the continuous noise model follow the
[Kalibr documentation](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model).
The current solver uses constant gyroscope and accelerometer biases. YAML
random-walk parameters are not yet used.

## Differences from Kalibr

Kalibr uses an order-6 pose spline by default and time-varying biases. The cubic
SO(3)+R3 variant is motivated by
[Sommer et al., CVPR 2020](https://arxiv.org/abs/1911.08860) and approaches used in
[Basalt](https://gitlab.com/VladyslavUsenko/basalt/-/blob/master/doc/Calibration.md).
Its smaller local support does not guarantee higher accuracy. Extrinsics, time
offset, trajectory, gravity, and biases are optimized jointly; camera intrinsics
remain fixed.

Ceres evaluates residuals and Jacobians across multiple threads. The linear
solver is `SPARSE_NORMAL_CHOLESKY`, using available sparse libraries (SuiteSparse
in Docker). This does not guarantee parallel execution of the entire
factorization. A Huber loss is applied per frame; per-corner robustification and
individual residual analysis remain future work.

## Checks and limitations

Nonfinite inputs, unordered timestamps, unsupported formats, incompatible image
sizes, and duplicate target IDs are rejected. Calibration requires at least 12
overlapping poses, 100 IMU samples, sufficient duration, and angular excitation
on at least two axes. The excitation check is a heuristic, not a full
observability analysis of translation or timing.

A final YAML is exported only when Ceres converges, results are finite,
reprojection RMSE is below the threshold, and time offset is away from its
search bound. These checks do not replace normalized IMU residuals, uncertainty
estimates, or validation on another recording. Covariance, IMU intrinsics,
rolling shutter, multicamera calibration, clock drift, and standalone camera
calibration are not implemented.

## Evaluation protocol

Record tool commits, hardware, library versions, dataset hashes, selected
timestamps, and configurations. Measure reading/decoding, detection,
initialization, Jacobian evaluation, and linear solving separately. Report total
time and peak memory with and without an observation cache. Compare repeated
runs with 1/2/4/8 workers and native/high-resolution images.

Track retained corner counts and calibration quality alongside speed. The
[detector benchmark](benchmark-kalibr-detector.md) compares both Kalibr2
backends and the original Kalibr run on the same recording. The historical
backend restores full detection coverage. Independent validation is still
needed before claiming equivalent absolute accuracy.
