# Kalibr2 — camera–IMU calibration for ROS 2

Kalibr2 estimates the spatial transform and time offset between a camera and an
IMU using an AprilGrid recording. It combines C++20, parallel AprilTag detection,
and memory-aware image processing for offline calibration on ROS 2 datasets.

This is an **experimental, independent project** inspired by
[ETH Zurich's Kalibr](https://github.com/ethz-asl/kalibr). It is not an official
Kalibr release or a replacement for all of its features.

## What changes compared with Kalibr?

| Area | Official Kalibr | Kalibr2 approach |
|---|---|---|
| Software stack | ROS 1 tooling with Python and C++ components | C++20 calibration pipeline and native ROS 2 bag input |
| AprilGrid extraction | Historical Kalibr detector and extraction pipeline | Historical Kalibr detector by default, streamed across oneTBB workers; AprilTag 3 optional |
| High-resolution images | Historical image processing pipeline | Streaming input, bounded images in flight, pixels released after detection |
| Optimization | Kalibr's aslam backend | Ceres with parallel residual/Jacobian evaluation and a sparse linear solver |
| Trajectory and IMU model | Default order-6 pose spline and time-varying biases | Cubic SO(3) + R3 splines and constant IMU biases |
| Repeated runs | Existing Kalibr workflows | Persistent corner/IMU cache to rerun optimization without processing images again |
| Scope | Camera calibration, camera chains, and broader sensor/model support | One camera and one IMU, with fixed camera intrinsics |

The main improvements implemented here are **parallel extraction**, **bounded
image-buffer allocation**, **reusable observations**, and a **ROS 2 Docker
workflow**. The mathematical model also differs from Kalibr; improved accuracy
has not been demonstrated.

## Current support

- One global-shutter camera, one IMU, and a stationary AprilGrid.
- Fixed camera intrinsics: pinhole with radtan or equidistant distortion.
- ROS 2 bags, PNG/JPEG image directories, or a previously generated cache.
- Kalibr-style input YAML and `camchain-imucam.yaml` output.
- Historical Kalibr tags with a two-cell border and standard AprilTag 3 tags.

Only `cam0` is selected from a multicamera configuration. Camera intrinsic
calibration, rolling shutter, IMU intrinsic calibration, variable biases,
uncertainty estimates, and a GUI are not implemented. IMU random-walk parameters
are read but are not yet used. Use short, well-excited calibration sequences.

## Quick start

The tested Docker environment is Ubuntu 24.04 with ROS 2 Jazzy. Building the
image also runs the test suite:

```bash
docker build -t kalibr2:dev -f docker/Dockerfile .
docker run --rm kalibr2:dev
```

Place your ROS 2 bag directory at `data/recording`, along with `camchain.yaml`,
`imu.yaml`, and `aprilgrid.yaml`. Camera intrinsics can come from Kalibr.
Use [`config/`](config/) as format examples; replace example sensor parameters
with your measured values.

```bash
mkdir -p results
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$PWD/data:/data:ro" -v "$PWD/results:/results" \
  kalibr2:dev /opt/kalibr2/bin/kalibr2 calibrate \
  --bag /data/recording \
  --camera /data/camchain.yaml --imu /data/imu.yaml --target /data/aprilgrid.yaml \
  --output /results/run01 --threads 4
```

By default, Kalibr2 keeps up to `--threads` images in flight.
`--image-memory-mib` is optional and can reduce that count on
memory-constrained machines; it is **not total process RAM**. Bag buffers,
observations, IMU samples, detector workspaces, and the solver require
additional memory. `--tag-border 2` selects historical Kalibr targets; use `1`
for standard AprilTag 3 targets. Keep `--decimate 1` for initial validation.
The bundled historical Kalibr detector is the default. It processes the original
distorted images, supports `tag36h11` with either border, and requires
`--decimate 1`. Use `--detector apriltag3` only to select the optional AprilTag 3
backend. Detector work buffers are outside the pixel-buffer budget. See the
[detector comparison](docs/benchmark-kalibr-detector.md).

Measure the printed target: `tagSize` is in meters and `tagSpacing` is a ratio.

Each run requires a new output directory and saves:

- `observations.cache` and `extraction.json`: reusable observations and extraction metrics.
- `camera-input.yaml`, `imu-input.yaml`, `target-input.yaml`: input configurations.
- `camchain-imucam.yaml` and `solver.txt`: calibration and solver diagnostics on success.
- `calibration-report.pdf`, `residuals.csv`, `residual_summary.json`: single-page PDF report and detailed reprojection residual diagnostics.

Use `extract` instead of `calibrate` to save observations only. To rerun the
solver from a cache, with a local installation:

```bash
kalibr2 calibrate --cache results/run01/observations.cache \
  --camera data/camchain.yaml --imu data/imu.yaml --target data/aprilgrid.yaml \
  --output results/run02 --threads 4 --max-time-offset 0.1
```

For Docker, use the same mounts and container paths as in the first example.
See [usage and installation](docs/usage.md) for ROS 1 conversion and local builds.

## Measurements so far

On a synthetic 11 MP sequence, parallel extraction took **12.36 s versus
23.46 s** with one worker, producing byte-identical caches. That historical benchmark used `--image-memory-mib 512`, which limited
processing to two simultaneous images.

A matched TUM VI rerun with the historical detector, using the recording
referenced by DT-VI-Calib, measured:

| Camera–IMU run | Kalibr | Kalibr2 + Kalibr detector |
|---|---:|---:|
| Images detected | 1,038 / 1,038 | 1,038 / 1,038 |
| Wall time | 98.89 s | 34.84 s |
| Extraction time | 17.37 s | 12.48 s |
| CPU time | 130.10 s | 70.91 s |
| Sampled peak anonymous memory | 607.47 MiB | 367.70 MiB |

Kalibr2 used 1,035 frames after applying its IMU-overlap margin. The original
Kalibr run used 1,038. The detector family and input are matched, but filtering,
corner counts, solver models, report generation, and retained frames still
differ. These single runs establish reliable detection on this recording, not
an accuracy guarantee or a pure solver benchmark.

See the [real-data benchmark](docs/benchmark-dtvi.md),
[synthetic validation](docs/validation.md), and
[detector comparison](docs/benchmark-kalibr-detector.md)
for details.

## Development priorities

1. Validate spatial/time estimates on independent recordings and sensor references.
2. Benchmark bounded extraction on long 4K recordings and machines with more cores.
3. Add variable IMU biases, per-corner robustness, and uncertainty diagnostics.
4. Profile bottlenecks before adding GPU support; extend acquisition guidance and camera support.

The [architecture document](docs/architecture.md) describes the equations,
memory strategy, and differences from Kalibr. Attribution and third-party
notices are in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
A license for the new project code has not yet been selected.
