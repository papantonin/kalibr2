# Local synthetic validation — September 17, 2026

These measurements characterize the prototype on **synthetic data**. They are
neither a comparison with Kalibr nor an accuracy measurement on a real sensor.
Hardware: Intel Core i3-1115G4, 2 cores / 4 logical threads. Environment: Ubuntu
24.04 / ROS 2 Jazzy container, Ceres 2.2.0, AprilTag 3.3.0, OpenCV 4.6.0,
Eigen 3.4.0, oneTBB 2021.11, GCC 13.3 Release build.

## High-resolution detection

194 PNG images at 3840 × 2880 (11.06 MP), a 3×2 target, the AprilTag 3 backend,
`tag_border=2`, `decimate=1`, and an explicit `--image-memory-mib 512`
buffer budget. Runs were
performed sequentially, once per configuration, with no concurrent build. The
second run may benefit from filesystem caching; repeat in alternating order.

| Setting / result | 1 requested thread | 4 requested threads |
|---|---:|---:|
| Effective workers and maximum images in flight | 1 | 2 (explicit memory budget) |
| Extraction time | 23.4583 s | 12.3642 s |
| Throughput | 8.27 images/s | 15.69 images/s |
| Peak RSS through the end of extraction | 186,152 KiB ≈ 182 MiB | 314,776 KiB ≈ 307 MiB |
| Frames with enough accepted tags | 120 / 194 | 120 / 194 |

The observed time ratio is **1.90×**, comparing parallel and sequential Kalibr2
extraction. The caches are byte-identical, with SHA-256:
`589fa783f2e2e243e6559c84923fe1491268e9d9557c62c45f0f3c59a70b72be`.
RSS includes libraries and detection workspaces but not Ceres optimization
(`extract` command). All 194 images are not held in RAM simultaneously.

To reproduce inside the development container after building:

```bash
build/test_solver --write-fixture build/benchmark-11mp 3
build/kalibr2 extract --dataset build/benchmark-11mp \
  --camera build/benchmark-11mp/camera.yaml --imu build/benchmark-11mp/imu.yaml \
  --target build/benchmark-11mp/target.yaml \
  --output build/benchmark-11mp-threads-1 --detector apriltag3 \
  --threads 1 --image-memory-mib 512
build/kalibr2 extract --dataset build/benchmark-11mp \
  --camera build/benchmark-11mp/camera.yaml --imu build/benchmark-11mp/imu.yaml \
  --target build/benchmark-11mp/target.yaml \
  --output build/benchmark-11mp-threads-4 --detector apriltag3 \
  --threads 4 --image-memory-mib 512
```

Output directories must be new. Read `cache/extraction.json` for elapsed time and
`peak_rss_kib_at_extraction`: the maximum process RSS up to that point, not a
total-memory cap guaranteed by the optional image budget.

## End-to-end image pipeline

A CTest generates analytic motion, IMU measurements, and 194 rendered PNG
images at 1280 × 960. It invokes the CLI and compares the output YAML against
known parameters. IMU motion, target rendering, and reference projections are
independent of the solver's splines.

Observed result: 106 retained frames, rotation error ≈ 0.000325 rad (0.0186°),
translation error ≈ 1.43 mm, and time-offset error ≈ 0.138 ms. Reprojection RMSE
is ≈ 0.294 pixels; the imposed time offset is +37.3 ms. Reusing the cache
produces the same numerical results.

The direct solver test uses slightly noisy analytic corners and covers both
radtan and equidistant models, gravity, and constant biases. It reaches about
0.0051 pixels RMSE. Its lower errors reflect the absence of rasterization and
detection and are not guaranteed real-world accuracy.

## Coverage and remaining work

Six tests cover detection, the solver, memory admission/cache, rosbag2, CLI help,
and the end-to-end workflow. The ROS test exercises SQLite and MCAP, image
encodings, stride, 16-bit endianness, sensor timestamps, and invalid messages.
The tests are not a comprehensive memory-leak or observability assessment.

A separate [real-data comparison](benchmark-dtvi.md) records initial Kalibr
and Kalibr2 results. Remaining work includes common-observation evaluation,
repeatability across recordings, realistic noise/drift, uncertainty estimates,
long-duration recordings, and detailed solver memory profiling.
