# Historical Kalibr detector port — 2026-09-18

The default `--detector kalibr` backend recovers all 1,038 camera images on
TUM VI `dataset-calib-imu1_512_16.bag`. AprilTag 3 remains available explicitly
through `--detector apriltag3`.
This product includes software developed by the Autonomous Systems Lab and Skybotix AG.

## Implementation

The detector library from the local Kalibr checkout at commit
`1f60227442d25e36365ef5f72cd80b9666d73467` is bundled under
[`third_party/ethz_apriltag2`](../third_party/ethz_apriltag2/README.md).
It builds with OpenCV/Eigen, without ROS 1 or Python bindings. Each worker has
its own detector; mutable segment counters and warning flags are thread-local.
Image stride handling supports OpenCV ROIs. Both backends detect directly on
the original distorted grayscale images; neither undistorts before detection.

Kalibr2 retains its common tag-ID/border filters, minimum four tags,
zero corrected bits, subpixel refinement (epsilon 0.01), and corner geometry.
This is a detector port, not the complete original Kalibr observation pipeline:
Kalibr's defaults permit one corrected bit, require seven tags, and use different
subpixel/outlier settings. Consequently corner counts need not match exactly.
The historical backend requires decimation 1. New cache metadata records the
backend; previous V1 AprilTag 3 caches remain readable.

## Same-input comparison

Both runs used the same newly built executable, converted ROS 2 MCAP input,
camera intrinsics, target YAML and IMU YAML, four workers, a 512 MiB pixel-buffer
budget, and otherwise default solver settings. Conversion was outside timing.
The ROS 2 reader supplies the same rounded mono16-to-mono8 conversion to both
backends. The target is 6 × 6, tag36h11, two-cell border, 0.088 m tags, spacing 0.3.
The original Kalibr run selected three extraction processes on this four-thread
host (`cpu_count() - 1`); Kalibr2 used four TBB workers. `OMP_NUM_THREADS=1`
prevented nested detector threads but did not disable Kalibr multiprocessing.

Hardware: Intel Core i3-1115G4, four logical CPUs. One run per backend, sequentially;
these timings do not estimate run-to-run variance.

| Metric | Kalibr2 + AprilTag 3 | Kalibr2 + historical Kalibr |
|---|---:|---:|
| Input camera images | 1,038 | 1,038 |
| Images with accepted detections | 146 | **1,038** |
| Accepted corners before solver | 2,606 | 144,640 |
| Camera frames used by solver | 145 | **1,035** |
| IMU samples read | 10,345 | 10,345 |
| Extraction wall time | 5.408 s | 12.481 s |
| Full container wall time | 17.034 s | 34.839 s |
| Full container CPU time | 35.370 s | 70.914 s |
| Sampled peak anonymous memory | 88.086 MiB | 367.699 MiB |
| Process peak RSS at extraction | 87.730 MiB | 186.543 MiB |
| Cgroup peak including file cache | 713.371 MiB | 529.898 MiB |
| Reprojection RMS | 0.457362 px | 0.116507 px |
| Camera-to-IMU time shift | −0.195626 ms | +0.160267 ms |
| Ceres termination | Convergence, 35 iterations | Convergence, 16 iterations |

Full timing includes Docker startup and a one-second end delay for cgroup
sampling. Cgroup memory includes file-cache charges that differ between runs;
its lower total for the historical backend does **not** mean lower detector
memory. Anonymous memory and extraction RSS show the additional memory cost.
The pixel-buffer budget does not constrain detector workspaces or solver RAM.

The historical backend's three unused camera frames are exactly those outside
the IMU overlap with the default ±50 ms time-shift margin:
`1520527958463478167`, `1520527958513480167`, `1520528010315098167` ns.
They were detected successfully. AprilTag 3 detects only the middle one of
these, explaining its reduction from 146 detected to 145 solver frames.

The prior original Kalibr run detected 1,038 images and 144,494 corners,
with 17.375 s extraction and 98.894 s full wall time. That is a separate earlier
measurement with a different solver and observation pipeline, not an isolated
detector speed comparison. Its +0.163079 ms time shift is close to the port's
+0.160267 ms. Its reported mean reprojection error is not the RMS metric above.
The improved coverage resolves this dataset's detection loss; different fitted
observation sets and absence of ground truth prevent concluding that extrinsic
calibration accuracy is proven better solely from the RMS values.

## Reproduction and artifacts

Build the source with `docker build -t kalibr2:legacy-detector -f docker/Dockerfile .`.
Use the usual calibration command; `--detector kalibr --decimate 1` shows the
defaults explicitly.

Local artifacts are in `results/kalibr2-legacy-port/` (ignored by Git):

- `run-comparison.sh`: exact commands for both measured runs.
- `provenance.json`, `source.patch`, `vendor-source.tar.gz`: input/binary hashes,
  base image ID and source provenance.
- `apriltag3/`, `kalibr/`: caches, extraction metrics, solver reports, input YAMLs
  and calibrated transforms.
- `*-counts.json`, `*-metrics.json`, `*.log`: counts, resources and logs.
- `temporal-exclusions.json`: the three excluded timestamps and IMU bounds.
- `tests.log`, `docker-build.log`: six passing tests including parallel detection,
  corner coordinates, ROI immutability, tilted tags, V1/V2 caches and end-to-end
  calibration/cache reuse with both backends.

The run script expects the converted bag in `ros2/`; conversion command:

```bash
rosbags-convert --src dataset/dataset-calib-imu1_512_16.bag \
  --dst results/kalibr2-legacy-port/ros2 \
  --dst-storage mcap --dst-typestore ros2_jazzy
```

Use new output paths when repeating a run; existing results are never overwritten.

## Direct comparison with the previous original Kalibr run

Using the stored metrics (no new calibration run):

| Metric | Original Kalibr | Kalibr2 + historical detector | Reduction |
|---|---:|---:|---:|
| Full wall time | 98.894 s | 34.839 s | 64.8% (2.84× speedup) |
| Extraction | 17.375 s | 12.481 s | 28.2% (1.39× speedup) |
| CPU time | 130.099 s | 70.914 s | 45.5% |
| Sampled peak anonymous memory | 607.465 MiB | 367.699 MiB | 39.5% |
| Detected images | 1,038 | 1,038 | — |
| Camera images in solver | 1,038 | 1,035 | — |

Subtracting extraction from total gives 81.519 s versus 22.358 s, but this
remainder includes initialization, optimization, output/report generation and
container overhead; it must not be labeled pure solver time. Original Kalibr
also generated a PDF report during these measurements; Kalibr2 now writes a
single-page PDF report. Most of the absolute time saved
is outside extraction. The extraction comparison includes input handling and
filtering, and is not a microbenchmark of identical detector settings.

Both outputs use the same `T_cam_imu` convention. Their translation vectors
differ by 0.4233 mm (Euclidean norm), their rotations by 0.00460 degrees
(relative SO(3) angle), and their time shifts by −2.812 microseconds
(Kalibr2 minus original Kalibr). These are agreement measures, not accuracy
against ground truth. Computed differences are saved locally in
`results/kalibr2-legacy-port/comparison-original-kalibr.json`.
