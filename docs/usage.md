# Inputs, installation, and outputs

## Converting ROS 1 bags

Historical [Kalibr datasets](https://github.com/ethz-asl/kalibr/wiki/downloads)
use ROS 1 bags. The Docker image includes `rosbags-convert`; no ROS 1
installation is required for conversion:

```bash
docker run --rm --user "$(id -u):$(id -g)" -v "$PWD/data:/data" \
  kalibr2:dev rosbags-convert --src /data/imu_april.bag \
  --dst /data/imu_april_ros2 --dst-storage mcap --dst-typestore ros2_jazzy
```

Then use `--bag /data/imu_april_ros2` with the corresponding camera, IMU, and
target YAML files. Timestamps come from sensor `header.stamp`, not bag reception
times. Conversion time is separate from calibration time.

## Directory input

Use `--dataset /data/sequence` with:

```text
sequence/
  cam0/<timestamp_ns>.png  (or .jpg)
  imu0.csv
```

IMU CSV columns are `timestamp_ns,wx,wy,wz,ax,ay,az`, with angular velocity in
rad/s and acceleration in m/s². Comment lines start with `#`. Timestamps must be
strictly increasing for each sensor.

## Local build

Ubuntu dependencies: `build-essential cmake ninja-build pkg-config libeigen3-dev
libceres-dev libsuitesparse-dev libtbb-dev libopencv-dev libapriltag-dev libyaml-cpp-dev`.
For ROS 2, also install `ros-jazzy-ros-base ros-jazzy-rosbag2
ros-jazzy-rosbag2-storage-mcap ros-jazzy-sensor-msgs`.

```bash
source /opt/ros/jazzy/setup.bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DKALIBR2_WITH_ROS2=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
cmake --install build --prefix "$PWD/install"
./install/bin/kalibr2 --help
```

The package includes ament integration for `colcon build`. The core also builds
with `-DKALIBR2_WITH_ROS2=OFF`, retaining directory and cache input.
The Dockerfile accepts a `ROS_DISTRO` argument, but only Jazzy has been tested.

## Detector selection

`--detector kalibr` is the default and selects the historical Kalibr detector
bundled with this project, without ROS 1 dependencies. `--detector apriltag3`
selects the optional AprilTag 3 backend. Both operate directly on grayscale
images, without undistortion, and use the same Kalibr2 filtering and subpixel
refinement. This does not reproduce the entire Kalibr observation/pose outlier
pipeline. The historical backend requires `--decimate 1`. Detector workspaces
are outside `--image-memory-mib`.

New V2 caches record the backend. Older V1 caches are still readable and are
identified as AprilTag 3. Detector options cannot be changed with `--cache`;
extract from the original images to change detectors.

## High-resolution recordings

The automatic worker count follows hardware concurrency but is capped at eight.
Pass `--threads N` explicitly to use more. Effective concurrency is also limited
by the image budget described in the [architecture document](architecture.md).
A UHD 3840 × 2160 frame reserves about 129 MiB, so the default 512 MiB budget
admits only three frames. On a 12-thread machine with sufficient RAM, start with
`--threads 6 --image-memory-mib 2048`, measure extraction RSS and elapsed time,
then compare `--threads 12`. Detector workspaces are outside this budget; it is
not a total-memory limit. Use `extract` first and reuse its cache for calibration.

## Output and cache reuse

The output directory must not already exist. Extraction saves full-precision
corners and IMU samples in `observations.cache`, metrics in `extraction.json`,
and copies of the three input YAML files. Successful calibration additionally
exports `camchain-imucam.yaml` and `solver.txt`.

The cache remains available if a later optimization fails. Reuse it with
`calibrate --cache FILE` and a new output directory, as shown in the
[README](../README.md). This avoids image decoding and detection on repeated runs.

## Measuring resources

[`measure_docker_run.sh`](../scripts/measure_docker_run.sh) requires Linux cgroups
v2 and Docker. Usage:

```text
scripts/measure_docker_run.sh CONTAINER_NAME METRICS_JSON LOG_FILE -- docker run ...
```

Use the same name in `docker run --name CONTAINER_NAME`. The script samples
container CPU and memory counters. Report file cache separately from anonymous
memory and preserve the measurement interval, input hashes, image versions,
commands, and retained observation counts alongside the metrics.

Generated bags, build trees, and `results/` are excluded from Git. Keep raw
benchmark artifacts separately when reproducibility or later analysis is required.
