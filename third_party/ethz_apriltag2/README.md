# Historical Kalibr AprilTag detector

Copied from `ethz-asl/kalibr`, commit
`1f60227442d25e36365ef5f72cd80b9666d73467`, directory
`aslam_offline_calibration/ethz_apriltag2`. Upstream source:
<https://github.com/ethz-asl/kalibr>. Only the detector library and tag36h11 family
are bundled; ROS/catkin, Python, examples and other tag families are omitted.
The original repository license is preserved in `LICENSE`.

Local adaptations:
- Standalone CMake static library using Eigen and OpenCV; no OpenMP enabled.
- Read input pixels using OpenCV row strides (including padded image ROIs).
- Make the segment identifier counter thread-local and reset per image.
- Make the Gaussian warning flag thread-local.
- Allow configuring `TagDetector::thisTagFamily` recovery bits to match the
  Kalibr2 adapter's `max_hamming` option (zero by default for both backends).

The detector algorithm is otherwise retained. Kalibr2 supplies its common
ID/border/minimum-tag filters, subpixel refinement and observation mapping.
This is not the complete Kalibr `GridDetector` pose/outlier-filtering pipeline.
