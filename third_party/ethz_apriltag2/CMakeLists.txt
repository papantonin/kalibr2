# Historical detector only; no ROS, Python, demo or nested OpenMP workers.
add_library(kalibr2_ethz_apriltag STATIC
  src/Edge.cc
  src/FloatImage.cc
  src/GLine2D.cc
  src/GLineSegment2D.cc
  src/Gaussian.cc
  src/GrayModel.cc
  src/Homography33.cc
  src/MathUtil.cc
  src/Quad.cc
  src/Segment.cc
  src/TagDetection.cc
  src/TagDetector.cc
  src/TagFamily.cc
  src/UnionFindSimple.cc
)
target_include_directories(kalibr2_ethz_apriltag SYSTEM PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(kalibr2_ethz_apriltag PUBLIC Eigen3::Eigen ${OpenCV_LIBS})
set_target_properties(kalibr2_ethz_apriltag PROPERTIES POSITION_INDEPENDENT_CODE ON)
