# References and attribution

Kalibr2 is an independent prototype, without affiliation with or endorsement by
ETH Zurich. Its equations and conventions build on the following research:

- Paul Furgale, Joern Rehder, Roland Siegwart, *Unified Temporal and Spatial
  Calibration for Multi-Sensor Systems*, IROS 2013.
- Paul Furgale, Timothy D. Barfoot, Gabe Sibley, *Continuous-Time Batch Estimation
  Using Temporal Basis Functions*, ICRA 2012.
- Christiane Sommer et al., *Efficient Derivative Computation for Cumulative
  B-Splines on Lie Groups*, CVPR 2020.

The core is reimplemented here; no submodule of Kalibr's aslam engine is bundled.
The six reference tag codewords in `tests/test_detector.cpp` come from Kalibr's
target generator and are derived from AprilTags. Sources:
[Kalibr](https://github.com/ethz-asl/kalibr),
[target generator](https://github.com/ethz-asl/kalibr/blob/master/aslam_offline_calibration/kalibr/python/kalibr_create_target_pdf).

This product includes software developed by the Autonomous Systems Lab and Skybotix AG.

Historical Kalibr license notice (retain for reused code):

Copyright (c) 2014, Paul Furgale, Jérôme Maye and Jörn Rehder,
Autonomous Systems Lab, ETH Zurich, Switzerland

Copyright (c) 2014, Thomas Schneider, Skybotix AG, Switzerland

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Autonomous Systems Lab and Skybotix AG.
4. Neither the name of the Autonomous Systems Lab and Skybotix AG nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTONOMOUS SYSTEMS LAB AND SKYBOTIX AG ''AS IS''
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE AUTONOMOUS SYSTEMS LAB OR SKYBOTIX AG BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.

Ceres, Eigen, oneTBB, OpenCV, AprilTag, yaml-cpp, ROS 2, and rosbags are installed
separately and retain their respective licenses. Their notices are provided by
the system and Python packages in the development image. The package's
`Proprietary` field only indicates that a license for the new code has not yet
been selected; it does not change the licenses of these dependencies.
