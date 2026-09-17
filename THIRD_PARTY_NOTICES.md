# Références et attributions

Kalibr2 est un prototype indépendant, sans affiliation ou approbation ETH Zurich.
Les équations et conventions s'appuient sur les travaux de Kalibr :

- Paul Furgale, Joern Rehder, Roland Siegwart, *Unified Temporal and Spatial
  Calibration for Multi-Sensor Systems*, IROS 2013.
- Paul Furgale, Timothy D. Barfoot, Gabe Sibley, *Continuous-Time Batch Estimation
  Using Temporal Basis Functions*, ICRA 2012.
- Christiane Sommer et al., *Efficient Derivative Computation for Cumulative
  B-Splines on Lie Groups*, CVPR 2020.

Le noyau est réimplémenté ici ; aucun sous-module du moteur aslam de Kalibr n'est
embarqué. Les six mots de référence de tags dans `tests/test_detector.cpp` sont
ceux du générateur de mire Kalibr, dérivés d'AprilTags. Sources :
[Kalibr](https://github.com/ethz-asl/kalibr),
[générateur](https://github.com/ethz-asl/kalibr/blob/master/aslam_offline_calibration/kalibr/python/kalibr_create_target_pdf).

This product includes software developed by the Autonomous Systems Lab and Skybotix AG.

Licence historique Kalibr (à conserver pour tout code repris) :

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

Les dépendances Ceres, Eigen, oneTBB, OpenCV, AprilTag, yaml-cpp, ROS 2 et rosbags
sont installées séparément et conservent leurs licences respectives. Leurs notices
sont fournies par les paquets système et Python dans l'image de développement.
Le champ `Proprietary` du package indique uniquement qu'aucune licence du nouveau
code n'a encore été choisie ; il ne modifie pas les licences de ces dépendances.
