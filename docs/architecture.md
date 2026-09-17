# Modèle et architecture du prototype

Le noyau C++ ne dépend pas de ROS. `FrameSource` sépare le stockage du calcul :
`DirectorySource` ou `RosbagSource` fournissent une image grise et son timestamp
entier en nanosecondes. Un pipeline oneTBB en trois étapes (lecture ordonnée,
détection parallèle, collecte ordonnée) limite les images en vol. Chaque worker
possède sa propre famille/détecteur AprilTag pour éviter toute table mutable partagée.
Les images disparaissent après détection ; seuls les coins et les mesures IMU
entrent dans le solveur. Le cache ne contient aucun pixel.

## Conventions et équations

Le monde W est fixé à la cible AprilGrid. `T_w_i(t)` est la pose de l'IMU dans W.
`T_cam_imu = T_c_i` transforme les coordonnées IMU en coordonnées caméra :
`p_c = R_c_i p_i + t_c_i`. Le temps associé à une observation est
`t_imu = t_camera + timeshift_cam_imu`. Les origines temporelles sont soustraites
avant passage en secondes pour préserver la précision des grands timestamps.

Les prédictions sont :

```text
pixel = projection(T_c_i · inverse(T_w_i(t_camera + Δt)) · point_cible)
gyro  = vee(R_w_iᵀ · dR_w_i/dt) + biais_gyro
accel = R_w_iᵀ · (d²p_w_i/dt² − gravité) + biais_accel
```

Une spline cubique uniforme en R3 représente la translation, une spline cumulative
sur SO(3) représente la rotation. Vitesse angulaire et accélération sont dérivées
analytiquement. Ceres calcule les dérivées par rapport aux paramètres avec AutoDiff,
sur les blocs locaux de la spline. Les quaternions utilisent une variété de dimension
3 ; la gravité conserve une norme de 9,80665 m/s² via une variété sphérique.

Le facteur caméra réserve un surensemble fixe de nœuds couvrant toute la plage du
décalage temporel. Son support actif peut ainsi franchir un intervalle de spline
pendant l'optimisation sans référencer des paramètres hors du facteur.

L'initialisation calcule des poses PnP depuis les coins, balaie le décalage temporel
et aligne les vitesses angulaires par SVD, puis estime la direction de gravité et
le biais accéléromètre par moindres carrés. Les différences de poses servent
uniquement à l'initialisation ; les facteurs optimisés utilisent les dérivées de spline.

Les densités de bruit IMU sont pondérées par `sqrt(1/update_rate)/noise_density`.
Cela suppose un échantillonnage nominal correspondant à `update_rate`. Une simple
sous-sélection des mesures ne constitue pas un filtrage anti-repliement. Les unités
et le modèle continu suivent la [documentation Kalibr](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model).
Le solveur actuel utilise un biais constant par capteur ; les marches aléatoires
du YAML ne sont pas encore utilisées. Aucune prétention à un modèle IMU complet.

## Ce qui diffère de Kalibr

Kalibr emploie par défaut une spline de pose d'ordre 6 et des biais temporels.
La variante cubique SO(3)+R3 est motivée par les travaux de
[Sommer et al., CVPR 2020](https://arxiv.org/abs/1911.08860) et les approches de
[Basalt](https://gitlab.com/VladyslavUsenko/basalt/-/blob/master/doc/Calibration.md).
Ce choix réduit le support local mais n'assure pas une meilleure précision.
Les extrinsèques, le décalage temporel, la trajectoire, la gravité et les biais
sont optimisés conjointement ; les intrinsèques caméra restent fixes.

Ceres évalue les résidus/Jacobiennes sur plusieurs threads. Le solveur linéaire
est `SPARSE_NORMAL_CHOLESKY`, avec les bibliothèques creuses disponibles, SuiteSparse
dans Docker. Ce réglage ne garantit pas que toute la factorisation soit parallèle.
Une perte de Huber est appliquée par frame ; une robustification par coin et une
analyse des résidus individuels sont des étapes futures.

## Contrôles et limites

Les entrées non finies, timestamps désordonnés, formats non supportés, tailles
incompatibles et cibles avec IDs répétés sont refusés. La calibration exige au
moins 12 poses et 100 échantillons IMU en recouvrement, une durée suffisante et
une excitation angulaire sur au moins deux axes. Ce dernier contrôle est une
heuristique, pas une analyse complète d'observabilité de la translation/du temps.

Un YAML final n'est exporté que si Ceres converge, les résultats sont finis,
la RMSE de reprojection est sous le seuil et le décalage n'est pas à la borne.
Ces contrôles ne remplacent pas des résidus IMU normalisés, des incertitudes et une
validation sur une autre séquence. Ni covariance ni intrinsics IMU, rolling shutter,
multicaméra, dérive d'horloge ou calibration caméra autonome ne sont implémentés.

## Protocole de comparaison à exécuter

Conserver version/commit des outils, machine, CPU/GPU, RAM, versions des bibliothèques,
dataset, empreintes des entrées, mêmes timestamps sélectionnés et configurations.
Mesurer séparément lecture/décodage, détection, initialisation, évaluation des
Jacobiennes et solveur linéaire ; rapporter temps total et pic RSS, avec puis sans cache.
Comparer plusieurs répétitions, 1/2/4/8 workers et résolution native/haute résolution.
Surveiller nombre de coins retenus et qualité de calibration en même temps que la vitesse.
Les gains de temps ne sont acceptables que si les erreurs restent dans les tolérances
définies sur la référence et si les résidus n'indiquent pas de biais systématique.
