# Kalibr2 — calibration caméra–IMU en C++ / ROS 2

Prototype expérimental exécutable : extraction AprilGrid en flux, cache de coins,
calibration spatiale et temporelle conjointe, export YAML compatible Kalibr.
Ce projet indépendant n'est pas une version officielle ETH de Kalibr.

Le périmètre actuel est **une caméra à obturateur global et une IMU**, intrinsèques
caméra connus, cible immobile. Modèles : `pinhole-radtan` et
`pinhole-equidistant`. Dans un camchain multicaméra, seul `cam0` est utilisé,
avec un message explicite. La calibration des intrinsèques caméra n'est pas incluse.

## Ce qui change par rapport à Kalibr

| Sujet | Choix implémenté | Conséquence et limite |
|---|---|---|
| Exécution | Chaîne C++20, rosbag2 natif | Pas de bindings Python dans le calcul ; Python reste uniquement pour convertir les anciens bags |
| Images haute résolution | Lecture progressive, admission bornée avant lecture, libération après détection | Les pixels ne s'accumulent pas avec la durée du bag ; les coins/IMU et le solveur restent proportionnels à sa durée |
| Détection | AprilTag 3, une instance et une famille par worker oneTBB | Parallélisme entre images ; pas de pools de threads imbriqués |
| Anciennes mires | Adaptation explicite des bordures noires de 2 cellules de Kalibr | Tests sur codes historiques, positions de coins, rotation et stride ; bordure standard 1 sélectionnable |
| Optimisation | Ceres, résidus/Jacobiennes parallèles, système creux | Gains à mesurer sur les mêmes données et la même machine |
| Trajectoire | Spline cubique cumulative SO(3) + translation R3 | Dérivées temporelles analytiques ; variante du modèle historique, pas reproduction à l'identique |
| Relance | Cache persistant coins + IMU | Réglage du solveur sans redécoder ni redétecter les images |
| Utilisation | Deux commandes, contrôle des entrées, résultat Kalibr YAML | CLI hors ligne ; pas encore de GUI ni d'acquisition live |

Le modèle actuel utilise des **biais IMU constants**. Les paramètres de marche
aléatoire sont lus dans le YAML, mais ne sont pas encore exploités par le solveur.
Ce point limite l'emploi sur les longues séquences et les IMU à forte dérive.
Une convergence numérique n'est pas une certification de calibration.
Voir [les équations et écarts explicites](docs/architecture.md).

## Démarrer avec Docker

La configuration testée est Ubuntu 24.04 / ROS 2 Jazzy (LTS disponible sur la
machine de développement). Il ne s'agit pas de la dernière distribution ROS 2.
Le paramètre `ROS_DISTRO` du Dockerfile permet de préparer d'autres distributions,
mais leur compatibilité doit être vérifiée avant de les annoncer comme supportées.

```bash
docker build -t kalibr2:dev -f docker/Dockerfile .
docker run --rm kalibr2:dev
```

La construction compile et exécute les tests. Pour calibrer un bag ROS 2, placer
le bag et les trois YAML dans `data/` et créer le répertoire parent `results/` :

```bash
mkdir -p results
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$PWD/data:/data:ro" -v "$PWD/results:/results" \
  kalibr2:dev /opt/kalibr2/bin/kalibr2 calibrate \
  --bag /data/recording \
  --camera /data/camchain.yaml --imu /data/imu.yaml --target /data/aprilgrid.yaml \
  --output /results/run01 --threads 8 --image-memory-mib 512
```

Les valeurs sous `config/*.example.yaml` sont des **exemples**, à remplacer par
les paramètres mesurés de votre caméra/IMU. Mesurer aussi la taille imprimée des
tags : `tagSize` est en mètres, `tagSpacing` est un ratio de cette taille.
`--tag-border 2` convient aux mires historiques Kalibr ; `1` aux tags AprilTag3
standards. Conserver `--decimate 1` pour la première validation.

Résultats dans un nouveau dossier (aucun écrasement d'une calibration existante) :

- `observations.cache` : coins en précision complète et mesures IMU ;
- `extraction.json` : nombre de frames, détections, limite de concurrence et durée ;
- `camera-input.yaml`, `imu-input.yaml`, `target-input.yaml` : paramètres utilisés ;
- `camchain-imucam.yaml` : transformation et décalage temporel, si convergence ;
- `solver.txt` : état du solveur, biais, gravité et erreur de reprojection.

Une erreur après extraction conserve le cache. Pour reprendre sans retraiter les images :

```bash
kalibr2 calibrate --cache results/run01/observations.cache \
  --camera data/camchain.yaml --imu data/imu.yaml --target data/aprilgrid.yaml \
  --output results/run02 --threads 8 --max-time-offset 0.1
```

La commande ci-dessus suppose une installation locale (voir plus bas). Dans
Docker, utiliser les chemins `/results`, `/data` et `/opt/kalibr2/bin/kalibr2`
comme dans l'exemple précédent. `extract` exécute uniquement la détection/cache.

## Utiliser les datasets Kalibr

Récupérer le bag **IMU-CAM** et ses fichiers de configuration depuis la
[page officielle](https://github.com/ethz-asl/kalibr/wiki/downloads).
Les bags ROS 1 historiques nécessitent une conversion préalable ; le convertisseur
est inclus dans Docker et n'exige pas d'installation ROS 1 :

```bash
docker run --rm --user "$(id -u):$(id -g)" -v "$PWD/data:/data" \
  kalibr2:dev rosbags-convert --src /data/imu_april.bag \
  --dst /data/imu_april_ros2 --dst-storage mcap --dst-typestore ros2_jazzy
```

Utiliser ensuite `--bag /data/imu_april_ros2` avec les YAML de ce dataset.
Les timestamps proviennent des `header.stamp`, jamais des dates de réception du bag.
La voie répertoire est aussi disponible : `--dataset /data/sequence`, contenant
`cam0/<timestamp_ns>.png` (ou `.jpg`) et `imu0.csv` avec les colonnes
`timestamp_ns,wx,wy,wz,ax,ay,az`, radians/s et m/s². Les lignes de commentaire
commencent par `#`. Les horodatages doivent être strictement croissants par capteur.

Une première comparaison reproductible sur le dataset public DT-VI 512_16 est
documentée dans [le benchmark Kalibr 1 / Kalibr2](docs/benchmark-dtvi.md).
Elle mesure séparément temps mur, temps CPU, mémoire de calcul et cache fichier.
Ce résultat sur un seul dataset ne constitue pas encore une validation générale.

## Mémoire et parallélisation

Le nombre maximal de frames en vol est :

```text
min(threads, budget_images / (16 × largeur × hauteur + 2 Mio))
```

Cette réservation conservatrice couvre les buffers pixel/encodage, pas le RSS total :
les index/chunks du lecteur de bag, les workspaces des codecs et d'AprilTag,
les observations, les mesures IMU et la factorisation s'y ajoutent.
Une image qui ne tient pas dans le budget est refusée avant sa lecture.
Les dimensions PNG/JPEG sont vérifiées avant décodage.
La lecture/décompression d'entrée reste séquentielle ; la détection est parallèle.
La mémoire des images reste bornée, celle de la calibration batch ne l'est pas
indépendamment de la durée. Utiliser des séquences de calibration courtes et riches
en mouvements. Aucun gain GPU ni facteur d'accélération n'est promis sans benchmark.

## Compilation locale et tests

Dépendances Ubuntu : `build-essential cmake ninja-build pkg-config libeigen3-dev
libceres-dev libsuitesparse-dev libtbb-dev libopencv-dev libapriltag-dev libyaml-cpp-dev`,
et pour ROS 2 : `ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap ros-jazzy-sensor-msgs`.

```bash
source /opt/ros/jazzy/setup.bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DKALIBR2_WITH_ROS2=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
cmake --install build --prefix "$PWD/install"
./install/bin/kalibr2 --help
```

`colcon build` est également prévu par le package ament. Le cœur est compilable
sans ROS avec `-DKALIBR2_WITH_ROS2=OFF` et conserve les entrées répertoire/cache.
Les assertions des tests restent actives en Release. La simulation du solveur
utilise un mouvement analytique indépendant de sa spline : extrinsèques non
triviales, décalage +37,3 ms, biais et gravité connus, deux modèles de distorsion.
Les erreurs obtenues sur cette simulation peu bruitée ne prédisent pas celles d'un
capteur réel. Le détecteur vérifie aussi les coins de mires anciennes/std, leurs
rotations, les doublons et les buffers non contigus ; rosbag2 teste des bags temporaires.

Le test complet génère également de vraies images, lance la CLI et vérifie le
YAML et la relance depuis le cache. Les [mesures locales détaillées](docs/validation.md)
incluent un essai 11 MP : 23,46 s en séquentiel contre 12,36 s avec deux workers
effectifs, caches identiques. Le [benchmark DT-VI](docs/benchmark-dtvi.md) mesure
également la chaîne caméra–IMU complète face à Kalibr 1.

## Suite du développement

1. Répéter le protocole DT-VI sur plusieurs datasets avec vérité terrain et
   analyser l'écart de décalage temporel et de reprojection.
2. Modèle de biais variables avec marche aléatoire, diagnostics d'observabilité,
   incertitudes, traitement robuste des coins aberrants et rapports graphiques.
3. Profilage du décodage/Jacobiennes/factorisation avant SIMD, CUDA ou cuDSS ;
   conservation d'un backend CPU portable.
4. Assistant d'acquisition ROS 2 et interface guidée, puis multicaméra si souhaité.

La licence du nouveau projet reste à choisir avant publication ; aucune licence
open source n'est attribuée implicitement. Voir [les références et attributions](THIRD_PARTY_NOTICES.md).
