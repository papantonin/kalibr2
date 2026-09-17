# Benchmark DT-VI 512_16 face à Kalibr 1

Mesure du 17 septembre 2026 sur la même machine, avec des exécutions
séquentielles et aucun conteneur de calibration concurrent. Les deux
calibrations caméra–IMU utilisent `cam0`, les 1 038 images, les 10 345 mesures
IMU, les mêmes intrinsèques pinhole-equidistant produits par Kalibr 1 et le même
AprilGrid.

## Protocole

[`scripts/measure_docker_run.sh`](../scripts/measure_docker_run.sh) échantillonne
toutes les 100 ms les cgroups v2 du conteneur. `memory.peak` fournit le pic total
exact. `anon`, `file` et `kernel` sont les maxima observés de chaque catégorie.
Le temps CPU vient de `cpu.stat/usage_usec` et le temps mur entoure `docker run`.
Les maxima des catégories ne sont pas nécessairement simultanés et ne doivent
pas être additionnés.

La calibration intrinsèque Kalibr 1 utilise 260 images échantillonnées à 5 Hz
et 20 vues optimisées. Elle est rapportée séparément et n'entre pas dans le
ratio caméra–IMU.

## Performance

| Étape | Temps mur | Temps CPU | Pic cgroup | Pic anonyme | Pic cache fichier | Pic noyau |
|---|---:|---:|---:|---:|---:|---:|
| Kalibr 1, intrinsèques cam0 | 28,389 s | 37,604 s | 1 213,238 Mio | 414,121 Mio | 784,500 Mio | 14,594 Mio |
| Kalibr 1, cam0–IMU | 110,060 s | 142,252 s | 1 382,863 Mio | 612,449 Mio | 757,922 Mio | 16,324 Mio |
| Kalibr2, cam0–IMU | 17,316 s | 35,477 s | 1 020,094 Mio | 87,223 Mio | 929,141 Mio | 3,332 Mio |

Sur l'étape comparable caméra–IMU, Kalibr2 est 6,36 fois plus rapide en temps
mur et consomme 4,01 fois moins de temps CPU. Son pic de mémoire anonyme est
7,02 fois plus faible. Le pic cgroup total ne baisse que de 26,2 %, car la
lecture du MCAP charge près de 929 Mio de cache fichier, récupérable par le
noyau et distinct des buffers de calcul.

## Résultats numériques

| Mesure | Kalibr 1 | Kalibr2 |
|---|---:|---:|
| Reprojection | moyenne 0,08373 px | RMSE 0,45732 px |
| Décalage `t_imu = t_cam + shift` | +0,16258 ms | -0,19245 ms |
| Échantillons IMU d'entrée | 10 345 | 10 345 |
| Images d'entrée cam0 | 1 038 | 1 038 |

Les extrinsèques diffèrent de 1,166 mm en translation et 0,0380 degré en
rotation. Les erreurs de reprojection ne suffisent pas à classer l'exactitude :
les deux détecteurs ne retiennent pas exactement les mêmes observations et les
agrégations diffèrent (moyenne contre RMSE). Une comparaison absolue exige une
vérité terrain ou plusieurs datasets indépendants.

Les bags, caches, PDF et logs bruts ne sont volontairement pas versionnés. Le
script de mesure et ce protocole permettent de reproduire la collecte locale.
