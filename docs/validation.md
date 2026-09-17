# Validation locale — 17 septembre 2026

Ces chiffres caractérisent ce prototype sur des données **synthétiques**.
Ils ne constituent pas une comparaison avec Kalibr 1 ni une mesure de précision
sur un capteur réel. Machine : Intel Core i3-1115G4, 2 cœurs / 4 threads logiques,
conteneur Ubuntu 24.04 / ROS 2 Jazzy, Ceres 2.2.0, AprilTag 3.3.0, OpenCV 4.6.0,
Eigen 3.4.0, oneTBB 2021.11, compilation GCC 13.3 en Release.

## Détection haute résolution

194 images PNG de 3840 × 2880 (11,06 MP), cible 3×2, `tag_border=2`,
`decimate=1`, budget buffers de 512 Mio. Exécutions séquentielles, une mesure
par configuration, sans autre compilation en parallèle. La seconde bénéficie
potentiellement du cache disque : le rapport doit être confirmé par répétitions
en ordre alterné avant toute conclusion générale.

| Paramètre / résultat | 1 thread demandé | 4 threads demandés |
|---|---:|---:|
| Workers effectifs et frames maximales en vol | 1 | 2 (budget mémoire) |
| Temps d'extraction | 23,4583 s | 12,3642 s |
| Débit | 8,27 images/s | 15,69 images/s |
| Pic RSS à la fin de l'extraction | 186 152 Kio ≈ 182 Mio | 314 776 Kio ≈ 307 Mio |
| Frames contenant assez de tags acceptés | 120 / 194 | 120 / 194 |

Rapport des temps observé : **1,90×**. Il compare le mode parallèle au mode
séquentiel de Kalibr2. Les caches sont identiques octet pour octet, empreinte SHA-256 :
`589fa783f2e2e243e6559c84923fe1491268e9d9557c62c45f0f3c59a70b72be`.
Le RSS inclut les bibliothèques et le travail de détection, pas une optimisation
Ceres (commande `extract`). Les 194 images ne sont pas retenues en RAM.

Reproduction depuis le conteneur de développement, après compilation :

```bash
build/test_solver --write-fixture build/benchmark-11mp 3
build/kalibr2 extract --dataset build/benchmark-11mp \
  --camera build/benchmark-11mp/camera.yaml --imu build/benchmark-11mp/imu.yaml \
  --target build/benchmark-11mp/target.yaml \
  --output build/benchmark-11mp-threads-1 --threads 1 --image-memory-mib 512
build/kalibr2 extract --dataset build/benchmark-11mp \
  --camera build/benchmark-11mp/camera.yaml --imu build/benchmark-11mp/imu.yaml \
  --target build/benchmark-11mp/target.yaml \
  --output build/benchmark-11mp-threads-4 --threads 4 --image-memory-mib 512
```

Les dossiers de sortie doivent être nouveaux. Lire `extraction.json` pour le
temps et `peak_rss_kib_at_extraction`. Il s'agit du maximum RSS du processus
jusqu'à ce stade, pas d'un plafond garanti par le budget d'images.

## Chaîne complète depuis les images

Un test CTest génère un mouvement analytique, les mesures IMU et 194 vraies images
PNG de 1280 × 960 par projection d'une mire. Il invoque le binaire CLI puis lit
le YAML pour comparer aux paramètres connus. Le mouvement IMU, le rendu de mire
et les projections de référence sont indépendants des splines du solveur.

Résultat observé : 106 frames retenues, erreur de rotation ≈ 0,000325 rad
(0,0186°), erreur de translation ≈ 1,43 mm et erreur de décalage temporel
≈ 0,138 ms. RMSE de reprojection ≈ 0,294 pixel. Le décalage imposé est +37,3 ms.
La relance depuis le cache produit les mêmes résultats numériques.

Le test direct du solveur, avec coins analytiques légèrement bruités, couvre
aussi les deux modèles `radtan` / `equidistant`, la gravité et les biais constants.
Il atteint environ 0,0051 pixel de RMSE. Ses erreurs plus faibles s'expliquent par
l'absence de rasterisation/détection et ne doivent pas être présentées comme une
précision réelle garantie.

## Couverture et travail restant

Les six tests couvrent détecteur, solveur, admission mémoire/cache, rosbag2,
aide CLI et parcours complet. Le test ROS vérifie SQLite et MCAP, encodages,
stride, endian 16 bits, temps capteur, messages invalides et absence d'erreurs
de libération de buffers. Les notices de fuite mémoire font échouer ce test.

Reste à réaliser : Kalibr 1 face à Kalibr2 sur le même bag réel, stabilité sur
plusieurs acquisitions, bruit/dérive réalistes, incertitudes, longue durée et
profils mémoire du solveur. Les liens du dataset officiel n'étaient pas accessibles
lors de cette session ; aucun résultat réel n'est inventé en remplacement.
