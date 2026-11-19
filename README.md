# SIMC_ROS2_LiDAR

Exploration autonome d'une piece avec un robot Cutebot et un lidar, sous ROS 2 Humble.

Le projet avance en trois etapes, une par dossier :

```
cutebot_teleop  ──►  cutebot_sim  ──►  cutebot_irl
 (clavier)           (Nav2, SLAM)      (vrai robot)
```

## Les dossiers

### `cutebot_teleop` : simulation pilotee au clavier

Premiere etape, tout fait main, sans SLAM ni Nav2.
La tortue de turtlesim joue le robot, `lidar_sim` simule le lidar,
et `nuage` construit un nuage de points a partir des mesures.
On pilote avec les fleches du clavier.

```
ros2 launch cutebot_teleop simulation.launch.py
```

### `cutebot_sim` : simulation avec Nav2

Meme robot et meme lidar simules, mais le robot se deplace tout seul :

- **slam_toolbox** construit la carte a partir du lidar ;
- **Nav2** calcule un chemin vers un objectif en evitant les murs.

Objectif : ajouter **explore_lite** pour que le robot explore la piece sans aide.
Pour l'instant, on donne les objectifs a la main avec "2D Goal Pose" dans RViz.

```
ros2 launch cutebot_sim simulation.launch.py
```

### `cutebot_irl` : le vrai Cutebot

Code pour le vrai robot (vrai lidar, vrais moteurs).
Vide pour l'instant : rempli une fois que `cutebot_sim` sera fini et operationnel.

### `m-explore-ros2`

Le paquet **explore_lite** (exploration par frontieres), qui n'existe pas en `apt` pour Humble.
Copie sans modification de [robo-friends/m-explore-ros2](https://github.com/robo-friends/m-explore-ros2)
(commit `326cf8a`).

## A installer

- **Ubuntu 22.04** (ou WSL2 sous Windows)
- **ROS 2 Humble** (version desktop : contient turtlesim et RViz)
- **colcon** (pour compiler)
- **slam_toolbox**
- **Nav2** (navigation2 et nav2_bringup)
- **xterm** (fenetre de la teleop)
- **git**
