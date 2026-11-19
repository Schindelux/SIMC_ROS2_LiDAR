# cutebot_sim

Simulation avec Nav2 : lidar simule, slam_toolbox, Nav2.

```
ros2 launch cutebot_sim simulation.launch.py
```

## Objectif

Le robot doit explorer la piece **tout seul** et en faire la carte, avec :

- **slam_toolbox** : construit la carte `/map` a partir du lidar ;
- **Nav2** : calcule un chemin vers un objectif en evitant les murs ;
- **explore_lite** (`m-explore-ros2`) : choisit automatiquement le prochain objectif
  (les frontieres entre zones connues et inconnues) et l'envoie a Nav2.



## Les deux reperes

- **`monde`** : les coordonnees turtlesim (0, 0 = coin en bas a gauche de la piece).
  C'est la "verite" : le lidar simule et la vraie carte l'utilisent.
- **`odom`** : ce que le robot sait de lui meme. Il commence a (0, 0, 0) la ou le
  robot demarre, comme un vrai robot qui s'allume.

Arbre des reperes (TF) :

```
map ──► odom          (slam_toolbox : corrige la derive de l'odometrie)
odom ──► monde        (lidar_sim, fixe : ou est le coin de la piece, vu depuis le point de depart)
odom ──► base_link    (lidar_sim, bouge : position du robot depuis son demarrage)
base_link ──► laser_scan   (fixe : le lidar est au centre du robot)
```
