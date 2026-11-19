# cutebot_teleop

Simulation avec teleop_key : lidar simule, nuage de points.

```
ros2 launch cutebot_teleop simulation.launch.py
```

## Les deux reperes

- **`monde`** : les coordonnees turtlesim (0, 0 = coin en bas a gauche de la piece).
  C'est la "verite" : le lidar simule et la vraie carte l'utilisent.
- **`odom`** : ce que le robot sait de lui meme. Il commence a (0, 0, 0) la ou le
  robot demarre, comme un vrai robot qui s'allume.

Arbre des reperes (TF) publie par `lidar_sim` :

```
odom ──► monde        (fixe : ou est le coin de la piece, vu depuis le point de depart)
odom ──► base_link    (bouge : position du robot depuis son demarrage)
base_link ──► laser_scan   (fixe : le lidar est au centre du robot)
```
