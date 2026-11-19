# Explications de `lidar_sim.py`

Ce nœud fait croire à ROS qu'il y a un vrai robot avec un vrai lidar.
Il lit la position de la tortue (turtlesim), calcule ce qu'un lidar verrait
dans la pièce, et publie le tout comme le ferait un vrai robot.

```
turtlesim ──/turtle1/pose──► lidar_sim ──► /scan        (les mesures du lidar)
                                       ──► /odom        (position + vitesses du robot)
                                       ──► TF           (où sont les repères)
                                       ──► /carte       (la vraie carte de la pièce)
                                       ──► /robot       (flèche rouge pour RViz)
```

---

## 1. Les réglages (en haut du fichier)

| Nom | Valeur | Rôle |
|---|---|---|
| `NB_POINTS` | 450 | nombre de mesures par tour de lidar |
| `DISTANCE_MIN` / `DISTANCE_MAX` | 0,02 m / 12 m | portée du lidar |
| `BRUIT` | 0,015 m | erreur aléatoire ajoutée à chaque mesure (un vrai lidar n'est pas parfait) |
| `FREQUENCE` | 10 | tours de lidar par seconde |
| `TAILLE_CASE` | 0,05 m | taille d'une case de la carte (5 cm) |
| `LARGEUR` / `HAUTEUR` | 11,09 m | taille de la fenêtre turtlesim |
| `MURS` | liste | chaque mur est un rectangle `(x_min, y_min, x_max, y_max)` |

Les 4 premiers murs sont les bords de la pièce, les autres sont les murs intérieurs.

## 2. Les deux repères

- **`monde`** : les coordonnées turtlesim (0, 0 = coin en bas à gauche). C'est la vérité.
- **`odom`** : ce que le robot sait de lui-même. Il vaut (0, 0, 0) là où le robot démarre.

Voir le `README.md` pour le schéma des repères.

---

## 3. Les fonctions (hors du nœud)

### `creer_carte()`
Découpe la pièce en cases de 5 cm et met `True` dans chaque case dont le centre est
dans un mur. Résultat : `carte[ligne][colonne]`, la ligne 0 est en bas.

### `est_un_mur(carte, x, y)`
Dit si le point (x, y) est dans un mur :
- `True` : mur ;
- `False` : libre ;
- `None` : **hors de la carte** (en dehors des 11,09 m × 11,09 m)

### `mesurer_un_rayon(carte, x, y, angle)`
Simule **un seul rayon** du lidar : on part du robot et on avance par petits pas
(2,5 cm, plus petit qu'une case pour ne pas sauter un mur) dans la direction `angle`.
- On touche un mur → on renvoie la distance parcourue.
- On sort de la carte ou on dépasse `DISTANCE_MAX` → on renvoie `inf` (rien touché).

### `faire_un_scan(carte, x, y, angle_robot)`
Fait **un tour complet** : 450 rayons, de -180° (derrière) à +180°, 0° = devant le robot.
On ajoute l'orientation du robot à chaque rayon pour le placer sur la carte,
puis un peu de bruit aléatoire à chaque mesure.

### `vers_odom(x, y, angle, depart)`
Convertit une position du repère `monde` vers le repère `odom` :
on retire le point de départ, puis on tourne pour que l'angle de départ devienne 0.

### `transformation(parent, enfant, x, y, angle, heure)`
Fabrique un **message TF** : « le repère `enfant` est en (x, y), tourné de `angle`,
par rapport au repère `parent`, à cette heure-ci ».
L'angle est écrit en **quaternion** : pour une rotation à plat,
`z = sin(angle/2)` et `w = cos(angle/2)`.

---

## 4. Le nœud `LidarSim`

### `__init__` : le démarrage
- crée la carte ;
- s'abonne à `/turtle1/pose` (la position de la tortue) ;
- crée les publishers : `/scan`, `/odom`, `/carte`, `/robot` et les TF ;
- `/carte` est en **transient local** : RViz reçoit la carte même s'il démarre après ce nœud ;
- lance deux timers : `publier_scan` 10 fois par seconde, `publier_carte` 1 fois par seconde.

### `nouvelle_position(msg)` : à chaque position de turtlesim (environ 60 fois/s)
- retient la position de la tortue ;
- **la toute première fois**, elle devient le point de départ (origine de `odom`) et on publie `publier_monde` ;
- publie l'odométrie avec `publier_odom`.

### `publier_monde()` : une seule fois
Publie la TF **fixe** `odom → monde` : où est le coin de la pièce vu depuis le point de départ.
Sert à placer la vraie carte au bon endroit dans RViz.

### `publier_odom(pose)` : à chaque position
- TF `odom → base_link` : où est le robot par rapport à son départ ;
- message `/odom` : la même position + les vitesses (utilisées par Nav2 pour connaître la vitesse du robot).

C'est ce que donneraient les encodeurs des roues sur un vrai robot.

### `publier_scan()` : 10 fois par seconde
- ne fait rien tant que turtlesim n'a pas envoyé de position ;
- copie la position (elle pourrait changer pendant le calcul) ;
- fait un scan avec `faire_un_scan` ;
- remplit un message `LaserScan` dans le repère `laser_scan` (angles, portée, distances) et le publie sur `/scan` ;
- publie la flèche rouge du robot.

### `publier_carte()` : 1 fois par seconde
Publie la vraie carte sur `/carte` (repère `monde`) :
ROS veut une seule liste, ligne par ligne, avec **100 = mur** et **0 = libre**.

### `publier_robot(heure)`
Publie une **flèche rouge** dans le repère `base_link` : comme elle est attachée au robot,
elle le suit toute seule dans RViz.

---

## 5. `main()`
Démarre ROS, crée le nœud `LidarSim` et le fait tourner (`spin`) jusqu'à Ctrl+C.
