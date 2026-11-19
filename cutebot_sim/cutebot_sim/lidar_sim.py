#!/usr/bin/env python3
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from turtlesim.msg import Pose


#Reglages (distances en m, angles en rad) 

NB_POINTS = 450           # mesures par tour
DISTANCE_MIN = 0.02
DISTANCE_MAX = 12.0
BRUIT = 0.015             # erreur 
FREQUENCE = 10.0          # tours/s

TAILLE_CASE = 0.05
LARGEUR = 11.09           # taille fenetre turtlesim
HAUTEUR = 11.09

# Chaque mur est un rectangle : (x_min, y_min, x_max, y_max)
MURS = [
    (0.00, 0.00, 11.09, 0.11),
    (0.00, 10.98, 11.09, 11.09),
    (0.00, 0.00, 0.11, 11.09),
    (10.98, 0.00, 11.09, 11.09),
    (2.22, 0.00, 2.33, 6.65),
    (4.44, 2.22, 4.55, 11.09),
    (6.65, 0.00, 6.76, 4.44),
    (6.65, 6.65, 8.87, 6.76),
    (7.21, 8.32, 8.04, 9.15),
]

# Deux reperes :
#   'monde' : les coordonnees turtlesim (0, 0 = coin en bas a gauche de la piece).
#             C'est la "verite" : le lidar simule et la vraie carte l'utilisent.
#   'odom'  : ce que le robot sait de lui meme. Il commence a (0, 0, 0) la ou le
#             robot demarre, comme un vrai robot qui s'allume.




def creer_carte():
    # carte[ligne][colonne] = True si mur. Ligne 0 = en bas.
    nb_colonnes = round(LARGEUR / TAILLE_CASE)
    nb_lignes = round(HAUTEUR / TAILLE_CASE)

    carte = []
    for ligne in range(nb_lignes):
        carte.append([False] * nb_colonnes)

    # Une case est un mur si son centre est dans un rectangle de MURS
    for ligne in range(nb_lignes):
        for colonne in range(nb_colonnes):
            x = (colonne + 0.5) * TAILLE_CASE
            y = (ligne + 0.5) * TAILLE_CASE
            for (x_min, y_min, x_max, y_max) in MURS:
                if x_min <= x <= x_max and y_min <= y <= y_max:
                    carte[ligne][colonne] = True

    return carte


def est_un_mur(carte, x, y):
    # Renvoie True (mur), False (libre) ou None (hors de la carte)
    colonne = int(x / TAILLE_CASE)
    ligne = int(y / TAILLE_CASE)

    if colonne < 0 or colonne >= len(carte[0]) or ligne < 0 or ligne >= len(carte):
        return None

    return carte[ligne][colonne]


def mesurer_un_rayon(carte, x, y, angle):
    # On avance le long du rayon par petits pas jusqu'a toucher un mur.
    # Rien touche -> infini, comme un vrai lidar.
    distance = DISTANCE_MIN
    pas = TAILLE_CASE / 2   # plus petit qu'une case, sinon on pourrait sauter un mur

    while distance < DISTANCE_MAX:
        point_x = x + distance * math.cos(angle)
        point_y = y + distance * math.sin(angle)

        resultat = est_un_mur(carte, point_x, point_y)
        if resultat is None:
            return float('inf')
        if resultat:
            return distance

        distance = distance + pas

    return float('inf')


def faire_un_scan(carte, x, y, angle_robot):
    distances = []
    ecart = 2 * math.pi / NB_POINTS

    for i in range(NB_POINTS):
        # Le premier rayon part derriere le robot (-180 degres), 0 = devant
        angle_rayon = -math.pi + i * ecart
        # Sur la carte, il faut ajouter l'orientation du robot
        angle_carte = angle_robot + angle_rayon

        d = mesurer_un_rayon(carte, x, y, angle_carte)
        d = d + random.gauss(0, BRUIT)
        distances.append(d)

    return distances


def vers_odom(x, y, angle, depart):
    # Position dans 'monde' -> position dans 'odom' (par rapport au point de depart).
    # On retire la position de depart, puis on tourne de -angle_depart.
    x0, y0, a0 = depart
    dx = x - x0
    dy = y - y0
    c = math.cos(-a0)
    s = math.sin(-a0)
    return c * dx - s * dy, s * dx + c * dy, angle - a0


def transformation(parent, enfant, x, y, angle, heure):
    msg = TransformStamped()
    msg.header.stamp = heure
    msg.header.frame_id = parent
    msg.child_frame_id = enfant
    msg.transform.translation.x = x
    msg.transform.translation.y = y
    # Rotation autour de l'axe vertical en "quaternion" : z = sin(a/2), w = cos(a/2)
    msg.transform.rotation.z = math.sin(angle / 2)
    msg.transform.rotation.w = math.cos(angle / 2)
    return msg


# --- Noeud ROS ---

class LidarSim(Node):

    def __init__(self):
        super().__init__('lidar_sim')

        self.carte = creer_carte()

        # Position turtlesim (repere 'monde'), utilisee pour simuler le lidar
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_angle = 0.0

        # Point de depart (x, y, angle) dans 'monde' : la premiere position recue.
        # None tant que turtlesim n'a rien envoye.
        self.depart = None

        self.create_subscription(Pose, '/turtle1/pose', self.nouvelle_position, 10)

        self.publisher_scan = self.create_publisher(LaserScan, 'scan', 10)
        self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)

        # "Transient local" : RViz recoit la carte meme s'il demarre apres ce noeud
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher_carte = self.create_publisher(OccupancyGrid, 'carte', qos)

        self.publisher_robot = self.create_publisher(Marker, 'robot', 10)
        self.publisher_tf = TransformBroadcaster(self)
        self.publisher_tf_fixe = StaticTransformBroadcaster(self)

        self.create_timer(1.0 / FREQUENCE, self.publier_scan)
        self.create_timer(1.0, self.publier_carte)

        self.get_logger().info('Lidar simule demarre : lit /turtle1/pose, publie /scan')

    def nouvelle_position(self, msg):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_angle = msg.theta

        if self.depart is None:
            # Premiere position recue : c'est le point de depart, l'origine de 'odom'
            self.depart = (msg.x, msg.y, msg.theta)
            self.publier_monde()
            self.get_logger().info('Depart en x = %.2f, y = %.2f (monde) : odom commence a 0'
                                   % (msg.x, msg.y))

        # Publiee a chaque position (60 Hz)
        self.publier_odom(msg)

    def publier_monde(self):
        # TF fixe odom -> monde : ou se trouve le coin de la piece (origine de 'monde')
        # vu depuis le point de depart. Sert a placer la vraie carte dans RViz.
        x, y, angle = vers_odom(0.0, 0.0, 0.0, self.depart)
        heure = self.get_clock().now().to_msg()
        self.publisher_tf_fixe.sendTransform(transformation('odom', 'monde', x, y, angle, heure))

    def publier_odom(self, pose):
        # Position par rapport au point de depart, comme les encodeurs d'un vrai robot
        heure = self.get_clock().now().to_msg()
        x, y, angle = vers_odom(pose.x, pose.y, pose.theta, self.depart)

        # TF odom -> base_link : ou est le robot
        self.publisher_tf.sendTransform(transformation('odom', 'base_link', x, y, angle, heure))

        # /odom : la meme position + les vitesses (Nav2 s'en sert pour connaitre la vitesse du robot)
        msg = Odometry()
        msg.header.stamp = heure
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(angle / 2)
        msg.pose.pose.orientation.w = math.cos(angle / 2)
        msg.twist.twist.linear.x = pose.linear_velocity     # vitesses dans le repere du robot
        msg.twist.twist.angular.z = pose.angular_velocity
        self.publisher_odom.publish(msg)

    def publier_scan(self):
        # Rien a simuler tant que turtlesim n'a pas donne de position
        if self.depart is None:
            return

        # On copie la position maintenant : elle peut changer pendant le calcul,
        # et le scan doit correspondre exactement a la position publiee
        x = self.robot_x
        y = self.robot_y
        angle = self.robot_angle
        heure = self.get_clock().now().to_msg()

        distances = faire_un_scan(self.carte, x, y, angle)

        msg = LaserScan()
        msg.header.stamp = heure
        msg.header.frame_id = 'laser_scan'   # mesures exprimees par rapport au lidar
        msg.angle_min = -math.pi
        msg.angle_increment = 2 * math.pi / NB_POINTS
        msg.angle_max = msg.angle_min + (NB_POINTS - 1) * msg.angle_increment
        msg.scan_time = 1.0 / FREQUENCE
        msg.range_min = DISTANCE_MIN
        msg.range_max = DISTANCE_MAX
        msg.ranges = distances

        self.publisher_scan.publish(msg)
        self.publier_robot(heure)

    def publier_carte(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'monde'   # la vraie carte est dessinee en coordonnees turtlesim
        msg.info.resolution = TAILLE_CASE
        msg.info.width = len(self.carte[0])
        msg.info.height = len(self.carte)
        msg.info.origin.orientation.w = 1.0

        # ROS veut une seule liste, ligne par ligne : 100 = mur, 0 = libre
        cases = []
        for ligne in self.carte:
            for case in ligne:
                if case:
                    cases.append(100)
                else:
                    cases.append(0)
        msg.data = cases

        self.publisher_carte.publish(msg)

    def publier_robot(self, heure):
        # Fleche rouge dans le repere 'base_link' : elle suit le robot toute seule
        msg = Marker()
        msg.header.stamp = heure
        msg.header.frame_id = 'base_link'
        msg.ns = 'robot'
        msg.id = 0
        msg.type = Marker.ARROW
        msg.action = Marker.ADD
        msg.pose.position.x = -0.2      # moitie de la longueur : fleche centree sur le robot
        msg.pose.orientation.w = 1.0
        msg.scale.x = 0.4
        msg.scale.y = 0.15
        msg.scale.z = 0.15
        msg.color.r = 1.0
        msg.color.a = 1.0

        self.publisher_robot.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(LidarSim())
