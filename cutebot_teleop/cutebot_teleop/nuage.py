#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry



# REGLAGES


NB_TOURS = 2              # nb de tours de lidar a chaque arret
ATTENTE = 0.1             # en sec
AVANCE_MIN = 0.05         # en m

TAILLE_POINT = 0.05       # taille des points dans RViz en m
COULEUR_NOUVEAUX = (1.0, 0.5, 0.0)   # orange
COULEUR_ANCIENS = (0.0, 0.4, 1.0)    # bleu


class Nuage(Node):

    def __init__(self):

        super().__init__('nuage')   # nom du noeud

        # La bibliotheque de points, en deux listes de points (x, y) sur la carte :

        self.nouveaux = []
        self.anciens = []

        # Position actuelle du robot, dans le repere 'odom' (0, 0 = point de depart).
        # Comme sur le vrai robot, on n'utilise que l'odometrie, pas turtlesim.
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_angle = 0.0

        # Etat du robot. On considere qu'il bouge au depart :
        # ainsi, son premier arret sera enregistre.
        self.en_mouvement = True
        self.heure_arret = None       # moment ou le robot s'est arrete
        self.tours_restants = 0       # nombre de scans encore a garder

        # Position du robot lors des dernieres mesures.
        # None au depart : le premier arret est donc toujours enregistre.
        self.mesure_x = None
        self.mesure_y = None

        # Subscribers
        self.create_subscription(Odometry, 'odom', self.nouvelle_position, 10)
        self.create_subscription(LaserScan, 'scan', self.nouveau_scan, 10)

        # Publisher du nuage de points
        self.publisher_nuage = self.create_publisher(Marker, 'nuage', 10)

        # On republie le nuage chaque seconde, pour que RViz le recoive
        # meme s'il est lance apres ce noeud
        self.create_timer(1.0, self.publier_nuage)

        self.get_logger().info('Noeud nuage demarre : %d tours gardes par arret' % NB_TOURS)

    def nouvelle_position(self, msg):

        # Position et orientation lues dans le message /odom
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        angle = 2 * math.atan2(q.z, q.w)   # quaternion -> angle (rotation autour de z)

        self.robot_x = x
        self.robot_y = y
        self.robot_angle = angle

        # Le robot est arrete si ses deux vitesses sont nulles
        arrete = (msg.twist.twist.linear.x == 0.0 and msg.twist.twist.angular.z == 0.0)

        if arrete and self.en_mouvement:
            # Il vient juste de s'arreter
            self.en_mouvement = False

            # A-t-il avance depuis les dernieres mesures ?
            # (distance entre la position actuelle et celle des dernieres mesures)
            if self.mesure_x is not None:
                avance = math.hypot(x - self.mesure_x, y - self.mesure_y)
                if avance < AVANCE_MIN:
                    # Il a seulement tourne sur lui-meme : pas de nouvelles mesures
                    self.get_logger().info('Robot arrete sans avancer (%.2f m) : pas de mesure'
                                           % avance)
                    return

            # Il a avance : on se prepare a garder NB_TOURS scans
            self.mesure_x = x
            self.mesure_y = y
            self.heure_arret = self.get_clock().now()
            self.tours_restants = NB_TOURS

            # Les points de l'arret precedent deviennent des "anciens" points
            self.anciens = self.anciens + self.nouveaux
            self.nouveaux = []
            self.get_logger().info('Robot arrete en x = %.2f, y = %.2f : enregistrement'
                                   % (x, y))

        elif not arrete and not self.en_mouvement:
            # Il vient de repartir : on arrete d'enregistrer
            self.en_mouvement = True
            self.tours_restants = 0

    def nouveau_scan(self, msg):

        # On ne garde le scan que si le robot est arrete et qu'il reste des tours a prendre
        if self.tours_restants == 0:
            return

        # On ignore les scans calcules avant l'arret (ou juste au moment de l'arret)
        heure_scan = Time.from_msg(msg.header.stamp)
        if (heure_scan - self.heure_arret).nanoseconds < ATTENTE * 1e9:
            return

        # On ajoute les points de ce scan
        nb_ajoutes = self.ajouter_points(msg)
        self.tours_restants = self.tours_restants - 1

        tour = NB_TOURS - self.tours_restants
        self.get_logger().info('Tour %d/%d garde : %d points ajoutes, %d au total'
                               % (tour, NB_TOURS, nb_ajoutes,
                                  len(self.anciens) + len(self.nouveaux)))

        self.publier_nuage()

    def ajouter_points(self, msg):

        nb_ajoutes = 0

        for i in range(len(msg.ranges)):
            d = msg.ranges[i]

            # On ignore les mesures invalides : rien touche (infini) ou hors portee
            if math.isinf(d) or math.isnan(d):
                continue
            if d < msg.range_min or d > msg.range_max:
                continue

            # Angle du rayon sur la carte = orientation du robot + angle du rayon
            angle = self.robot_angle + msg.angle_min + i * msg.angle_increment

            point = Point()
            point.x = self.robot_x + d * math.cos(angle)
            point.y = self.robot_y + d * math.sin(angle)
            self.nouveaux.append(point)
            nb_ajoutes = nb_ajoutes + 1

        return nb_ajoutes

    def publier_nuage(self):

        self.publier_points(self.anciens, COULEUR_ANCIENS, 0)
        self.publier_points(self.nouveaux, COULEUR_NOUVEAUX, 1)

    def publier_points(self, points, couleur, numero):

        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'      # points exprimes par rapport au point de depart
        msg.ns = 'nuage'
        msg.id = numero                    # 0 = anciens points, 1 = nouveaux points
        msg.type = Marker.POINTS           # on dessine des points
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0       # pas de rotation
        msg.scale.x = TAILLE_POINT         # largeur des points
        msg.scale.y = TAILLE_POINT         # hauteur des points
        msg.color.r = couleur[0]
        msg.color.g = couleur[1]
        msg.color.b = couleur[2]
        msg.color.a = 1.0                  # opaque
        msg.points = points

        self.publisher_nuage.publish(msg)


# -----------------------------------------------------------------------------
# DEMARRAGE DU PROGRAMME
# -----------------------------------------------------------------------------

def main():

    rclpy.init()            # demarre ROS
    rclpy.spin(Nuage())     # cree le noeud et le fait tourner en boucle
