#   ros2 launch cutebot_teleop simulation.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ici = get_package_share_directory('cutebot_teleop')
    rviz_config = os.path.join(ici, 'rviz', 'simulation.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('teleop', default_value='true'),

        # Le robot : turtlesim donne sa position sur /turtle1/pose
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim'),

        # Le lidar simule : /scan, /carte, /robot et TF odom -> base_link
        Node(package='cutebot_teleop', executable='lidar_sim', name='lidar_sim', output='screen'),

        # TF fixe base_link -> laser_scan : le lidar est au centre du robot
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'laser_scan']),

        # Le nuage de points : 2 tours de lidar gardes a chaque arret
        Node(package='cutebot_teleop', executable='nuage', name='nuage', output='screen'),

        # La teleop au clavier : publie directement sur /turtle1/cmd_vel, sans filtre,
        # donc la tortue peut traverser les murs.
        # Un appui = la vitesse pendant 1 s : 0,5 m ou environ 30 degres.
        Node(package='turtlesim', executable='turtle_teleop_key', name='teleop',
             prefix='xterm -title "Teleop tortue" -e',
             parameters=[{'scale_linear': 0.5, 'scale_angular': 0.5}],
             condition=IfCondition(LaunchConfiguration('teleop'))),

        # RViz, deja regle pour cette simulation (repere odom, scan, nuage, vraie carte).
        # Rendu logiciel : plus lent, mais evite les plantages lies a la carte graphique.
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config], output='log',
             additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}),
    ])
