#   ros2 launch cutebot_sim simulation.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ici = get_package_share_directory('cutebot_sim')
    rviz_config = os.path.join(ici, 'rviz', 'simulation.rviz')
    slam_config = os.path.join(ici, 'config', 'slam.yaml')
    nav2_config = os.path.join(ici, 'config', 'nav2.yaml')
    nav2_launch = os.path.join(get_package_share_directory('nav2_bringup'),
                               'launch', 'navigation_launch.py')

    return LaunchDescription([
        # Le robot : turtlesim donne sa position sur /turtle1/pose.
        # Il ecoute /cmd_vel : il recoit directement les commandes de Nav2.
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim',
             remappings=[('turtle1/cmd_vel', '/cmd_vel')]),

        # Le lidar simule : /scan, /odom, /carte, /robot et TF odom -> base_link
        Node(package='cutebot_sim', executable='lidar_sim', name='lidar_sim', output='screen'),

        # TF fixe base_link -> laser_scan : le lidar est au centre du robot
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'laser_scan']),

        # slam_toolbox : lit /scan et la TF odom -> base_link, construit la carte /map
        # et publie la TF map -> odom
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             parameters=[slam_config, {'use_sim_time': False}]),

        # Nav2 : calcule le chemin vers l'objectif ("2D Goal Pose" dans RViz)
        # en evitant les murs de /map, et publie /cmd_vel
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={'use_sim_time': 'false', 'params_file': nav2_config}.items())]),

        # RViz, deja regle pour cette simulation (repere map, carte SLAM, scan, chemin).
        # Rendu logiciel : plus lent, mais evite les plantages lies a la carte graphique.
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config], output='log',
             additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}),
    ])
