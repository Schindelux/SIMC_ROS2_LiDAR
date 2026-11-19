from glob import glob

from setuptools import setup

package_name = 'cutebot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='schindelux',
    maintainer_email='schindelux@todo.todo',
    description='Simulation avec Nav2 : lidar simule, slam_toolbox, Nav2',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            # nom de la commande = dossier.fichier:fonction
            'lidar_sim = cutebot_sim.lidar_sim:main',
        ],
    },
)
