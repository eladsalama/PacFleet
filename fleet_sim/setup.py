from glob import glob

from setuptools import find_packages, setup

package_name = 'fleet_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elad Salama',
    maintainer_email='salamaelad@gmail.com',
    description='PacFleet: a robot swarm that plays coordinated Pac-Man, with Kalman tracking, neural detection, and auction task allocation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'world = fleet_sim.world:main',
            'coins = fleet_sim.coins:main',
            'ugv_sim = fleet_sim.ugv_sim:main',
            'hub = fleet_sim.hub:main',
            'ops_console = fleet_sim.ops_console:main',
            'train_classifier = fleet_sim.threat_classifier:main',
        ],
    },
)
