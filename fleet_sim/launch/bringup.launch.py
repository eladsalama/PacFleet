"""One-command bringup of the whole mini-HUB stack.

    ros2 launch fleet_sim bringup.launch.py            # with RViz
    ros2 launch fleet_sim bringup.launch.py rviz:=false

Spawns: world + contacts + HUB + 3 namespaced UGVs (each: robot_state_publisher
loading the shared xacro with its own frame prefix/color + the simulator node).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# robot_id: (color_r, color_g, color_b, battery0)  -- neon arcade palette
FLEET = {
    'chomp': (1.00, 0.50, 0.05, 100.0),   # neon orange
    'dash': (0.10, 0.90, 1.00, 100.0),    # neon cyan
    'nibble': (1.00, 0.20, 0.85, 23.0),   # neon magenta; spawns low -> demos recharge + reassignment
}


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('fleet_description'),
                        'urdf', 'ugv.urdf.xacro')
    rviz_cfg = os.path.join(get_package_share_directory('fleet_sim'),
                            'rviz', 'fleet_ops.rviz')

    nodes = [
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='fleet_sim', executable='world', namespace='world',
             output='screen'),
        Node(package='fleet_sim', executable='coins', namespace='coins',
             output='screen'),
        Node(package='fleet_sim', executable='hub', namespace='hub',
             parameters=[{'robots': list(FLEET.keys())}], output='screen'),
        Node(package='fleet_sim', executable='ops_console', namespace='ops',
             parameters=[{'robots': list(FLEET.keys())}], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg],
             condition=IfCondition(LaunchConfiguration('rviz')),
             output='log'),
    ]

    for rid, (r, g, b, battery0) in FLEET.items():
        description = ParameterValue(
            Command(['xacro ', urdf,
                     ' prefix:=', f'{rid}/',
                     ' color_r:=', str(r),
                     ' color_g:=', str(g),
                     ' color_b:=', str(b)]),
            value_type=str)
        nodes += [
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 namespace=rid,
                 parameters=[{'robot_description': description}],
                 output='log'),
            Node(package='fleet_sim', executable='ugv_sim', namespace=rid,
                 parameters=[{'robot_id': rid, 'battery0': battery0}],
                 output='screen'),
        ]

    return LaunchDescription(nodes)
