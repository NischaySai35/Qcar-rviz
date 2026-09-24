"""Publish the real QCar2 URDF transforms and visual joint state for RViz."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    model_path = os.path.join(get_package_share_directory('qcar2'), 'urdf', 'QCar2.urdf')
    return LaunchDescription([
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='qcar2_robot_state_publisher', arguments=[model_path], output='screen',
        ),
        Node(
            package='qcar2_rviz_gui', executable='qcar2_model_joint_state.py',
            name='qcar2_model_joint_state', output='screen',
        ),
    ])
