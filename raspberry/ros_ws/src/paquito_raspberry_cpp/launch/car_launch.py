from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('viz_package_cpp')
    path_to_urdf = os.path.join(package_dir, 'urdf', 'robot.urdf')
    with open(path_to_urdf, 'r') as f:
        robot_desc = f.read()
    return LaunchDescription([
        Node(
            package='paquito_raspberry_cpp',
            namespace='paquito',
            executable='command_executor',
            name='command_executor',
            output='screen',
        ),
        Node(
            package='paquito_raspberry_cpp',
            name='car_velocity_subscriber',
            executable='car_velocity_subscriber',
            output='screen',
        ),
    ])

