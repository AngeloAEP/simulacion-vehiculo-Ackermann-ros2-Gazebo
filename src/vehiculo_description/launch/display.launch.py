import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# 👇 Cambia esto si tu URDF se llama distinto
URDF_NAME = "vehiculo_autonomo-urdf.urdf"

def generate_launch_description():
    pkg_share = get_package_share_directory('vehiculo_description')
    urdf_file = os.path.join(pkg_share, 'urdf', URDF_NAME)

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])
