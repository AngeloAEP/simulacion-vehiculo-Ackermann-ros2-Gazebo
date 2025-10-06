from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')

    pkg_share = get_package_share_directory('vehiculo_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'vehiculo_autonomo-urdf.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'mundo1.world')

    # 1) Arranca Gazebo (Classic). NO agregues más spawns aquí.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'gui': gui, 'world': world_path}.items()
    )

    # 2) Publica robot_description ANTES de insertar el robot
    with open(urdf_path, 'r') as f:
        urdf_text = f.read()
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf_text}]
    )

    # 3) Inserta el modelo UNA SOLA VEZ, con retardo
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_path, '-entity', 'vehiculo', '-z', '0.35'],
        output='screen'
    )
    delayed_spawn = TimerAction(period=5.0, actions=[spawn])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        gazebo,
        rsp,
        delayed_spawn,
    ])
