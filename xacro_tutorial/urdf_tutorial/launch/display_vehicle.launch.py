from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('urdf_tutorial')
    model_path = os.path.join(pkg_path, 'urdf', 'my_vehicle.xacro')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'urdf.rviz')
    controller_config = os.path.join(pkg_path, 'config', 'diff_drive_controller.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', model_path])}]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config, {'robot_description': Command(['xacro ', model_path])}]
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller']
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        diff_drive_spawner,
        rviz
    ])
