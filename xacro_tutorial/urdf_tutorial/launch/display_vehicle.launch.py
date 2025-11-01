from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('urdf_tutorial')
    model_path = os.path.join(pkg_path, 'urdf', 'my_vehicle.urdf')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'urdf.rviz')
    controller_config = os.path.join(pkg_path, 'config', 'diff_drive_controller.yaml')

    urdf_launch = IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
            launch_arguments={
                'urdf_package': 'urdf_tutorial',
                'urdf_package_path': PathJoinSubstitution(['urdf', 'my_vehicle.xacro'])
            }.items()
        )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', model_path])}]
    )

    return LaunchDescription([
        urdf_launch,
        robot_state_publisher,
    ])
