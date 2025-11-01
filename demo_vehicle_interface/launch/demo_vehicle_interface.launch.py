from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import FileContent, LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ''use_sim_time'' is used to have ros2 use /clock topic for the time source
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
 
    xacro_file = FileContent(
        PathJoinSubstitution([FindPackageShare('urdf_tutorial'), 'urdf', 'my_vehicle.xacro']))
    
    
    robot_description = ParameterValue(
        Command(['xacro', xacro_file]),
        value_type=str
    )
    

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': "/home/cmodi/cmodi/ros2_ws/src/mls_ros2_packages/xacro_tutorial/urdf_tutorial/urdf/my_vehicle.xacro"}],
            ),
        Node(
            package='demo_vehicle_interface',
            executable='demo_vehicle_interface_node',
            name='demo_vehicle_interface',
            output='screen'),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )
    ])