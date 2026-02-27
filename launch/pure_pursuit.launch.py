from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=EnvironmentVariable('ROVER_NAME', default_value='RR03'),
        description='Robot namespace for all topics',
    )

    config = PathJoinSubstitution(
        [FindPackageShare('pure_pursuit_tracker'), 'config', 'params.yaml']
    )

    return LaunchDescription([
        robot_name_arg,
        Node(
            package='pure_pursuit_tracker',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            namespace=LaunchConfiguration('robot_name'),
            parameters=[config],
            output='screen'
        )
    ])
