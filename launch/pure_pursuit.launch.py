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
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='odom',
        description='Input odometry topic name (relative to namespace)',
    )
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='cmd_vel',
        description='Output velocity command topic name (relative to namespace)',
    )

    config = PathJoinSubstitution(
        [FindPackageShare('pure_pursuit_tracker'), 'config', 'params.yaml']
    )

    return LaunchDescription([
        robot_name_arg,
        odom_topic_arg,
        cmd_vel_topic_arg,
        Node(
            package='pure_pursuit_tracker',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            namespace=LaunchConfiguration('robot_name'),
            parameters=[config],
            remappings=[
                ('odom', LaunchConfiguration('odom_topic')),
                ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
            ],
            output='screen'
        )
    ])
