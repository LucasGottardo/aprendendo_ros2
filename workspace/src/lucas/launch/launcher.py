from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            name='log_level',
            default_value='INFO',
            choices=
['DEBUG', 'INFO','WARN','ERROR','FATAL'],
            description='Flag to set log level'
        ),

        Node(
            name='publisher',
            package='lucas',
            executable='publisher',
            arguments=
['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
    ])
