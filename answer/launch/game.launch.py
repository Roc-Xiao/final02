import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='answer',
            executable='game_node',
            name='game_node',
            output='screen'
        ),
    ])
