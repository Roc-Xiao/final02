import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='answer',
            executable='image_processor_node',
            name='image_processor_node',
            output='screen'
        ),
        Node(
            package='answer',
            executable='game_logic_node',
            name='game_logic_node',
            output='screen'
        ),
    ])