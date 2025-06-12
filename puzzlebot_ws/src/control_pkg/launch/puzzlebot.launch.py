from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_pkg',
            executable='line_follower_intersection',
            name='follow_line_node',
            output='screen',
        ),
        Node(
            package='control_pkg',
            executable='turn_node',
            name='turn_executor_node',
            output='screen',
        ),
        Node(
            package='control_pkg',
            executable='state_machine',
            name='puzzlebot_state_machine',
            output='screen',
        ),

        Node(
            package='detector_pkg',
            executable='sign_detector',
            name='simple_traffic_sign_detector',
            output='screen',
        ),
        Node(
            package='detector_pkg',
            executable='traffic_detector',
            name='traffic_signal_detector',
            output='screen',
        ),
        # Puedes añadir más nodos aquí
        # Node(...),
    ])
