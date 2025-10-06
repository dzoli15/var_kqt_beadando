from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generates the launch description for the cruise control system."""
    return LaunchDescription([
        Node(
            package='var_kqt_beadando',
            executable='speed_sensor_node',
            name='speed_sensor',
            output='screen'
        ),
        Node(
            package='var_kqt_beadando',
            executable='cruise_control_node',
            name='cruise_control',
            output='screen'
        ),
    ])