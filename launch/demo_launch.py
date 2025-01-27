from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_gnc_sensor_abstraction',
            executable='demo_sensor',  # Correct executable name
            name='sensor_node',
            output='screen',
        ),
    ])

