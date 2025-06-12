from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_marker',
            executable='oakd_aruco_node',
            name='oakd_tracker',
            output='screen'
        ),
        Node(
            package='aruco_marker',
            executable='vesc_twist_node',
            name='vesc_controller',
            output='screen'
        )
    ])
