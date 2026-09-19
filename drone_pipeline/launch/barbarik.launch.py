from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='drone_pipeline',
            executable='nav_delivery_node',
            name='barbarik_navigation',
            output='screen',
            parameters=[
                {
                    'drone_id': 'barbarik'
                }
            ]
        ),

        Node(
            package='drone_pipeline',
            executable='payload_control_node',
            name='barbarik_payload_control',
            output='screen',
            parameters=[
                {
                    'drone_id': 'barbarik'
                }
            ]
        ),

        Node(
            package='drone_pipeline',
            executable='rfd_receiver_node',
            name='barbarik_rfd_receiver',
            output='screen',
            parameters=[
                {
                    'serial_port': '/dev/REPLACE_ME',
                    'baudrate': 115200
                }
            ]
        ),

    ])
