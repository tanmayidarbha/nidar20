from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # ==================================================
        # BARBARIK - SCOUT
        # ==================================================

        Node(
            package='drone_pipeline',
            executable='camera_node',
            name='barbarik_camera',
            parameters=[
                {
                    'drone_id': 'barbarik',
                    'rtsp_url': 'rtsp://192.168.144.25:8554/main.264'
                }
            ],
            output='screen'
        ),

        Node(
            package='drone_pipeline',
            executable='detection_node',
            name='barbarik_detection',
            parameters=[
                {
                    'drone_id': 'barbarik'
                }
            ],
            output='screen'
        ),

        Node(
            package='drone_pipeline',
            executable='geotag_node',
            name='barbarik_geotag',
            parameters=[
                {
                    'drone_id': 'barbarik'
                }
            ],
            output='screen'
        ),

        Node(
            package='drone_pipeline',
            executable='mission_control_node',
            name='barbarik_mission_control',
            parameters=[
                {
                    'drone_id': 'barbarik'
                }
            ],
            output='screen'
        ),

        # ==================================================
        # FUSION
        # ==================================================

        Node(
            package='drone_pipeline',
            executable='fusion_node',
            name='fusion',
            output='screen'
        ),

        # ==================================================
        # RUDRA - PAYLOAD DRONE
        # ==================================================

        Node(
            package='drone_pipeline',
            executable='nav_delivery_node',
            name='rudra_navigation',
            output='screen'
        ),

        Node(
            package='drone_pipeline',
            executable='payload_control_node',
            name='rudra_payload',
            parameters=[
                {
                    'drone_id': 'rudra'
                }
            ],
            output='screen'
        ),

        Node(
            package='drone_pipeline',
            executable='csv_logger_node',
            name='csv_logger',
            output='screen'
        ),

    ])
