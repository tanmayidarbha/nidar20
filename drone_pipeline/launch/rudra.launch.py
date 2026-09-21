from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='drone_pipeline',
            executable='camera_node',
            name='rudra_camera',
            output='screen',
            parameters=[
                {
                    'drone_id': 'rudra',
                    'rtsp_url': 'rtsp://192.168.144.25:8554/main.264'
                }
            ]
        ),

        Node(
            package='drone_pipeline',
            executable='detection_node',
            name='rudra_detection',
            output='screen',
            parameters=[
                {
                    'drone_id': 'rudra'
                }
            ]
        ),

        Node(
            package='drone_pipeline',
            executable='geotag_node',
            name='rudra_geotag',
            output='screen',
            parameters=[
                {
                    'drone_id': 'rudra'
                }
            ]
        ),

        Node(
            package='drone_pipeline',
            executable='mission_control_node',
            name='rudra_mission_control',
            output='screen',
            parameters=[
                {
                    'drone_id': 'rudra'
                }
            ]
        ),

        Node(
            package='drone_pipeline',
            executable='fusion_node',
            name='rudra_fusion',
            output='screen'
        ),

        Node(
            package='drone_pipeline',
            executable='csv_logger_node',
            name='rudra_csv_logger',
            output='screen'
        ),

        Node(
            package='drone_pipeline',
            executable='rfd_sender_node',
            name='rudra_rfd_sender',
            output='screen',
            parameters=[
                {
                    'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                    'baudrate': 115200
                }
            ]
        ),

    ])
