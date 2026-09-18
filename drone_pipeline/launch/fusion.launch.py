from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='drone_pipeline',
            executable='fusion_node',
            name='fusion',
            output='screen'
        ),
    ])
