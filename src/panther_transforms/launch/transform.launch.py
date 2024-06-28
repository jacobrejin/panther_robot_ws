from launch import LaunchDescription
from launch_ros.actions import Node

current_package = 'panther_transforms'  # Replace with your package name

def generate_launch_description():
    return LaunchDescription([
        Node(
            package=current_package,  # Replace with your package name
            executable='uwb_baseink_tf_broadcaster',
            name='uwb_baseink_tf_broadcaster',
            parameters=[
                {'uwb_frame_id': 'uwb_tag'},
                {'base_link_frame_id': 'base_link'}
            ]
        ),
    ])