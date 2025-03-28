from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

# define the current package name
current_package = "fusion_test"



def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_panther',
            output='screen',
            parameters=[os.path.join(get_package_share_directory(current_package), 'params', 'ekf_panther.yaml')],
           ),
])


