
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare



# This is not being used as we weer able to generate the camera msg topic with the right frame id


# define the current package name
current_package = "custom_panther_2"


def generate_launch_description():
    
    depth_cam_data2cam_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cam3Tolink',
        output='log',
        arguments=[
            '0.0', '0.0', '0.0', 
            '0.0', '0.0', '0.0', 
            'camera_center_optical_frame', 
            'panther/base_link/camera_stereolabs_zed_depth'
        ]
    )


    return LaunchDescription(
        [
            depth_cam_data2cam_link_tf,
        ]
    )
