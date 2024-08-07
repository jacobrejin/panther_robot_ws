from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import math


# define the current package name
current_package = "test_2d_map"






def generate_launch_description():

    # launch the node to convert the 3d pointcloud to 2d laser scan using the pointcloud_to_laserscan package
    point2laser = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/PointCloud2'),
                    ('scan', '/scan')],
        parameters=[{
            # 'target_frame': 'os_lidar',
            # 'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': -0.3,
            'max_height': 0.15,
            'angle_min': -math.pi,  # -M_PI/2
            'angle_max': math.pi,  # M_PI/2
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 50.00,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )

    # # Wrap the point2laser action in TimerAction to introduce a delay
    # point2laser_with_delay = TimerAction(
    #     period=10.0,
    #     actions=[point2laser]
    # )
    
    
    
    
    
    # Launch the SLAM Toolbox node, which will perform the mapping with the panther provided config file
    slam_param_file_path = get_package_share_directory(current_package) + "/config/slam_toolbox_params.yaml"
    # Define the SLAM Toolbox mapping launch command
    slam_toolbox_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                ]
            )
        ),
        launch_arguments={
            "slam_params_file": slam_param_file_path,
            "use_sim_time": "True",
        }.items(),
    )

    # Wrap the SLAM Toolbox launch action in TimerAction to introduce a delay
    slam_mapping_launch_with_delay = TimerAction(
        period=8.0,
        actions=[slam_toolbox_mapping_launch]
    )


    # Get the path to the RViz2 configuration file
    rviz_config_path = get_package_share_directory(current_package) + "/config/panther.rviz"
    # # Define the RViz2 launch command
    # rviz_launch_cmd = ['bash', '-c', f'rviz2 -d {rviz_config_path}']
    # # --use_sim_time:=True'
    # # Define the RViz2 launch action wrapped in a TimerAction
    # rviz_launch_with_delay = TimerAction(
    #     period=7.0,  # Adjust the delay period as needed (in seconds)
    #     actions=[
    #         ExecuteProcess(
    #             cmd=rviz_launch_cmd,
    #             output='screen'
    #         )
    #     ]
    # )

    # Define the RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace='',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    rviz_launch_with_delay = TimerAction(
        period=8.0,  # Adjust the delay period as needed (in seconds)
        actions=[rviz_node]
    )





    return LaunchDescription(
        [   
            # Launh the Dartec world with the custorm panther inside of it.
            # slam_toolbox_mapping_launch,
            point2laser,
            slam_mapping_launch_with_delay,
            # rviz_launch_with_delay,
            # point2laser_with_delay,
        ]
    )
