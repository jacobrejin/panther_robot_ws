import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory


current_packagge = "panther_nav_1"


def generate_launch_description():
    # Declare launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['map_server']

    # Map fully qualified names to relative ones so the node's namespace can be prepended
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True
        ),
        allow_substs=True
    )

    # Declare launch arguments

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', 
        default_value=os.path.join(get_package_share_directory(current_packagge), 'config', 'nav2_param.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory(current_packagge), 'maps', 'empty_map.yaml'),
        description='Full path to map yaml file to load')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true')
   
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically startup the nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False', description='Whether to respawn if a node crashes')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='Log level')

    # Set environment variable for logging
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Node for map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=LaunchConfiguration('use_respawn'),
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=remappings
    )

    # Node for lifecycle manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'node_names': lifecycle_nodes
        }]
    )

    

    # Include the navigation_launch.py file
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_respawn': use_respawn,
            'log_level': log_level,
        }.items()
    )


    return LaunchDescription([
        declare_namespace_cmd,
        declare_map_yaml_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        stdout_linebuf_envvar,
        map_server_node,
        lifecycle_manager_node,
        navigation_launch
    ])
