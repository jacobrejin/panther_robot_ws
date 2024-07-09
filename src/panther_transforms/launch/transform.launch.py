from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

current_package = 'panther_transforms'  # Replace with your package name

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time')

    rot_yaw = LaunchConfiguration('rot_yaw')
    declare_rot_yaw_arg = DeclareLaunchArgument(
        'rot_yaw', default_value='1.5708', description='Initial yaw of the robot')

    uwb_frame_id = LaunchConfiguration('uwb_frame_id')
    declare_uwb_frame_id_arg = DeclareLaunchArgument(
        'uwb_frame_id', default_value='tag_link_0', description='Frame ID of the UWB tag')

    base_link_frame_id = LaunchConfiguration('base_link_frame_id')
    declare_base_link_frame_id_arg = DeclareLaunchArgument(
        'base_link_frame_id', default_value='base_link', description='Frame ID of the base link')

    pose_topic = LaunchConfiguration('pose_topic')
    declare_pose_topic_arg = DeclareLaunchArgument(
        'pose_topic', default_value='uwb/pose', description='Pose topic of the UWB sensor')

    transformed_pose_topic = LaunchConfiguration('transformed_pose_topic')
    declare_transformed_pose_topic_arg = DeclareLaunchArgument(
        'transformed_pose_topic', default_value='uwb/baselink_pose', description='Transformed pose topic of the base link')

    world_frame_id = LaunchConfiguration('world_frame_id')
    declare_world_frame_id_arg = DeclareLaunchArgument(
        'world_frame_id', default_value='odom', description='World frame ID')

    logging_level = LaunchConfiguration('logging_level')
    declare_logging_level_arg = DeclareLaunchArgument(
        'logging_level', default_value='0', description='Logging level')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution(
            [get_package_share_directory(current_package), 'config', 'transfomr_test.rviz']),
        description='Path to the RViz configuration file')




    uwb_baseink_tf_broadcaster_node = Node(
        package=current_package,
        executable='uwb_baseink_tf_broadcaster',
        name='uwb_baseink_tf_broadcaster',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'rot_yaw': rot_yaw},
            {'uwb_frame_id': uwb_frame_id},
            {'base_link_frame_id': base_link_frame_id},
            {'pose_topic': pose_topic},
            {'transformed_pose_topic': transformed_pose_topic},
            {'world_frame_id': world_frame_id},
            {'logging_level': logging_level},
        ]
    )

    uwb_world_to_odom_tf_node = Node(
        package=current_package,
        executable='uwb_world_to_odom_tf',
        name='uwb_world_to_odom_tf',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'rot_yaw': rot_yaw},
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_rot_yaw_arg,
        declare_uwb_frame_id_arg,
        declare_base_link_frame_id_arg,
        declare_pose_topic_arg,
        declare_transformed_pose_topic_arg,
        declare_world_frame_id_arg,
        declare_logging_level_arg,
        declare_rviz_config_file_arg,
        uwb_baseink_tf_broadcaster_node,
        uwb_world_to_odom_tf_node,
        rviz_node,
    ])
