from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition


current_package = 'panther_transforms'  # Replace with your package name


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time')
    

    # Declare the launch argument for zeroing the initial pose
    publish_map_to_baselink = LaunchConfiguration('publish_map_to_baselink')
    declare_publish_map_to_baselink_arg = DeclareLaunchArgument(
        'publish_map_to_baselink',
        default_value='true',
        description='If true, we will run the approproate node to pubslish the map -> base_link tf \
                    else we will run the world -> base_link tf node. The map->basleink publishing node will also \
                    publish the world -> map tf. using the initial offset values'
    )

    
    # This is the intial offset at which the robot spawns
    rot_yaw = LaunchConfiguration('rot_yaw')
    declare_rot_yaw_arg = DeclareLaunchArgument(
        'rot_yaw', default_value='1.5708', description='Initial yaw of the robot')
    
    pose_x = LaunchConfiguration("pose_x")
    declare_pose_x_arg = DeclareLaunchArgument(
        "pose_x",
        default_value=["4.0"],
        description="Initial robot position in the global 'x' axis.",
    )

    pose_y = LaunchConfiguration("pose_y")
    declare_pose_y_arg = DeclareLaunchArgument(
        "pose_y",
        default_value=["15.0"],
        description="Initial robot position in the global 'y' axis.",
    )

    # This is assumed to not change,  since we are dealing with 2D nvaigaion ahead
    pose_z = LaunchConfiguration('pose_z')
    declare_pose_z_arg = DeclareLaunchArgument(
        'pose_z', default_value='0.2', description="Initial robot position in the global 'z' axis.")

    # TODO: Add detailed comment
    use_offset_values = LaunchConfiguration('use_offset_values')
    declare_use_offset_values_arg = DeclareLaunchArgument(
        'use_offset_values', default_value='true', description='This parameter when used will tell the node \
        to use the offset values specifed in the launch file instead of the  UWB measured offset values at the start.')

    uwb_frame_id = LaunchConfiguration('uwb_frame_id')
    declare_uwb_frame_id_arg = DeclareLaunchArgument(
        'uwb_frame_id', default_value='tag_link_0', description='Frame ID of the UWB tag')

    base_link_frame_id = LaunchConfiguration('base_link_frame_id')
    declare_base_link_frame_id_arg = DeclareLaunchArgument(
        'base_link_frame_id', default_value='base_link', description='Frame ID of the base link')

    pose_topic = LaunchConfiguration('pose_topic')
    declare_pose_topic_arg = DeclareLaunchArgument(
        'pose_topic', default_value='uwb/pose', description='Pose topic of the UWB sensor')

    transformed_world_pose_topic = LaunchConfiguration('transformed_world_pose_topic')
    declare_transformed_world_pose_topic = DeclareLaunchArgument(
        'transformed_world_pose_topic', default_value='uwb/world_baselink_pose', description='Transformed pose topic of the base link')
    
    transformed_map_pose_topic = LaunchConfiguration('transformed_map_pose_topic')
    declare_transformed_map_pose_topic = DeclareLaunchArgument(
        'transformed_map_pose_topic', default_value='uwb/map_baselink_pose', description='Transformed pose topic of the base link')
    
    odom_topic = LaunchConfiguration('odom_topic')
    declare_odom_topic_name = DeclareLaunchArgument(
        'odom_topic', default_value='odom', description='odom topic name')

    world_frame_id = LaunchConfiguration('world_frame_id')
    declare_world_frame_id_arg = DeclareLaunchArgument(
        'world_frame_id', default_value='odom', description='World frame ID, this is used by the uwb_world_baselink_pose_pub node \
            to get the rotation of the robot, it can be wrt to the odom frame or the map frame(if we have the map->odom tf being published)')

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
        executable='uwb_world_baselink_pose_pub',
        name='uwb_world_baselink_pose_pub',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'rot_yaw': rot_yaw},
            {'pose_z': pose_z},
            {'uwb_frame_id': uwb_frame_id},
            {'base_link_frame_id': base_link_frame_id},
            {'pose_topic': pose_topic},
            {'transformed_world_pose_topic': transformed_world_pose_topic},
            {'world_frame_id': world_frame_id},
            {'logging_level': 1},
        ]
    )


    # node to publish the world -> odom tf
    uwb_world_to_odom_tf_node = Node(
        package=current_package,
        executable='uwb_world_to_odom_tf',
        name='uwb_world_to_odom_tf',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'rot_yaw': rot_yaw},
            {'odom_topic': odom_topic},
            {'transformed_world_pose_topic': transformed_world_pose_topic},
        ],
        condition = UnlessCondition(publish_map_to_baselink)
    )

    # node to publish the map -> odom tf
    uwb_map_to_odom_tf_node = Node(
        package=current_package,
        executable='uwb_map_to_odom_tf',
        name='uwb_map_to_odom_tf',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'rot_yaw': rot_yaw},
            {'pose_x': pose_x},
            {'pose_y': pose_y},
            {'pose_z': pose_z},
            {'odom_topic': odom_topic},
            {'transformed_world_pose_topic': transformed_world_pose_topic},
            {'use_offset_values': use_offset_values},
            {'transformed_map_pose_topic': transformed_map_pose_topic}
        ],
        condition = IfCondition(publish_map_to_baselink)
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
        # declare_zero_initial_pose_arg,
        declare_rot_yaw_arg,
        declare_pose_x_arg,
        declare_pose_y_arg,
        declare_pose_z_arg,
        declare_publish_map_to_baselink_arg,
        declare_use_offset_values_arg,
        declare_uwb_frame_id_arg,
        declare_base_link_frame_id_arg,
        declare_pose_topic_arg,
        declare_transformed_world_pose_topic,
        declare_transformed_map_pose_topic,
        declare_world_frame_id_arg,
        declare_logging_level_arg,
        declare_rviz_config_file_arg,
        uwb_baseink_tf_broadcaster_node,
        uwb_world_to_odom_tf_node,
        uwb_map_to_odom_tf_node,
        # rviz_node,
    ])
