#!/usr/bin/env python3

# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare





# define the current package name
current_package = "panther_exp_1"


def generate_launch_description():

    # common parameters
    use_sim_time = "true"

    # Define the required arguments
    custom_panther_package = "custom_panther_3"
    world_package = "dartec_world_4"
    gz_sim_system_plugin_path = "/home/rejin/Desktop/irp/plugin_test/dev_env/uwb_plugin/build"
    ign_gazebo_render_engine_path = "/usr/local/lib"
    gazebo_verbose_level = "0"
    ground_truth_publisher = "true"
    gazebo_ground_truth_topic = "/ground_truth_poses"
    ground_truth_topic = "/ground_truth_baselink_pose"
    pose_x = "4.0"
    pose_y = "15.0"
    pose_z = "0.2"
    rot_yaw = "1.5708"

    # define the arguments required for the panther tranforms package launch
    panther_tf_package = "panther_transforms"
    publish_map_to_baselink = "true"
    use_offset_values = 'true'
    uwb_frame_id = 'tag_link_0'
    base_link_frame_id = 'base_link'
    pose_topic = 'uwb/pose'
    transformed_world_pose_topic = 'uwb/world_baselink_pose'
    transformed_map_pose_topic = 'uwb/map_baselink_pose'
    odom_topic = 'odom'
    world_frame_id = 'odom'
    logging_level = '0'

    # define the parameters required for the panther fusion package
    panther_fusion_package = "panther_fusion"

    # define the parameters required for the panther navigation package
    panther_navigation_package = "panther_nav_1"
    logging_level_nav2 = "warn"







    # Launch the custom panther robot with the appropiate world selected
    panther_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare(custom_panther_package),
                    "launch",
                    "panther.launch.py",
                ]
            )
        ),
        launch_arguments={
            "world_package": world_package,
            "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_sim_system_plugin_path,
            "IGN_GAZEBO_RENDER_ENGINE_PATH": ign_gazebo_render_engine_path,
            "gazebo_verbose_level": gazebo_verbose_level,
            "ground_truth_publisher": ground_truth_publisher,
            "gazebo_ground_truth_topic": gazebo_ground_truth_topic,
            "ground_truth_topic": ground_truth_topic,
            "pose_x": pose_x,
            "pose_y": pose_y,
            "pose_z": pose_z,
            "rot_yaw": rot_yaw,
        }.items(),
    )

    # Launch the custom panther robot with the appropiate world selected
    panther_tf_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare(panther_tf_package),
                            "launch",
                            "panther_tf.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "publish_map_to_baselink": publish_map_to_baselink,
                    "pose_x": pose_x,
                    "pose_y": pose_y,
                    "pose_z": pose_z,
                    "rot_yaw": rot_yaw,
                    "use_offset_values": use_offset_values,
                    "uwb_frame_id": uwb_frame_id,
                    "base_link_frame_id": base_link_frame_id,
                    "pose_topic": pose_topic,
                    "transformed_world_pose_topic": transformed_world_pose_topic,
                    "transformed_map_pose_topic": transformed_map_pose_topic,
                    "odom_topic":odom_topic,
                    "world_frame_id": world_frame_id,
                    "logging_level": logging_level,
                }.items(),
            )
        ]
    )

    # Launch the panther fusion package
    panther_fusion_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare(panther_fusion_package),
                            "launch",
                            "panther_fusion.launch.py",
                        ]
                    )
                )
            )
        ]
    )

    # Launch the panther navigation package
    panther_navigation_launch = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare(panther_navigation_package),
                            "launch",
                            "panther_nav2.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "log_level": logging_level_nav2,
                }.items(),
            )
        ]
    )


    return LaunchDescription(
        [
            panther_launch,
            panther_tf_launch,
            panther_fusion_launch,
            panther_navigation_launch,
        ]
    )
