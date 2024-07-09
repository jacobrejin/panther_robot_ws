from launch import LaunchDescription
from launch_ros.actions import Node

current_package = 'panther_transforms'  # Replace with your package name

def generate_launch_description():
    return LaunchDescription([
        
        
        # This node converted the uwb tag pose in world frame to the baselink pose of the robot in the world frame
        Node(
            package=current_package,  # Replace with your package name
            executable='uwb_world_to_odom_tf',
            name='uwb_world_to_odom_tf',
            parameters=[
                {'use_sim_time': True},
                {'rot_yaw': 1.5708}, # This is the initial yaw of the robot
                # {'uwb_frame_id': 'tag_link_0'},
                # {'base_link_frame_id': 'base_link'},
                # {'pose_topic': 'uwb/pose'}, # This is the pose of the tag, which ideally we would receive from the UWB sensor (in world frame)
                # {'transformed_pose_topic': 'uwb/baselink_pose'}, # This the output calcualted transformed pose of the Baselink (in the world frame)
                # {'world_frame_id': 'odom'},# This is the world frame, in which the orientaion of the robot is calculated (will be better to set to map, in case one is avialable)
                # {'logging_level': 1},
            ]
        ),


        
    ])