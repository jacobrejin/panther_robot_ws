# 

import rclpy
from rclpy.node import Node
import tf2_ros
from nav_msgs.msg import Odometry
import tf_transformations
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
import math
import numpy as np

class WorldOdomTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('world_odom_transform_broadcaster')

        self.get_logger().info("uwb world to odom Node: Started")

        # create a initial yaw parameter
        self.initial_yaw = self.declare_parameter('rot_yaw', 1.5708).get_parameter_value().double_value
        # Check if 'use_sim_time' parameter is already declared
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_odom_pose = None
        self.current_baselink_pose_in_world = None

        # Get the parameters
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value



        # compute initial orientation matrix (rotation matrix)
        self.initial_rotation_matrix = tf_transformations.euler_matrix(0, 0, self.initial_yaw)

        self.create_subscription(Odometry, '/odometry/filtered', self.odometry_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/uwb/baselink_pose', self.baselink_pose_callback, 10)

        self.timer = self.create_timer(0.5, self.compute_world_to_odom)

    def odometry_callback(self, msg):
        self.current_odom_pose = msg.pose.pose

    def baselink_pose_callback(self, msg):
        self.current_baselink_pose_in_world = msg.pose.pose


    def compute_world_to_odom(self):
        if self.current_odom_pose and self.current_baselink_pose_in_world:
            try:
             
                # Step 1: Create a temporary transform from UWB (world) to base
                world_baselink_translation = [
                    self.current_baselink_pose_in_world.position.x,
                    self.current_baselink_pose_in_world.position.y,
                    self.current_baselink_pose_in_world.position.z
                ]

                # # Step 2: Inverse this to get base to UWB (world)
                # inv_tmp_tf_matrix = np.linalg.inv(world_baselink_matrix)

                # Step 3: Create odom pose matrix
                odom_translation = [
                    self.current_odom_pose.position.x,
                    self.current_odom_pose.position.y,
                    self.current_odom_pose.position.z
                ]
                odom_orientation = [
                    self.current_odom_pose.orientation.x,
                    self.current_odom_pose.orientation.y,
                    self.current_odom_pose.orientation.z,
                    self.current_odom_pose.orientation.w
                ]

                # Step 1: Create a rotation matrix for the initial yaw (no translation)
                intial_yaw_rotaion_matrix = tf_transformations.euler_matrix(0, 0, self.initial_yaw)

                # Step 2: Create a rotation matrix for odom->base_link orientation
                odom_baselink_rotaion_matrix = tf_transformations.quaternion_matrix(odom_orientation)

                # Step 3: Combine the two rotation matrices to get the world->base_link rotation matrix
                world_baselink_tf= np.dot(intial_yaw_rotaion_matrix, odom_baselink_rotaion_matrix)
                # Add the translation to the matrix
                world_baselink_tf[:3, 3] = world_baselink_translation

                # Step 4: Combine the two rotation matrices to get the world->base_link rotation matrix
                odom_baselink_tf = tf_transformations.quaternion_matrix([0, 0, 0, 1])
                # Add the translation to the matrix
                odom_baselink_tf[:3, 3] = odom_translation

                # Step 5: Invert the odom -> base_link matrix to get the base_link -> odom matrix
                odom_baselink_tf_inv = np.linalg.inv(odom_baselink_tf)

                # Step 6: Compute the map -> odom transformation matrix
                map_to_odom_matrix = np.dot(world_baselink_tf, odom_baselink_tf_inv)

                # Step 7: Extract translation and rotation
                trans = tf_transformations.translation_from_matrix(map_to_odom_matrix)
                rot = tf_transformations.quaternion_from_matrix(map_to_odom_matrix)

                # Step 8: Create and broadcast TransformStamped
                world_to_odom = TransformStamped()
                world_to_odom.header.stamp = self.get_clock().now().to_msg()
                world_to_odom.header.frame_id = "world"
                world_to_odom.child_frame_id = "odom"

                world_to_odom.transform.translation.x = trans[0]
                world_to_odom.transform.translation.y = trans[1]
                world_to_odom.transform.translation.z = trans[2]

                # account for the initial offset if any
                tmp_tf = tf_transformations.quaternion_from_euler(0, 0, self.initial_yaw)
                world_to_odom.transform.rotation.x = tmp_tf[0]
                world_to_odom.transform.rotation.y = tmp_tf[1]
                world_to_odom.transform.rotation.z = tmp_tf[2]
                world_to_odom.transform.rotation.w = tmp_tf[3]

                # world_to_odom.transform.rotation.x = rot[0]
                # world_to_odom.transform.rotation.y = rot[1]
                # world_to_odom.transform.rotation.z = rot[2]
                # world_to_odom.transform.rotation.w = rot[3]

                self.tf_broadcaster.sendTransform(world_to_odom)
            except Exception as e:
                self.get_logger().error(f"Failed to compute transform: {e}")




def main(args=None):
    rclpy.init(args=args)
    node = WorldOdomTransformBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
