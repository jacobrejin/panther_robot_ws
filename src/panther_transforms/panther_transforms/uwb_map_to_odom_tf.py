# This node is supposed to publish the map -> baselink transform based on the odometry and the baselink pose
# This node was ideally supposed to work similar to the `uwb_world_to_odom_tf` node, But provided the `robot_localization` package
# takes care of calculatig the map -> odom transform, given the map -> basleink pose of any sensor, we just keep the node simple
# and let robot_localization handle the rest of the tfs
# This node has to publish the world -> odom tf, if we want to do some accuracy calcualtion later on
# the node can operate in 2 modes,
# 1. simulation (in this mode the intial spawn location of the robot can be prespecified as a parameter)
#    the node will use this information to create a world -> map transform (this is the inital offset wrt to the world of uwb)
# 2. Real world (in this mode the node will use the first few values of the uwb data to compute the world -> map transform)
#    It assues that the values are accurate. Or eles you can specify the initial spawn location of the robot as a parameter

import rclpy
from rclpy.node import Node
import tf2_ros
from nav_msgs.msg import Odometry
import tf_transformations
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
import math
import numpy as np
import tf2_geometry_msgs



class MapOdomTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('uwb_map_to_odom_tf')
        self.get_logger().info("Started")

        # Check if 'use_sim_time' parameter is already declared
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # Create a parameter for the initial robot spawn location
        self.initial_yaw = self.declare_parameter('rot_yaw', 0.0).get_parameter_value().double_value
        self.initial_x = self.declare_parameter('pose_x', 0.0).get_parameter_value().double_value
        self.initial_y = self.declare_parameter('pose_y', 0.0).get_parameter_value().double_value
        self.initial_z = self.declare_parameter('pose_z', 0.0).get_parameter_value().double_value
        # get other parameters
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.use_offset_values = self.declare_parameter('use_offset_values', True).get_parameter_value().bool_value
        self.transformed_map_pose_topic = self.declare_parameter('transformed_map_pose_topic', 'uwb/map_baselink_pose').get_parameter_value().string_value
        self.odom_topic = self.declare_parameter('odom_topic', '/odometry/filtered').get_parameter_value().string_value
        self.transformed_world_pose_topic = self.declare_parameter('transformed_world_pose_topic', 'uwb/world_baselink_pose').get_parameter_value().string_value



        # Create a TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # compute initial orientation matrix (rotation matrix)
        self.initial_rotation_matrix = tf_transformations.euler_matrix(0, 0, self.initial_yaw)

        # variables to store various poses
        self.current_odom_pose = None
        self.current_baselink_pose_in_world = None
        self.create_subscription(Odometry, self.odom_topic, self.odometry_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.transformed_world_pose_topic, self.baselink_pose_callback, 10)


        # Create a transform broadcaster and listener
        self.tf_broadcaster_world_map = tf2_ros.StaticTransformBroadcaster(self)
        # call the fuction to compute the world -> odom transform and publish a static transform
        self.compute_world_to_map()

        # Publisher for transformed pose
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.transformed_map_pose_topic,
            10
        )

        # self.timer = self.create_timer(0.5, self.compute_world_to_odom)


    def odometry_callback(self, msg):
        self.current_odom_pose = msg.pose.pose


    def baselink_pose_callback(self, msg):
        # self.current_baselink_pose_in_world = msg.pose.pose
        try:
            # invert world to map transformstamped
            self.map_to_world = self.tf_buffer.lookup_transform('map', 'world', rclpy.time.Time())
            # print he transform
            self.get_logger().info(f'map to world TF: {self.map_to_world}')
            # Transform the pose from the UWB tag frame to the corrected base link frame
            transformed_pose = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(msg, self.map_to_world)
            # set the z of the transformed pose to the intial z offset, as we do not have any z movevement, robot is 2d
            transformed_pose.pose.pose.position.z = self.initial_z
            # log the pose msgs x, y and z
            self.get_logger().info(f'map->basleink Pose: {transformed_pose}')
            # Publish the transformed pose
            self.publisher.publish(transformed_pose)
        except Exception as e:
            self.get_logger().error(f'Could not transform baselink pose: {str(e)}')



    def publish_world_to_map_sim(self):
        # publish a static transform from world to map using the intial offset values
        # Create a TransformStamped message
        self.world_to_map = TransformStamped()

        # Fill in the details of the transform
        self.world_to_map.header.stamp = self.get_clock().now().to_msg()
        self.world_to_map.header.frame_id = "world"
        self.world_to_map.child_frame_id = "map"
        self.world_to_map.transform.translation.x = self.initial_x
        self.world_to_map.transform.translation.y = self.initial_y
        self.world_to_map.transform.translation.z = self.initial_z

        # Convert the initial yaw to a quaternion for the rotation
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.initial_yaw)
        self.world_to_map.transform.rotation.x = quaternion[0]
        self.world_to_map.transform.rotation.y = quaternion[1]
        self.world_to_map.transform.rotation.z = quaternion[2]
        self.world_to_map.transform.rotation.w = quaternion[3]

        # Use the tf broadcaster to publish the transform
        self.tf_broadcaster_world_map.sendTransform(self.world_to_map)


    def compute_world_to_map(self):
        # if we are using the offset values specified,
        # we use the world to map funciton as the implementaion would be the same
        if self.use_offset_values:
            self.publish_world_to_map_sim()
        else:
            # we need to listen to the world -> basleink pose from the UWB and then 
            # use this to offset all the following map -> basleink values from the UWB
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomTransformBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
