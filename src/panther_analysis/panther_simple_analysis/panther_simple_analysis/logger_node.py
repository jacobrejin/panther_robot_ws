# this node is supposed to subscribe to the relevant pose topics and tfs and then log the data to a file
# THis file can later be preocessed by another node to generate graphs or relevant information

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Pose
import tf2_ros
from tf_transformations import euler_from_quaternion
import os
import datetime
import math

class PoseLogger(Node):

    def __init__(self):
        super().__init__('pose_logger')

        # Check if 'use_sim_time' parameter is already declared
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # declare parameters
        self.declare_parameter('ground_truth_topic', '/ground_truth_baselink_pose')
        self.declare_parameter('log_folder', '/home/rejin/Desktop/irp/ws/panther_robot_ws/log_folder')
        
        # get the parameters
        ground_truth_topic = self.get_parameter('ground_truth_topic').get_parameter_value().string_value
        log_folder = self.get_parameter('log_folder').get_parameter_value().string_value
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            ground_truth_topic,
            self.listener_callback,
            10)

        if not os.path.exists(log_folder):
            os.makedirs(log_folder)
        self.log_file_path = os.path.join(log_folder, 'pose_log.csv')

        # Write header to the log file
        with open(self.log_file_path, 'w') as log_file:
            log_file.write('time,X,Y,Yaw,ground_truth_x,ground_truth_y,ground_truth_yaw\n')


    def listener_callback(self, msg):
        ground_truth_pose = msg.pose.pose

        # get the curernt time
        now = self.get_clock().now()

        # Extract ground truth data
        ground_truth_x = ground_truth_pose.position.x
        ground_truth_y = ground_truth_pose.position.y
        quaternion = ground_truth_pose.orientation
        _,_,ground_truth_yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

        # Get current pose from TF
        try:
            trans = self.tf_buffer.lookup_transform('world', 'base_link', now)
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
            quaternion = trans.transform.rotation
            _,_,current_yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform: {e}')
            return

        # Log data to file
        with open(self.log_file_path, 'a') as log_file:
            log_file.write(f'{now.seconds}.{now.nanoseconds},{current_x},{current_y},{current_yaw},{ground_truth_x},{ground_truth_y},{ground_truth_yaw}\n')


def main(args=None):
    rclpy.init(args=args)
    node = PoseLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
