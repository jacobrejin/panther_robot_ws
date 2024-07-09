import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray




class PoseRepublisher(Node):
    def __init__(self):
        super().__init__('pose_republisher')

        # create a paramter for logging level
        self.declare_parameter('logging_level', 0)
        self.logging_level = self.get_parameter('logging_level').value

        self.subscription = self.create_subscription(
            PoseArray,
            '/ground_truth_poses',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Pose, '/ground_truth_baselink_pose', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if msg.poses:
            base_link_pose = msg.poses[0]
            self.publisher.publish(base_link_pose)
            if self.logging_level == 1:
                self.get_logger().info('Republished base_link pose')
                self.get_logger().info(f'Republished base_link position: {base_link_pose.position}')
                self.get_logger().info(f'Republished base_link orientation: {base_link_pose.orientation}')
        else:
            if self.logging_level == 1:
                self.get_logger().warn('No poses received in PoseArray')




def main(args=None):
    rclpy.init(args=args)
    pose_republisher = PoseRepublisher()
    rclpy.spin(pose_republisher)
    pose_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
