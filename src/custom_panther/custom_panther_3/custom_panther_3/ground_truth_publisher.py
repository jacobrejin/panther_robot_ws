import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray




class PoseRepublisher(Node):
    def __init__(self):
        super().__init__('pose_republisher')

        # create a paramter for the node
        self.logging_level = self.declare_parameter('logging_level', 0).get_parameter_value().integer_value
        self.input_topic = self.declare_parameter('gazebo_ground_truth_topic', '/ground_truth_poses').get_parameter_value().string_value
        self.output_topic = self.declare_parameter('ground_truth_topic', '/ground_truth_baselink_pose').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            PoseArray,
            self.input_topic,
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Pose, self.output_topic, 10)
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
