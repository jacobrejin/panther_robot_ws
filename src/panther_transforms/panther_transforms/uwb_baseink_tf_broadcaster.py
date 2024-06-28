import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformListener, Buffer, LookupException
import tf2_geometry_msgs

class UWBToBaseLinkPoseTransformer(Node):
    def __init__(self):
        super().__init__('uwb_to_base_link_pose_transformer')

        # Declare and get parameters
        self.uwb_frame_id = self.declare_parameter('uwb_frame_id', 'uwb_tag').get_parameter_value().string_value
        self.base_link_frame_id = self.declare_parameter('base_link_frame_id', 'base_link').get_parameter_value().string_value
        self.pose_topic = self.declare_parameter('pose_topic', 'uwb_pose').get_parameter_value().string_value
        self.transformed_pose_topic = self.declare_parameter('transformed_pose_topic', 'transformed_pose').get_parameter_value().string_value

        # Create the TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create the publisher for the transformed pose
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, self.transformed_pose_topic, 10)

        # Create the subscription to the UWB pose topic
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        try:
            # Lookup the transform from the base link frame to the UWB tag frame
            transform_base_to_uwb = self.tf_buffer.lookup_transform(
                self.base_link_frame_id,
                self.uwb_frame_id,
                rclpy.time.Time()
            )

            # Transform the pose to the base link frame
            transformed_pose = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(msg, transform_base_to_uwb)

            # Publish the transformed pose in the base link frame
            self.publisher.publish(transformed_pose)

        except LookupException as e:
            self.get_logger().error(f"Could not transform pose: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UWBToBaseLinkPoseTransformer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
