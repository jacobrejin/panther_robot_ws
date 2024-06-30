import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped
from tf2_ros import TransformListener, Buffer, TransformException
from tf_transformations import quaternion_matrix, quaternion_from_matrix, concatenate_matrices, translation_matrix, euler_from_quaternion
import tf2_geometry_msgs

class UWBToBaseLinkPoseTransformer(Node):
    def __init__(self):
        super().__init__('uwb_to_base_link_pose_transformer')

        # Declare and get parameters
        self.uwb_frame_id = self.declare_parameter('uwb_frame_id', 'tag_link_0').get_parameter_value().string_value
        self.base_link_frame_id = self.declare_parameter('base_link_frame_id', 'base_link').get_parameter_value().string_value
        self.pose_topic = self.declare_parameter('pose_topic', 'uwb_pose').get_parameter_value().string_value
        self.transformed_pose_topic = self.declare_parameter('transformed_pose_topic', 'transformed_pose').get_parameter_value().string_value
        # get the world frame id
        self.world_frame_id = self.declare_parameter('world_frame_id', 'odom').get_parameter_value().string_value

        # Create the TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create the subscription to the UWB pose topic
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.listener_callback,
            10
        )
        
        # Publisher for transformed pose
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            self.transformed_pose_topic, 
            10
        )
    
    def listener_callback(self, msg):

        # print the pose data of the incoming message
        self.get_logger().info(f"Received pose: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}")
        # print the orientation data of the incoming message

        try:
            # Lookup the transform from world to base_link
            transform_world_to_base = self.tf_buffer.lookup_transform(
                self.world_frame_id,      # Target frame (odom or map)
                self.base_link_frame_id,  # Source frame (base_link)
                rclpy.time.Time()
            )

            # Extract the rotation matrix of the vehicle's current orientation in the world frame
            rotation_matrix = quaternion_matrix([
                transform_world_to_base.transform.rotation.x,
                transform_world_to_base.transform.rotation.y,
                transform_world_to_base.transform.rotation.z,
                transform_world_to_base.transform.rotation.w
            ])

            # Lookup the static transform from base_link to uwb_tag
            static_transform_base_to_uwb = self.tf_buffer.lookup_transform(
                self.uwb_frame_id,        # Source frame (uwb_tag)
                self.base_link_frame_id,  # Target frame (base_link)
                rclpy.time.Time())

            # Convert the static transform to a transformation matrix
            static_transform_matrix = translation_matrix([
                static_transform_base_to_uwb.transform.translation.x,
                static_transform_base_to_uwb.transform.translation.y,
                static_transform_base_to_uwb.transform.translation.z
            ])


            static_rotation_matrix = quaternion_matrix([
                static_transform_base_to_uwb.transform.rotation.x,
                static_transform_base_to_uwb.transform.rotation.y,
                static_transform_base_to_uwb.transform.rotation.z,
                static_transform_base_to_uwb.transform.rotation.w
            ])

            static_transformation_matrix = concatenate_matrices(static_transform_matrix, static_rotation_matrix)


            # Rotate the static transform matrix by the vehicle's current orientation
            rotated_transformation_matrix = concatenate_matrices(rotation_matrix, static_transformation_matrix)

            # Extract the translation and rotation from the rotated transform matrix
            rotated_translation = rotated_transformation_matrix[:3, 3]
            rotated_quaternion = quaternion_from_matrix(rotated_transformation_matrix)

            # Create a TransformStamped for the rotated transform
            rotated_transform = TransformStamped()
            rotated_transform.header.stamp = self.get_clock().now().to_msg()
            rotated_transform.header.frame_id = self.world_frame_id
            rotated_transform.child_frame_id = self.base_link_frame_id + "_corrected"
            rotated_transform.transform.translation.x = rotated_translation[0]
            rotated_transform.transform.translation.y = rotated_translation[1]
            rotated_transform.transform.translation.z = rotated_translation[2]
            rotated_transform.transform.rotation.x = rotated_quaternion[0]
            rotated_transform.transform.rotation.y = rotated_quaternion[1]
            rotated_transform.transform.rotation.z = rotated_quaternion[2]
            rotated_transform.transform.rotation.w = rotated_quaternion[3]

           
            # Transform the pose from the UWB tag frame to the corrected base link frame
            transformed_pose = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(msg, rotated_transform)
            
            # Publish the transformed pose
            self.publisher.publish(transformed_pose)
            
            self.get_logger().info(f'Transformed Pose: x={transformed_pose.pose.pose.position.x}, y={transformed_pose.pose.pose.position.y}, z={transformed_pose.pose.pose.position.z}')
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform pose: {ex}')

def main(args=None):
    rclpy.init(args=args)
    node = UWBToBaseLinkPoseTransformer()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
