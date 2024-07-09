import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped

class MapOdomTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('map_odom_transform_broadcaster')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_odom_position = None

        self.create_subscription(Odometry, '/odometry/filtered', self.odometry_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/uwb/baselink_pose', self.uwb_callback, 10)

    def odometry_callback(self, msg):
        self.current_odom_position = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }

    def uwb_callback(self, msg):
        if self.current_odom_position is None:
            return

        try:
            world_to_map = self.tf_buffer.lookup_transform('map', 'world', rclpy.time.Time())

            uwb_position_in_world = msg.pose
            uwb_position_in_map = tf2_geometry_msgs.do_transform_pose(uwb_position_in_world, world_to_map)

            map_to_odom = TransformStamped()
            map_to_odom.header.stamp = self.get_clock().now().to_msg()
            map_to_odom.header.frame_id = "map"
            map_to_odom.child_frame_id = "odom"

            map_to_odom.transform.translation.x = uwb_position_in_map.pose.position.x - self.current_odom_position['x']
            map_to_odom.transform.translation.y = uwb_position_in_map.pose.position.y - self.current_odom_position['y']
            map_to_odom.transform.translation.z = uwb_position_in_map.pose.position.z - self.current_odom_position['z']

            # Assuming no rotation correction for simplicity
            map_to_odom.transform.rotation.x = 0.0
            map_to_odom.transform.rotation.y = 0.0
            map_to_odom.transform.rotation.z = 0.0
            map_to_odom.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(map_to_odom)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Transform lookup failed: {str(e)}")

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
