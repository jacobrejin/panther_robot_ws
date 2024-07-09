# def compute_world_to_odom(self):
#         if self.current_odom_pose and self.current_baselink_pose_in_world:
#             try:
#                 # Step 1: Create a temporary transform from UWB (world) to base
#                 tmp_tf = tf_transformations.quaternion_from_euler(0, 0, self.initial_yaw)
#                 tmp_translation = [
#                     self.current_baselink_pose_in_world.position.x,
#                     self.current_baselink_pose_in_world.position.y,
#                     self.current_baselink_pose_in_world.position.z
#                 ]
#                 tmp_tf_matrix = tf_transformations.quaternion_matrix(tmp_tf)
#                 tmp_tf_matrix[:3, 3] = tmp_translation

#                 # Step 2: Inverse this to get base to UWB (world)
#                 inv_tmp_tf_matrix = np.linalg.inv(tmp_tf_matrix)

#                 # Step 3: Create odom pose matrix
#                 odom_translation = [
#                     self.current_odom_pose.position.x,
#                     self.current_odom_pose.position.y,
#                     self.current_odom_pose.position.z
#                 ]
#                 odom_orientation = [
#                     self.current_odom_pose.orientation.x,
#                     self.current_odom_pose.orientation.y,
#                     self.current_odom_pose.orientation.z,
#                     self.current_odom_pose.orientation.w
#                 ]
#                 odom_matrix = tf_transformations.quaternion_matrix(odom_orientation)
#                 odom_matrix[:3, 3] = odom_translation

#                 # Step 4: Compute the odom to map transformation matrix
#                 odom_to_map_matrix = np.dot(inv_tmp_tf_matrix, odom_matrix)

#                 # Extract translation and rotation
#                 trans = tf_transformations.translation_from_matrix(odom_to_map_matrix)
#                 rot = tf_transformations.quaternion_from_matrix(odom_to_map_matrix)

#                 # Step 5: Create and broadcast TransformStamped
#                 world_to_odom = TransformStamped()
#                 world_to_odom.header.stamp = self.get_clock().now().to_msg()
#                 world_to_odom.header.frame_id = "world"
#                 world_to_odom.child_frame_id = "odom"

#                 world_to_odom.transform.translation.x = trans[0]
#                 world_to_odom.transform.translation.y = trans[1]
#                 world_to_odom.transform.translation.z = trans[2]
#                 world_to_odom.transform.rotation.x = rot[0]
#                 world_to_odom.transform.rotation.y = rot[1]
#                 world_to_odom.transform.rotation.z = rot[2]
#                 world_to_odom.transform.rotation.w = rot[3]

#                 self.tf_broadcaster.sendTransform(world_to_odom)
#             except Exception as e:
#                 self.get_logger().error(f"Failed to compute transform: {e}")







# # ================================================================================================================



#     def compute_world_to_odom(self):
#         if self.current_odom_pose and self.current_baselink_pose_in_world:
#             world_to_odom = TransformStamped()
#             world_to_odom.header.stamp = self.get_clock().now().to_msg()
#             world_to_odom.header.frame_id = "world"
#             world_to_odom.child_frame_id = "odom"

#             world_to_odom.transform.translation.x = self.current_baselink_pose_in_world.position.x - self.current_odom_pose.position.x
#             world_to_odom.transform.translation.y = self.current_baselink_pose_in_world.position.y - self.current_odom_pose.position.y
#             world_to_odom.transform.translation.z = self.current_baselink_pose_in_world.position.z - self.current_odom_pose.position.z

#             # world_to_odom.transform.rotation = self.compute_relative_orientation(self.current_baselink_pose_in_world.orientation, self.current_odom_pose.orientation)
#             self.tf_broadcaster.sendTransform(world_to_odom)