# 1. uwb_baseink_tf_broadcaster Node

This node converts the uwb tag pose data which is in the world coordinates to the 
baselink pose data in the world coordinates. 

It cannot be acheived direclty using the TF2 library as we have to account for the orientaion of the robot 
to calcuate the pose in the world coordinates. you can read through the below link to learn more about the issue

https://github.com/cra-ros-pkg/robot_localization/issues/203





# 2. uwb_map_to_odom_tf Node

This node listens to the baselink pose data and converts the baselink pose data to the zero pose data in the map coordinates.
This is uselful as, when using mapping packages, they assume the start point of the robot to be (0,0,0) position with a (0,0,0) orientaion. 



# 3. uwb_world_to_odom_tf node

this node is similar to the uwb__to_odom_tf node.
It assumes we have the position of the robot baselink being published by a node (in our case the `uwb_baseink_tf_broadcaster`)

So provided we have 
1. world -> baselink (from the uwb sensor)
2. odom -> baselink (from the robot_localization node which fuses IMU and odometry data)
3. We need to find the world -> odom transform

So its a trivial transformation task which can be easily solved using the following formula. <br>

`T_(map->odom) = T_(map->base_link) * T_(odom->base_link)^-1`


I followed this post to implement the required transformations. It has explanation regarding the transformations. <br>
https://answers.ros.org/question/401747/publishing-map-odom-transform-using-custom-localization-and-slam-algorithms/