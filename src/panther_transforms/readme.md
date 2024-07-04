# 1. uwb_baseink_tf_broadcaster Node

This node converts the uwb tag pose data which is in the world coordinates to the 
baselink pose data in the world coordinates. 

It cannot be acheived direclty using the TF2 library as we have to account for the orientaion of the robot 
to calcuate the pose in the world coordinates. you can read through the below link to learn more about the issue

https://github.com/cra-ros-pkg/robot_localization/issues/203


# 2. baselink_to_zero_tf_broadcaster Node

This node listens to the baselink pose data and converts the baselink pose data to the zero pose data in the map coordinates.
This is uselful as, when using mapping packages, they assume the start point of the robot to be (0,0,0) position with a (0,0,0) orientaion. 