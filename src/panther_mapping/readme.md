# This Folder has all the packages which are realed to mapping


# 1. map_2d

This package is a copy from the group project done for my MSc. robotics course. It converts the 3D lidar pointcloud data to 2d by using another ros package. And the resulting 2D laser scan is being used to create a 2D occupancy grid map. The map is then saved as a yaml file and a pgm file. The map is then served using the map server node. The map is then used by the navigation stack to navigate the robot.


