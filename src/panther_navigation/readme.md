# This folder has packages realted to panther navigation

The packages are divided as per types of navigation provided
<br>
<br>



## 1. `panther_nav_1`

This package provides the just the controller from the navigation package. So it doesnot implement the global planner or the local planner. It just takes the goal and moves the robot to the goal in a straight line. So this package just implements the controller without any planners.

Since Nav2 requires a map to be able to funciton at all, we had to create an empty map file(a white pgn image and a yal file to go with it)
This map is then served using the map server node, which enables the navigation stack to work. But there are no obstacle deeciton cababiliy, just the point to point navigation.

<hr>