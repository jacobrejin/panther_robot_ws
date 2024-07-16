from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import time

def main():
    time.sleep(5)
    rclpy.init()
    navigator = BasicNavigator()

    # Activate navigation, since the script implies setup is needed
    navigator.lifecycleStartup()
    # navigator.waitUntilNav2Active()

    waypoints = [
        {"name": "wp1", "x": 1.2684, "y": -0.1173, "rot_z": -0.81, "rot_w": 0.57},
        {"name": "wp2", "x": 3.0, "y": -4.5, "rot_z": -0.5, "rot_w": 0.86},
        {"name": "wp3", "x": 14.3587, "y": -1.86, "rot_z": -0.707, "rot_w": 0.707},
        {"name": "wp4", "x": 20.0567, "y": -10.359, "rot_z": 0.99, "rot_w": 0.087},
        # {"name": "wp5", "x": 16.6502, "y": -10.7822},
        # {"name": "wp6", "x": 7.9266, "y": -11.951},
        # {"name": "wp7", "x": 15.543, "y": -14.9063},
        # {"name": "wp8", "x": 22.9257, "y": -31.5772}
    ]

    for waypoint in waypoints:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = waypoint["x"]
        goal_pose.pose.position.y = waypoint["y"]
        # set the rotation of the robot 
        goal_pose.pose.orientation.z = waypoint["rot_z"]
        goal_pose.pose.orientation.w = waypoint["rot_w"]

        print(f"Going to {waypoint['name']} at coordinates: x = {waypoint['x']}, y = {waypoint['y']}")

        navigator.goToPose(goal_pose)

        # Wait for the robot to reach the current goal
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                eta_seconds = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                # print(f'Estimated time of arrival to {waypoint["name"]}: {eta_seconds:.0f} seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time).nanoseconds > 600e9:
                    print(f"Navigation to {waypoint['name']} timed out, cancelling task.")
                    navigator.cancelTask()
                    break

        # Handle the result for each waypoint
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'Reached {waypoint["name"]} successfully!')
            time.sleep(5)
        elif result == TaskResult.CANCELED:
            print(f'Navigation to {waypoint["name"]} was canceled!')
        elif result == TaskResult.FAILED:
            print(f'Failed to reach {waypoint["name"]}!')
        else:
            print(f'Navigation to {waypoint["name"]} returned an invalid status!')

    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()
