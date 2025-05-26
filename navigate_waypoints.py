import rclpy
import time
import math
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rclpy.clock.Clock().now().to_msg()
    # set z coordinate explicitly 0 as we are only considering plane route
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    
    # create initial position
    init = make_pose(0.0, 0.0, 0.0)
    navigator.setInitialPose(init)

    # list of waypoints : [x, y, yaw]
    route = [
        (1.0, 0.5, 0.0),
        (2.0, -0.3, 0.0),
        (0.5, -1.5, 0.0),
    ]
    poses = [make_pose(x, y, yaw) for x, y, yaw in route]

    navigator.goThroughPoses(poses)
    while not navigator.isTaskComplete():
        time.sleep(0.2)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Reached all waypoints')
    else:
        print('Move failed: ', result)

    navigator.lifecycleShutdown()
    navigator.destroyNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
