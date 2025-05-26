import rclpy
import time
import math
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rclpy.clock.Clock().now().to_msg()
    pose.pose.position.x = x; pose.pose.position.y = y; pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0; pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0
    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()
    nav.setInitialPose(make_pose(0.0, 0.0, 0.0)) # set initial position

    # list of waypoints: [x, y, yaw]
    route = [
        (1.0, 0.5, 0.0),
        (2.0, -0.3, 0.0),
        (0.5, -1.5, 0.0),
    ]

    for x, y, yaw in route:
        waypoint = make_pose(x, y, yaw)
        # check obstacles
        costmap = nav.getGlobalCostmap()
        mx, my = costmap.worldToMapValidated(x, y)
        # If wx wy coordinates are invalid, (None,None) is returned.
        # Get the cost (np.uint8) of a cell in the costmap using mx(int), my(int) of Map Coordinate.
        if mx is None or costmap.getCostXY(mx, my) > 0:
            print(f'Detected obstacles or out of range: Skipping ({x:.2f},{y:.2f})')
            continue

        # move only safe waypoints
        nav.goToPose(waypoint)
        while not nav.isTaskComplete():
            time.sleep(0.2)

        res = nav.getResult()
        if res == TaskResult.SUCCEEDED:
            print(f'reached ({x:.2f},{y:.2f}) ')
        else:
            print(f'failed to reach ({x:.2f},{y:.2f}): {res}')

    nav.lifecycleShutdown()
    nav.destroyNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
