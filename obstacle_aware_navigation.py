import rclpy
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def make_pose(x: float, y: float, z: float, w:float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rclpy.clock.Clock().now().to_msg()
    pose.pose.position.x = x 
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()

    # list of waypoints: [x, y, z, w]
    route = [
        (-0.178, -0.600, -0.116, 0.993),
        (2.178, -0.865, -0.987, 0.156),
        (1.083, 1.165, -0.117, 0,993),
        (3.028, 1.096, 0.902, 0.431),
        (1.829, 2.108, 0.970, 0.239)
    ]
    for x, y, z, w in route:
        waypoint = make_pose(x, y, z, w)
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
