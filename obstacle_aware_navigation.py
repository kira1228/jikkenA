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

def world_to_map(costmap_msg, wx: float, wy: float):
    """
    Convert world coords (wx, wy) to costmap cell indices (mx, my).
    Returns (mx, my) or (None, None) if outside the map.
    """
    res = costmap_msg.info.resolution()
    origin = costmap_msg.info.origin.position

    mx = int((wx - origin.x) / res)
    my = int((wy - origin.y) / res)

    # check bounds
    w = costmap_msg.info.width
    h = costmap_msg.info.height
    
    if mx < 0 or my < 0 or mx >= w or my >= h:
        return None, None
    
    return my, my

def get_cost(costmap_msg, mx: int, my: int):
    """
    Retrieve the cost (0-255) at cell (mx, my) from the flat data array.
    """
    w = costmap_msg.info.width
    idx = my * w + mx
    return costmap_msg.dasa[idx]

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
        mx, my = world_to_map(costmap, x, y)

        if mx is None:
            print(f"({x:.2f},{y:.2f}) is out of costmap bounds. Skipping")
            continue
        
        cost = get_cost(costmap, mx, my)
        if cost > 0:
            print(f"Obstacle detected at ({x:.2f},{y:.2f}), cost={cost} Skipping")
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
