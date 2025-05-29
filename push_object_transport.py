import rclpy
import time
import math
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rclpy.clock.Clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose

def compute_yaw(from_x, from_y, to_x, to_y):
    """return moving direction as yaw (rad)"""
    return math.atan2(to_y - from_y, to_x - from_x)

def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()
    
    # define object corrdinates and target coordinates
    obj_x, obj_y = 1.0,  0.5
    dst_x, dst_y = 2.5, -0.3

    # go to object 
    nav.goToPose(make_pose(obj_x, obj_y, 0.0))
    while not nav.isTaskComplete():
        time.sleep(0.1)
    if nav.getResult() != TaskResult.SUCCEEDED:
        print('Failed to reach the object')
        return

    # define waypoints route
    path_points = [
        (obj_x, obj_y),
        (1.2, 1.0),
        (dst_x, dst_y),
    ]

    # define each yaw and create PoseStamped
    push_poses = []
    for i in range(len(path_points) - 1):
        fx, fy = path_points[i]
        tx, ty = path_points[i+1]
        yaw = compute_yaw(fx, fy, tx, ty)
        push_poses.append(make_pose(tx, ty, yaw))

    # go to each poses
    nav.followWaypoints(push_poses)
    while not nav.isTaskComplete():
        time.sleep(0.1)

    if nav.getResult() == TaskResult.SUCCEEDED:
        print('Success')
    else:
        print('Failed to transport:', nav.getResult())

    nav.lifecycleShutdown()
    nav.destroyNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
