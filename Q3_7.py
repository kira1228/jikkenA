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
    nav.setInitialPose(make_pose(0.0, 0.0, 0.0))

    # define current object coordinates and target coordinates
    obj_x, obj_y = 1.0,  0.5
    dst_x, dst_y = 2.5, -0.3

    # fist off, move to the object
    nav.goToPose(make_pose(obj_x, obj_y, 0.0))
    while not nav.isTaskComplete():
        time.sleep(0.2)
    if nav.getResult() != TaskResult.SUCCEEDED:
        print('failed to reach object coordinates'); return

    # calculate yaw that we use 
    dx = dst_x - obj_x
    dy = dst_y - obj_y
    yaw = math.atan2(dy, dx)

    # face the same direction
    nav.spin(yaw)
    while not nav.isTaskComplete():
        time.sleep(0.2)

    # move forward
    distance = math.hypot(dx, dy)
    nav.driveOnHeading(distance, speed=0.05)
    while not nav.isTaskComplete():
        time.sleep(0.2)

    if nav.getResult() == TaskResult.SUCCEEDED:
        print('Transported the object to the destination')
    else:
        print('Failed object transportation')

    nav.lifecycleShutdown()
    nav.destroyNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
