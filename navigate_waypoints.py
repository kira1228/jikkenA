import rclpy
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def make_pose(x: float, y: float, z: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rclpy.clock.Clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    # list of waypoints: [x, y, z]
    route = [
        (-0.178, -0.600, 0),
        (2.178, -0.865, 0),
        (1.083, 1.165, 0),
        (3.028, 1.096, 0),
        (1.829, 2.108, 0)
    ]
    poses = [make_pose(x, y, z) for x, y, z in route]

    navigator.followWaypoints(poses)
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
