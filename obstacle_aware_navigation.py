#!/usr/bin/env python3
import rclpy
import time
import math

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer_interface import TransformStamped

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def make_pose(x: float, y: float, z: float, w: float) -> PoseStamped:
    """
    マップ座標系(frame_id='map')での目標姿勢を生成するヘルパー関数
    """
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

def lookup_transform(buffer: Buffer, from_frame: str, to_frame: str, time) -> TransformStamped:
    """
    TF2 バッファから from_frame → to_frame の変換を取得するヘルパー関数
    （例：'map' → 'base_link' など。）
    """
    try:
        # time を None にすると最新時刻の変換を取得する
        return buffer.lookup_transform(
            to_frame,   # target_frame
            from_frame, # source_frame
            time,
            timeout=rclpy.duration.Duration(seconds=0.5)
        )
    except (LookupException, ConnectivityException, ExtrapolationException, TransformException) as e:
        return None

def world_to_robot(transform_map_to_base: TransformStamped, wx: float, wy: float):
    """
    マップ座標系における点 (wx, wy) をロボット(base_link)座標系に変換する。
    戻り値は (rx, ry) で、base_link 座標系での座標。
    """
    # transform_map_to_base には、map → base_link の変換が入っている想定
    # (source_frame='map', target_frame='base_link')
    # RT行列を使って座標変換する
    # 平面 (z=0) と仮定し、x,y だけを見ればよい
    dx = wx - transform_map_to_base.transform.translation.x
    dy = wy - transform_map_to_base.transform.translation.y

    # 回転（クォータニオン → yaw）を取得
    q = transform_map_to_base.transform.rotation
    # yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # ワールド座標系（map）→ロボット座標系回転： 平面上での回転行列は
    # [ cos(-yaw)  -sin(-yaw) ] [dx]
    # [ sin(-yaw)   cos(-yaw) ] [dy]
    # = [ cos(yaw) sin(yaw); -sin(yaw) cos(yaw) ] をかけると
    # robot_x =  dx * cos(yaw) + dy * sin(yaw)
    # robot_y = -dx * sin(yaw) + dy * cos(yaw)
    rx =  dx * math.cos(yaw) + dy * math.sin(yaw)
    ry = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return rx, ry

def is_obstacle_in_waypoint(node, buffer: Buffer, wx: float, wy: float) -> bool:
    """
    LiDAR の最新データを取得し、ウェイポイント (wx, wy) が障害物で塞がれているかを判定する関数。
    - node: rclpy ノード
    - buffer: TF2 バッファ（'map'→'base_link' 変換が取得できる状態）
    - (wx, wy): マップ座標系でのウェイポイント位置
    戻り値: True=障害物あり→スキップすべき, False=障害物なし→進行可
    """

    # 1) map → base_link の変換を取得
    t_map_to_robot = lookup_transform(buffer, 'map', 'base_link', rclpy.time.Time())
    if t_map_to_robot is None:
        node.get_logger().warn('TF lookup failed. Treat as obstacle present.')
        return True

    # 2) ウェイポイントをロボット座標系 (base_link) に変換
    rx, ry = world_to_robot(t_map_to_robot, wx, wy)

    # 3) ウェイポイントまでの直線距離と方位角を計算
    distance_to_wp = math.hypot(rx, ry)
    angle_to_wp = math.atan2(ry, rx)  # 角度：ロボット前方を 0 rad とする

    # 4) 最新の LaserScan をブロッキング取得（秒数は適宜調整）
    try:
        scan: LaserScan = node.get_publisher_or_subscriber('/scan', LaserScan).last_message
        # ※ rclpy では簡単にブロッキング購読できないので、
        #    以下では wait_for_message を使った形にする
        scan = rclpy.wait_for_message('/scan', LaserScan, node, timeout_sec=1.0)
    except Exception:
        node.get_logger().warn('Failed to receive LaserScan. Treat as obstacle present.')
        return True

    if scan is None:
        node.get_logger().warn('No LaserScan received. Treat as obstacle present.')
        return True

    # 5) LaserScan データ（scan.ranges）から「angle_to_wp」に該当するインデックスを求める
    # LaserScan でカバーしている角度範囲: [angle_min, angle_max], インクリメント: angle_increment
    # idx = (angle_to_wp - angle_min) / angle_increment
    # ただし idx が負や最大を超えるときは輪切りに clamp する
    angle_min = scan.angle_min
    angle_max = scan.angle_max
    angle_inc = scan.angle_increment
    # LaserScan は原点（ロボット中心）から各方向へ走査
    # angle_to_wp が angle_min〜angle_max の間でない場合、外へ外れた角度とみなす
    if angle_to_wp < angle_min:
        angle_to_wp = angle_min
    elif angle_to_wp > angle_max:
        angle_to_wp = angle_max

    idx = int(round((angle_to_wp - angle_min) / angle_inc))
    # インデックスの範囲補正
    idx = max(0, min(idx, len(scan.ranges) - 1))

    # 6) 当該方向のレンジ値を取得
    laser_distance = scan.ranges[idx]
    # nan/inf の場合は判定できないので「障害なし」とみなす
    if math.isinf(laser_distance) or math.isnan(laser_distance):
        return False

    # 7) 「レーザーの距離」 < 「ウェイポイントまでの距離」の場合は途中に障害物ありと判断
    if laser_distance < distance_to_wp:
        # 十分マージンを取りたい場合は、ちょっと短く見積もる（例: laser_distance - 0.1 < distance_to_wp など）
        return True

    return False

def main():
    rclpy.init()
    node = rclpy.create_node('lidar_obstacle_checker')

    # BasicNavigator を立ち上げ
    nav = BasicNavigator(node=node)
    nav.waitUntilNav2Active()

    # TF2 バッファとリスナーを作成
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    # サブスクライバを準備しておく（LaserScan のみを扱いたいので型だけ定義する）
    lidar_sub = node.create_subscription(
        LaserScan,
        '/scan',
        lambda msg: None,
        10
    )
    # ※ 実装例では rclpy.wait_for_message('/scan', LaserScan, node) を使って最新を取る方式にしてもよい

    # ウェイポイントリスト [x, y, z, w]
    route = [
        (-0.178, -0.600, -0.116, 0.993),
        (2.178, -0.865, -0.987, 0.156),
        (1.083,  1.165, -0.117, 0.993),
        (3.028,  1.096,  0.902, 0.431),
        (1.829,  2.108,  0.970, 0.239)
    ]

    for x, y, z, w in route:
        # ① LiDAR ベースで障害物チェック
        obstacle = is_obstacle_in_waypoint(node, tf_buffer, x, y)
        if obstacle:
            node.get_logger().info(f'LiDAR: Obstacle detected toward ({x:.2f}, {y:.2f}) → SKIP')
            continue  # 障害物があればスキップ

        # ② 障害物がなければ、BasicNavigator で移動開始
        waypoint = make_pose(x, y, z, w)
        nav.goToPose(waypoint)

        # ③ ゴール到達まで待機
        while not nav.isTaskComplete():
            time.sleep(0.2)

        res = nav.getResult()
        if res == TaskResult.SUCCEEDED:
            node.get_logger().info(f'Reached ({x:.2f}, {y:.2f})')
        else:
            node.get_logger().warn(f'Failed to reach ({x:.2f}, {y:.2f}): {res}')

    # ナビゲータのクリーンアップ
    nav.lifecycleShutdown()
    nav.destroyNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
