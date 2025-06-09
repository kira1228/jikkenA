#!/usr/bin/env python3
import rclpy, time, math, cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException, TransformException

class CVObstacleNavigator:
    def __init__(self):
        # rclpy 初期化 & BasicNavigator() が内部でノードを作成
        rclpy.init()
        self.nav = BasicNavigator()
        # ここで nav.waitUntilNav2Active() すると内部ノードが起動する
        self.nav.waitUntilNav2Active()

        # 画像用
        self.bridge = CvBridge()
        self.latest_image = None
        self.nav.create_subscription(
            Image, '/image_raw', self._image_cb, 10
        )

        # TF バッファ＆リスナー
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self.nav)

        # ウェイポイント
        self.route = [
            (-0.178, -0.600, -0.116, 0.993),
            ( 2.178, -0.865, -0.987, 0.156),
            ( 1.083,  1.165, -0.117, 0.993),
            ( 3.028,  1.096,  0.902, 0.431),
            ( 1.829,  2.108,  0.970, 0.239)
        ]

    def _image_cb(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.nav.get_logger().error(f'CvBridge error: {e}')

    def make_pose(self, x, y, z, w) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.nav.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.z = z
        p.pose.orientation.w = w
        return p

    def lookup_tf(self):
        try:
            return self.tf_buffer.lookup_transform(
                'map', 'base_link',
                # 最新時刻を指定
                self.nav.get_clock().now(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except (LookupException, ConnectivityException, ExtrapolationException, TransformException):
            return None

    def is_obstacle_camera(self, wx, wy) -> bool:
        # 画像 or TF が取れなかったら「障害物あり」としてスキップ
        if self.latest_image is None:
            self.nav.get_logger().warn('No image → skip')
            return True
        tf = self.lookup_tf()
        if tf is None:
            self.nav.get_logger().warn('TF failed → skip')
            return True

        # 目標方向（ロボット座標系）
        dx = wx - tf.transform.translation.x
        dy = wy - tf.transform.translation.y
        q = tf.transform.rotation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny, cosy)
        rx =  dx*math.cos(yaw) + dy*math.sin(yaw)
        ry = -dx*math.sin(yaw) + dy*math.cos(yaw)
        ang = math.atan2(ry, rx)

        # カメラ水平視野外なら検出不要
        FOV = math.radians(60)
        if abs(ang) > FOV/2:
            return False

        # ROI 抽出
        img = self.latest_image
        h, w, _ = img.shape
        center = int((ang + FOV/2) / FOV * w)
        x1, x2 = max(0, center-100), min(w, center+100)
        roi = img[int(h*0.3):int(h*0.7), x1:x2]

        # エッジ → 輪郭 → 四角形カウント
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 50, 150)
        cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        rects = 0
        for c in cnts:
            area = cv2.contourArea(c)
            if area < 2000: continue
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02*peri, True)
            if len(approx) == 4:
                x,y,ww,hh = cv2.boundingRect(approx)
                ar = ww/float(hh)
                if 0.5 < ar < 2.0:
                    rects += 1

        # 四角形が 2 個以上あれば「積み重ねあり」と判断
        return rects >= 2

    def run(self):
        for x,y,z,w in self.route:
            if self.is_obstacle_camera(x, y):
                self.nav.get_logger().info(f'SKIP waypoint ({x:.2f},{y:.2f})')
                continue

            pose = self.make_pose(x,y,z,w)
            self.nav.goToPose(pose)
            while not self.nav.isTaskComplete():
                time.sleep(0.2)

            res = self.nav.getResult()
            if res == TaskResult.SUCCEEDED:
                self.nav.get_logger().info(f'Reached ({x:.2f},{y:.2f})')
            else:
                self.nav.get_logger().warn(f'Failed ({x:.2f},{y:.2f}): {res}')

        # 終了処理
        self.nav.lifecycleShutdown()
        self.nav.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    CVObstacleNavigator().run()
