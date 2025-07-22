import rclpy
from rclpy.node import Node
from yoloinference.msg import BoundingBoxes
from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped, PoseStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
import random
import time
import cv2

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        self.navigator = TurtleBot4Navigator()
        self.bridge = CvBridge()
        self.K = None
        self.depth_image = None
        self.camera_frame = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 순찰 경로
        self.goal_poses = [
            self.navigator.getPoseStamped([-0.2326, 0.2989], TurtleBot4Directions.EAST),
            self.navigator.getPoseStamped([-1.9897, -2.7512], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-2.3337, 2.4464], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-0.4372, 1.5495], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-0.8010, 2.5443], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-2.4568, 2.4879], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-2.1945, 0.0094], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-0.4167, 0.2248], TurtleBot4Directions.NORTH),
        ]
        self.position_index = 0

        # 상태
        self.patrol_active = True
        self.tracking_car = False
        self.goal_in_progress = False
        self.tracked_car_pose = None
        self.waypoint_to_resume = None
        self.car_protocol_phase = 0

        self.create_subscription(BoundingBoxes, '/yoloinference', self.yolo_callback, 10)
        self.create_subscription(CameraInfo, '/robot7/oakd/rgb/camera_info', self.camera_info_callback, 10)
        self.create_subscription(CompressedImage, '/robot7/oakd/rgb/image_raw/compressedDepth', self.depth_callback, 10)

        self.create_timer(0.5, self.patrol_timer_cb)
        self.get_logger().info('[BATTERY] 현재 잔량: 90.0% - 순찰 시작!')
        self.initialized = False

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        self.camera_frame = msg.header.frame_id

    def depth_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        if np_arr.size == 0:
            self.get_logger().error("Received empty depth buffer! (msg.data is empty)")
            return
        try:
            depth = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            if depth is not None and depth.size > 0:
                self.depth_image = depth
            else:
                self.get_logger().warn("Decoded depth image is None or empty!")
        except Exception as e:
            self.get_logger().error(f"Depth decode failed: {e}")


    def patrol_timer_cb(self):
        # 최초 초기화
        if not self.initialized:
            if not self.navigator.getDockedStatus():
                self.get_logger().info('[INIT] Docking before initializing pose')
                self.navigator.dock()
            initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
            self.navigator.setInitialPose(initial_pose)
            self.navigator.waitUntilNav2Active()
            self.navigator.undock()
            self.initialized = True
            return

        if self.tracking_car:
            if self.car_protocol_phase == 0 and self.tracked_car_pose is not None:
                self.get_logger().info(f"[NAV] 차량 추적, 좌표=({self.tracked_car_pose.pose.position.x:.2f},{self.tracked_car_pose.pose.position.y:.2f})")
                self.navigator.startToPose(self.tracked_car_pose)
                self.car_protocol_phase = 1  # 추적 이동 중
            elif self.car_protocol_phase == 1:
                if self.navigator.isTaskComplete():
                    self.get_logger().info("[PROTOCOL] 차량 도착, 1차 경고 진행")
                    self.car_protocol_phase = 2
                    self.protocol_timer = time.time()
            elif self.car_protocol_phase == 2:
                if time.time() - self.protocol_timer > 1.0:
                    self.get_logger().warn("[ALERT] 1차 경고! (음성+LED)")
                    if random.choice([True, False]):
                        self.get_logger().info("[INTRUDER] 차량 1차 경고 반응: 응함 → 정상 복귀")
                        self.reset_after_car_protocol()
                    else:
                        self.car_protocol_phase = 3
                        self.protocol_timer = time.time()
            elif self.car_protocol_phase == 3:
                if time.time() - self.protocol_timer > 1.0:
                    self.get_logger().warn("[ALERT] 2단계: 112신고, 테이저 발사!")
                    if random.choice([True, False]):
                        self.get_logger().info("[INTRUDER] 차량 2차 경고 반응: 응함 → 복귀")
                        self.reset_after_car_protocol()
                    else:
                        self.car_protocol_phase = 4
                        self.protocol_timer = time.time()
            elif self.car_protocol_phase == 4:
                if time.time() - self.protocol_timer > 1.0:
                    self.get_logger().warn("[ALERT] 3단계: Rope gun(포박줄) 발사!")
                    self.get_logger().info("[INTRUDER] 경찰 도착, 순찰 복귀 (로그만)")
                    self.reset_after_car_protocol()
            return

        # 순찰 주행
        if self.patrol_active and not self.goal_in_progress:
            self.get_logger().info(f'[PATROL] {self.position_index+1}/{len(self.goal_poses)}번 웨이포인트로 이동')
            pose = self.goal_poses[self.position_index]
            self.navigator.startToPose(pose)
            self.goal_in_progress = True

        elif self.patrol_active and self.goal_in_progress:
            if self.navigator.isTaskComplete():
                self.position_index = (self.position_index + 1) % len(self.goal_poses)
                self.goal_in_progress = False

    def yolo_callback(self, msg: BoundingBoxes):
        if self.tracking_car:
            return
        for box in msg.boxes:
            if box.class_name == 'car':
                # 중점 픽셀 좌표
                u = (box.x1 + box.x2) // 2
                v = (box.y1 + box.y2) // 2

                if self.K is None or self.depth_image is None:
                    self.get_logger().warn("CameraInfo 또는 Depth 미수신!")
                    return

                # 깊이값(m)으로 변환 (픽셀 단위 깊이가 mm 단위로 온다고 가정)
                z = float(self.depth_image[v, u]) / 1000.0
                if z == 0.0:
                    self.get_logger().warn("Depth 값 0")
                    return

                fx, fy = self.K[0,0], self.K[1,1]
                cx, cy = self.K[0,2], self.K[1,2]
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                # 카메라 프레임 → map 프레임으로 변환
                pt = PointStamped()
                pt.header.frame_id = self.camera_frame
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.point.x, pt.point.y, pt.point.z = x, y, z
                try:
                    pt_map = self.tf_buffer.transform(pt, 'map')
                except Exception as e:
                    self.get_logger().warn(f"TF 변환 실패: {e}")
                    return

                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = pt_map.point.x
                pose.pose.position.y = pt_map.point.y
                pose.pose.orientation.w = 1.0

                self.tracking_car = True
                self.patrol_active = False
                self.goal_in_progress = False
                self.tracked_car_pose = pose
                self.waypoint_to_resume = self.position_index
                self.car_protocol_phase = 0
                self.navigator.cancelTask()
                break

    def reset_after_car_protocol(self):
        self.tracking_car = False
        self.patrol_active = True
        self.goal_in_progress = False
        self.position_index = self.waypoint_to_resume if self.waypoint_to_resume is not None else 0
        self.tracked_car_pose = None
        self.car_protocol_phase = 0

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
