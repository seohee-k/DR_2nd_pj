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

        # 내비게이터 객체 생성 (순찰 및 목표 이동 제어)
        self.navigator = TurtleBot4Navigator()
        # 이미지 변환용 브리지
        self.bridge = CvBridge()
        # 카메라 내부 파라미터 행렬
        self.K = None
        # 최신 깊이 이미지
        self.depth_image = None
        # 카메라 프레임명
        self.camera_frame = None

        # TF 변환을 위한 버퍼 및 리스너
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 순찰 경로(웨이포인트) 정의
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
        self.position_index = 0  # 현재 순찰 위치 인덱스

        # 순찰 상태 변수
        self.patrol_active = True         # 순찰 활성화 여부
        self.goal_in_progress = False     # 목표 이동 중 여부
        self.initialized = False          # 초기화 완료 여부

        # 차량(침입자) 프로토콜 상태 변수
        self.tracking_car = False         # 차량 추적 중 여부
        self.tracked_car_pose = None      # 추적할 차량 위치
        self.waypoint_to_resume = None    # 프로토콜 종료 후 복귀할 웨이포인트 인덱스
        self.car_protocol_phase = 0       # 차량 프로토콜 단계

        # 화재 프로토콜 상태 변수
        self.tracking_fire = False        # 화재 추적 중 여부
        self.tracked_fire_pose = None     # 추적할 화재 위치
        self.fire_protocol_phase = 0      # 화재 프로토콜 단계

        # 토픽 구독 설정
        self.create_subscription(BoundingBoxes, '/yoloinference', self.yolo_callback, 1)
        self.create_subscription(CameraInfo, '/robot7/oakd/stereo/camera_info', self.camera_info_callback, 1)
        self.create_subscription(CompressedImage, '/robot7/oakd/stereo/image_raw/compressedDepth', self.depth_callback, 1)

        # 순찰 타이머 콜백 (0.5초마다 동작)
        self.create_timer(0.5, self.patrol_timer_cb)
        self.get_logger().info('[BATTERY] 현재 잔량: 90.0% - 순찰 시작!')

    def camera_info_callback(self, msg):
        # 카메라 내부 파라미터(K) 및 프레임명 저장
        self.K = np.array(msg.k).reshape(3, 3)
        self.camera_frame = msg.header.frame_id

    def depth_callback(self, msg):
        # 압축된 깊이 이미지를 디코딩하여 저장
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
        # 최초 1회 초기화 (도킹 → 초기 위치 설정 → undock)
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

        # 차량(침입자) 프로토콜 단계별 처리
        if self.tracking_car:
            # 0단계: 차량 위치로 이동 시작
            if self.car_protocol_phase == 0 and self.tracked_car_pose is not None:
                self.get_logger().info(f"[NAV] 차량 추적, 좌표=({self.tracked_car_pose.pose.position.x:.2f},{self.tracked_car_pose.pose.position.y:.2f})")
                self.navigator.startToPose(self.tracked_car_pose)
                self.car_protocol_phase = 1
            # 1단계: 도착 대기
            elif self.car_protocol_phase == 1:
                if self.navigator.isTaskComplete():
                    self.get_logger().info("[PROTOCOL] 차량 도착, 1차 경고 진행")
                    self.car_protocol_phase = 2
                    self.protocol_timer = time.time()
            # 2단계: 1차 경고 및 반응 대기
            elif self.car_protocol_phase == 2:
                if time.time() - self.protocol_timer > 1.0:
                    self.get_logger().warn("[ALERT] 1차 경고! (음성+LED)")
                    if random.choice([True, False]):
                        self.get_logger().info("[INTRUDER] 차량 1차 경고 반응: 응함 → 정상 복귀")
                        self.reset_after_car_protocol()
                    else:
                        self.car_protocol_phase = 3
                        self.protocol_timer = time.time()
            # 3단계: 2차 경고 및 반응 대기
            elif self.car_protocol_phase == 3:
                if time.time() - self.protocol_timer > 1.0:
                    self.get_logger().warn("[ALERT] 2단계: 112신고, 테이저 발사!")
                    if random.choice([True, False]):
                        self.get_logger().info("[INTRUDER] 차량 2차 경고 반응: 응함 → 복귀")
                        self.reset_after_car_protocol()
                    else:
                        self.car_protocol_phase = 4
                        self.protocol_timer = time.time()
            # 4단계: 최종 경고 및 프로토콜 종료
            elif self.car_protocol_phase == 4:
                if time.time() - self.protocol_timer > 1.0:
                    self.get_logger().warn("[ALERT] 3단계: Rope gun(포박줄) 발사!")
                    self.get_logger().info("[INTRUDER] 경찰 도착, 순찰 복귀 (로그만)")
                    self.reset_after_car_protocol()
            return

        # 화재 프로토콜 단계별 처리
        if self.tracking_fire:
            # 0단계: 화재 위치로 이동 시작
            if self.fire_protocol_phase == 0 and self.tracked_fire_pose is not None:
                self.get_logger().info(f"[NAV] 화재 위치 접근, 좌표=({self.tracked_fire_pose.pose.position.x:.2f},{self.tracked_fire_pose.pose.position.y:.2f})")
                self.navigator.startToPose(self.tracked_fire_pose)
                self.fire_protocol_phase = 1
            # 1단계: 도착 후 소화기 발사 및 프로토콜 종료
            elif self.fire_protocol_phase == 1:
                if self.navigator.isTaskComplete():
                    self.get_logger().warn("[ALERT] 화재 감지! 투척형 소화기 발사!!")
                    # (필요 시 실제 소화기 발사 토픽 publish 삽입)
                    self.get_logger().info("[FIRE] 화재 진압 시도, 순찰 복귀 (로그만)")
                    self.reset_after_fire_protocol()
            return

        # 순찰 주행: 목표 위치로 이동 시작
        if self.patrol_active and not self.goal_in_progress:
            self.get_logger().info(f'[PATROL] {self.position_index+1}/{len(self.goal_poses)}번 웨이포인트로 이동')
            pose = self.goal_poses[self.position_index]
            self.navigator.startToPose(pose)
            self.goal_in_progress = True

        # 순찰 주행: 목표 도착 시 다음 웨이포인트로 인덱스 갱신
        elif self.patrol_active and self.goal_in_progress:
            if self.navigator.isTaskComplete():
                self.position_index = (self.position_index + 1) % len(self.goal_poses)
                self.goal_in_progress = False

    def yolo_callback(self, msg: BoundingBoxes):
        # 차량/화재 프로토콜 중이면 무시 (한 번에 하나만 동작)
        if self.tracking_car or self.tracking_fire:
            return

        for box in msg.boxes:
            if box.class_name == 'car':
                # 차량 중심 픽셀 좌표 계산
                u = (box.x1 + box.x2) // 2
                v = (box.y1 + box.y2) // 2

                # 카메라 파라미터 또는 깊이 이미지 미수신 시 경고
                if self.K is None or self.depth_image is None:
                    self.get_logger().warn("CameraInfo 또는 Depth 미수신!")
                    return

                # 깊이값(m)으로 변환 (픽셀 단위 깊이가 mm 단위라고 가정)
                z = float(self.depth_image[v, u]) / 1000.0
                if z == 0.0:
                    self.get_logger().warn("Depth 값 0")
                    return

                # 픽셀 좌표 → 카메라 좌표계 3D 변환
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

                # map 프레임 좌표로 PoseStamped 생성
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = pt_map.point.x
                pose.pose.position.y = pt_map.point.y
                pose.pose.orientation.w = 1.0

                # 차량 프로토콜 시작
                self.tracking_car = True
                self.patrol_active = False
                self.goal_in_progress = False
                self.tracked_car_pose = pose
                self.waypoint_to_resume = self.position_index
                self.car_protocol_phase = 0
                self.navigator.cancelTask()
                break

            elif box.class_name == 'fire':
                # 화재 중심 픽셀 좌표 계산
                u = (box.x1 + box.x2) // 2
                v = (box.y1 + box.y2) // 2

                if self.K is None or self.depth_image is None:
                    self.get_logger().warn("CameraInfo 또는 Depth 미수신!")
                    return

                # 깊이값(m)으로 변환, 2m 이내로 제한
                z = float(self.depth_image[v, u]) / 1000.0
                approach_dist = 2.0  # 2미터까지 접근
                if z > approach_dist:
                    z = approach_dist

                # 픽셀 좌표 → 카메라 좌표계 3D 변환
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

                # map 프레임 좌표로 PoseStamped 생성
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = pt_map.point.x
                pose.pose.position.y = pt_map.point.y
                pose.pose.orientation.w = 1.0

                # 화재 프로토콜 시작
                self.tracking_fire = True
                self.patrol_active = False
                self.goal_in_progress = False
                self.tracked_fire_pose = pose
                self.fire_protocol_phase = 0
                self.navigator.cancelTask()
                break

    def reset_after_car_protocol(self):
        # 차량 프로토콜 종료 후 상태 초기화 및 순찰 복귀
        self.tracking_car = False
        self.patrol_active = True
        self.goal_in_progress = False
        self.position_index = self.waypoint_to_resume if self.waypoint_to_resume is not None else 0
        self.tracked_car_pose = None
        self.car_protocol_phase = 0

    def reset_after_fire_protocol(self):
        # 화재 프로토콜 종료 후 상태 초기화 및 순찰 복귀
        self.tracking_fire = False
        self.patrol_active = True
        self.goal_in_progress = False
        self.tracked_fire_pose = None
        self.fire_protocol_phase = 0

def main(args=None):
    # ROS2 노드 실행 진입점
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

