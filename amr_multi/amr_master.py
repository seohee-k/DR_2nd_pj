import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from yoloinference.msg import BoundingBoxes
from cv_bridge import CvBridge
from ultralytics import YOLO
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import numpy as np
import cv2
import tf2_ros
import math
import random

WAYPOINTS = [
    (-2.238, -0.081), (-2.099, 2.197), (-0.396, 2.411), (-0.382, 1.347),
    (-0.547, 2.352), (-2.091, 2.240), (-2.116, 0.382), (-0.436, 0.121)
]

class AMRMasterNode(Node):
    def __init__(self, robot_id='1'):
        super().__init__('amr_master_node', namespace=f'robot{robot_id}')
        # 상태: patrol, firestop, resque
        self.state = "patrol"
        self.i_am_fire_detector = False  # 내가 fire_flag를 보냈는지 여부

        # 순찰/직원 감지 관련 변수
        self.bridge = CvBridge()
        self.employee_stack = []
        self.current_waypoint = 0
        self.goal_running = False
        self.K = None
        self.rgb_image = None
        self.depth_image = None
        self.camera_frame = None

        # 화재 대응 관련 변수
        self.navigator = TurtleBot4Navigator()
        self.goal_poses = [self.navigator.getPoseStamped([x, y], TurtleBot4Directions.NORTH) for x, y in WAYPOINTS]
        self.position_index = 0
        self.fire_detected = False
        self.tracked_fire_pose = None
        self.fire_depth = None
        self.fire_response_phase = 0

        # 센서 토픽명
        self.depth_topic = f'/robot{robot_id}/oakd/stereo/image_raw'
        self.rgb_topic = f'/robot{robot_id}/oakd/rgb/image_raw/compressed'
        self.info_topic = f'/robot{robot_id}/oakd/rgb/camera_info'

        # 센서 데이터 구독
        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 1)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 1)
        self.create_subscription(CompressedImage, self.rgb_topic, self.rgb_callback, 1)

        # YOLO 결과 및 Depth, fire_flag 구독
        self.create_subscription(BoundingBoxes, f'/turtlebot{robot_id}/yoloinference', self.yolo_callback, 10)
        self.create_subscription(CompressedImage, f'/turtlebot{robot_id}/oakd/rgb/image_raw/compressedDepth', self.depth_callback, 10)
        self.create_subscription(Bool, 'fire_flag', self.fire_flag_cb, 10)
        self.emp_pub = self.create_publisher(Point, 'employee_stack', 10)
        self.fire_pub = self.create_publisher(Bool, 'fire_flag', 10)

        # 네비게이션 액션 클라이언트
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF 변환용 버퍼 및 리스너
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # YOLO 모델 로드
        self.model = YOLO('/home/djqsp2/rokey3_E4_ws/src/best.pt')

        # 직원 대피 관련 변수
        self.resque_executing = False

        # 최초 goal 전송용 타이머 (patrol)
        self.timer = self.create_timer(1.0, self.main_loop)
        self.first_goal_sent = False

        self.get_logger().info('AMR Master node initialized!')

    # -------------------- 공통 콜백 --------------------
    def fire_flag_cb(self, msg):
        """
        fire_flag 콜백: 화재 신호 수신 시 역할 분기
        """
        if msg.data:
            if not self.i_am_fire_detector:
                self.state = "resque"
                self.get_logger().warn('상대가 fire 감지 → 직원 대피 모드 전환')
            # fire_flag를 내가 이미 보냈으면 firestop 상태 유지

    def camera_info_callback(self, msg):
        """
        카메라 내부 파라미터 및 프레임 정보 저장
        """
        self.K = np.array(msg.k).reshape(3, 3)
        self.camera_frame = msg.header.frame_id

    def depth_callback(self, msg):
        """
        Depth 이미지 수신 및 저장
        """
        np_arr = np.frombuffer(msg.data, np.uint8)
        if np_arr.size == 0:
            self.get_logger().error("Received empty depth buffer!")
            return
        try:
            depth = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            if depth is not None and depth.size > 0:
                self.depth_image = depth
            else:
                self.get_logger().warn("Decoded depth image is empty!")
        except Exception as e:
            self.get_logger().error(f"Depth decode failed: {e}")

    def rgb_callback(self, msg):
        """
        RGB 이미지 수신, 디코딩 및 직원 탐지 시도
        """
        np_arr = np.frombuffer(msg.data, np.uint8)
        rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.rgb_image = rgb

    # -------------------- 메인 루프 --------------------
    def main_loop(self):
        """
        상태(state)에 따라 각 기능 수행
        """
        if self.state == "patrol":
            self.patrol_step()
        elif self.state == "firestop":
            self.firestop_step()
        elif self.state == "resque":
            self.resque_step()

    # -------------------- 순찰/직원 감지 --------------------
    def patrol_step(self):
        """
        순찰 및 직원 감지, 화재 감지 시 detect_fire() 호출
        """
        if not self.first_goal_sent and self.action_client.wait_for_server(timeout_sec=1.0):
            self.first_goal_sent = True
            self.send_next_waypoint()
        # 직원 탐지
        self.detect_employee()

    def send_next_waypoint(self):
        """
        다음 웨이포인트로 이동 goal 전송
        """
        if self.fire_detected:
            return
        x, y = WAYPOINTS[self.current_waypoint]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(f"Waypoint 이동: ({x:.2f}, {y:.2f})")
        self.goal_running = True
        send_goal_future = self.action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        네비게이션 goal 응답 콜백
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected!')
            self.goal_running = False
            return
        self.get_logger().info('Goal accepted!')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """
        네비게이션 goal 완료 콜백, 다음 웨이포인트로 이동 예약
        """
        self.get_logger().info('Goal result: 도착!')
        self.goal_running = False
        self.current_waypoint = (self.current_waypoint + 1) % len(WAYPOINTS)
        self._oneshot_timer = self.create_timer(0.5, self._timer_send_next_waypoint)

    def _timer_send_next_waypoint(self):
        """
        다음 웨이포인트 이동 트리거용 타이머 콜백
        """
        if not self.fire_detected:
            self.send_next_waypoint()
        self._oneshot_timer.cancel()

    def detect_employee(self):
        """
        YOLO로 직원 탐지, 좌표를 맵 프레임으로 변환 후 중복 없이 저장 및 publish
        """
        if self.rgb_image is None or self.depth_image is None or self.K is None:
            return

        results = self.model(self.rgb_image, verbose=False)[0]
        for det in results.boxes:
            cls = int(det.cls[0])
            label = self.model.names[cls]
            conf = float(det.conf[0])
            x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())

            if label.lower() == "employee" and conf > 0.8:
                u, v = int((x1 + x2)//2), int((y1 + y2)//2)
                z = float(self.depth_image[v, u]) / 1000.0
                if z == 0.0:
                    continue
                fx, fy = self.K[0, 0], self.K[1, 1]
                cx, cy = self.K[0, 2], self.K[1, 2]
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                pt = PointStamped()
                pt.header.frame_id = self.camera_frame
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.point.x = x
                pt.point.y = y
                pt.point.z = z

                try:
                    # 카메라 좌표를 맵 좌표로 변환
                    pt_map = self.tf_buffer.transform(pt, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
                    emp_point = Point()
                    emp_point.x = pt_map.point.x
                    emp_point.y = pt_map.point.y
                    emp_point.z = pt_map.point.z
                    # 중복 좌표는 저장하지 않음
                    if not any([np.isclose(emp_point.x, e.x, atol=0.1) and np.isclose(emp_point.y, e.y, atol=0.1) for e in self.employee_stack]):
                        self.employee_stack.append(emp_point)
                        self.get_logger().info(f'Employee 발견! 맵좌표=({emp_point.x:.2f}, {emp_point.y:.2f}, {emp_point.z:.2f})')
                        self.emp_pub.publish(emp_point)
                except Exception as e:
                    self.get_logger().warn(f"TF 변환 실패: {e}")

    def yolo_callback(self, msg: BoundingBoxes):
        """
        YOLO 결과 콜백: 화재(fire) 객체 감지 시 detect_fire() 호출
        """
        if self.state != "patrol":
            return
        for box in msg.boxes:
            if box.class_name == 'fire':
                self.detect_fire(box)
                break

    def detect_fire(self, box=None):
        """
        화재 감지 시 호출: fire_flag publish 및 상태 전환
        """
        if not self.i_am_fire_detector:
            self.i_am_fire_detector = True
            self.state = "firestop"
            self.fire_pub.publish(Bool(data=True))
            self.get_logger().warn('🔥 내가 fire 감지 → 화재 대응 모드 전환 및 fire_flag publish')
            # 직원 좌표도 publish
            for emp in self.employee_stack:
                self.emp_pub.publish(emp)
                self.get_logger().info(f'직원 좌표 publish: ({emp.x:.2f}, {emp.y:.2f})')
            # 화재 위치 계산
            if box is not None:
                u = (box.x1 + box.x2) // 2
                v = (box.y1 + box.y2) // 2
                if self.K is None or self.depth_image is None:
                    self.get_logger().warn("CameraInfo 또는 Depth 미수신!")
                    return
                z = float(self.depth_image[v, u])
                self.fire_depth = z
                if z == 0.0:
                    self.get_logger().warn("Depth 값 0")
                    return
                fx, fy = self.K[0,0], self.K[1,1]
                cx, cy = self.K[0,2], self.K[1,2]
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                pt = PointStamped()
                pt.header.frame_id = self.camera_frame
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.point.x, pt.point.y, pt.point.z = x, y, z / 1000.0
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
                self.tracked_fire_pose = pose

    # -------------------- 화재 대응 및 침입자 대응 --------------------
    def firestop_step(self):
        """
        화재 대응 단계별 상태 머신 (타이머 콜백)
        0: 화재 위치로 적정 거리까지 접근
        1: 소화기 투척 및 진압
        2: 침입자 대응 순찰
        """
        if self.tracked_fire_pose is None:
            return

        if self.fire_response_phase == 0:
            self.get_logger().info("[FIRE] 화재 위치로 적정 거리까지 접근")
            self.get_close_to_fire()
            self.fire_response_phase = 1
        elif self.fire_response_phase == 1:
            if self.navigator.isTaskComplete():
                self.get_logger().info("[FIRE] 소화기 투척 및 진압 시작")
                self.extinguish_fire()
                self.fire_response_phase = 2
        elif self.fire_response_phase == 2:
            self.get_logger().info("[INTRUDER] 침입자 대응 순찰 시작")
            self.patrol_intruder()
            self.fire_response_phase = 3

    def get_close_to_fire(self):
        """
        화재 위치로부터 안전거리(1m)까지 접근
        """
        if not self.tracked_fire_pose or self.fire_depth is None:
            self.get_logger().warn("Fire position or depth not available!")
            return

        fire_x = self.tracked_fire_pose.pose.position.x
        fire_y = self.tracked_fire_pose.pose.position.y
        distance_to_fire = self.fire_depth / 1000.0  # mm → m

        safe_distance = 1.0  # 안전거리(m)
        if distance_to_fire <= safe_distance:
            self.get_logger().info(f"[GET_CLOSE_TO_FIRE] Already at safe distance ({distance_to_fire:.2f}m) from fire")
            return

        # 현재 위치 가져오기
        current_pose = self.navigator.getPose()
        if current_pose is None:
            self.get_logger().error("Failed to get current pose.")
            return
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y

        # 화재 위치까지 벡터 계산
        dx = fire_x - current_x
        dy = fire_y - current_y
        total_distance = math.sqrt(dx**2 + dy**2)
        if total_distance == 0:
            return

        # 안전거리만큼 떨어진 위치 계산
        approach_distance = total_distance - safe_distance
        if approach_distance <= 0:
            return

        ratio = approach_distance / total_distance
        new_x = current_x + dx * ratio
        new_y = current_y + dy * ratio

        self.get_logger().info(f"[GET_CLOSE_TO_FIRE] Approaching fire at ({new_x:.2f}, {new_y:.2f})")
        target_pose = self.navigator.getPoseStamped([new_x, new_y], TurtleBot4Directions.NORTH)
        self.navigator.startToPose(target_pose)
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"[GET_CLOSE_TO_FIRE] Approached fire, distance: {safe_distance:.2f}m")

    def extinguish_fire(self):
        """
        투척형 소화기 발사 및 화재 진압(시뮬레이션)
        """
        self.get_logger().info('🔥 투척형 소화기 발사!')
        self.get_logger().info('Fire extinguished.')

    def patrol_intruder(self):
        """
        침입자 대응 순찰 및 시나리오별 대응(경고, 신고 등)
        - 각 웨이포인트에서 무조건 침입자를 감지하도록 수정
        """
        self.get_logger().info('Starting intruder patrol...')
        for pose in self.goal_poses[:3]:
            self.navigator.startToPose(pose)
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)
            # 무조건 침입자 감지
            intruder_position = {'x': pose.pose.position.x, 'y': pose.pose.position.y}
            self.get_logger().info(f'[INTRUDER] Detected at {intruder_position["x"]:.2f}, {intruder_position["y"]:.2f}')
            self.get_logger().info("[INTRUDER] 1차 대응: LED 점등 및 경고음 출력")
            self.get_logger().info("[INTRUDER] 경고: 즉시 나가십시오!")
            self.get_logger().info("[INTRUDER] 112에 침입자 신고 완료")
            self.get_logger().info("[INTRUDER] 2차 대응: 테이저건 발사")
            self.get_logger().info("[INTRUDER] 3차 대응: 포박줄 발사")
            break  # 한 번만 감지 후 순찰 종료
        self.get_logger().info('Patrol completed, resuming normal patrol.')

    # -------------------- 직원 대피 --------------------
    def resque_step(self):
        """
        직원 대피: fire_flag가 True이고, 이동 중이 아니며, 대피할 직원이 남아있으면
        employee_stack에서 좌표를 꺼내 해당 위치로 이동 goal 전송
        """
        if self.resque_executing or not self.employee_stack:
            return
        emp = self.employee_stack.pop(0)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = emp.x
        pose.pose.position.y = emp.y
        pose.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(f"직원 대피 좌표로 이동: ({emp.x:.2f}, {emp.y:.2f})")
        self.resque_executing = True
        send_goal_future = self.action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.resque_goal_response_callback)

    def resque_goal_response_callback(self, future):
        """
        직원 대피 goal 응답 콜백
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Resque Goal rejected!')
            self.resque_executing = False
            return
        self.get_logger().info('Resque Goal accepted!')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.resque_result_callback)

    def resque_result_callback(self, future):
        """
        직원 대피 goal 완료 콜백
        """
        self.get_logger().info('직원 대피 위치 도착')
        self.resque_executing = False

def main():
    rclpy.init()
    node = AMRMasterNode('1')  # robot_id는 환경에 맞게
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()