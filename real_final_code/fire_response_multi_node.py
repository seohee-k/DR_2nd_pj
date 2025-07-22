#!/usr/bin/env python3
# 이 스크립트는 Python 3로 실행되도록 지정합니다.

import rclpy  # ROS2의 Python 라이브러리를 임포트하여 ROS2 노드와 통신 기능을 제공합니다.
from rclpy.node import Node  # ROS2 노드 클래스를 임포트합니다.
from rclpy.executors import MultiThreadedExecutor  # 여러 노드를 병렬로 실행하기 위한 멀티스레드 실행자를 임포트합니다.
from yoloinference.msg import BoundingBoxes  # YOLO 객체 탐지 결과를 수신하기 위한 메시지 타입을 임포트합니다.
from sensor_msgs.msg import CameraInfo, CompressedImage  # 카메라 정보와 압축된 이미지를 처리하기 위한 메시지 타입을 임포트합니다.
from geometry_msgs.msg import PointStamped, PoseStamped  # 3D 포인트와 포즈 데이터를 처리하기 위한 메시지 타입을 임포트합니다.
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator  # TurtleBot4의 내비게이션 기능과 방향 상수를 임포트합니다.
import numpy as np  # 수치 계산 및 배열 처리를 위해 numpy 라이브러리를 임포트합니다.
import tf2_ros  # 좌표 변환을 처리하기 위한 TF2 라이브러리를 임포트합니다.
import math  # 수학 계산(예: 거리 계산)을 위해 math 모듈을 임포트합니다.
import random  # 무작위 값 생성(침입자 탐지 확률)을 위해 random 모듈을 임포트합니다.
import cv2  # OpenCV 라이브러리를 임포트하여 이미지 디코딩(깊이 이미지 처리)에 사용합니다.
import time  # 시간 관련 기능(로그 출력 시 현재 시간 표시)을 위해 time 모듈을 임포트합니다.

# 순찰 경로를 정의하는 웨이포인트 리스트입니다. 각 요소는 (x, y) 좌표 쌍으로, map 프레임에서의 위치를 나타냅니다.
WAYPOINTS = [
    (-2.2381389141082764, -0.08192165195941925),  # 웨이포인트 1
    (-2.099907636642456, 2.197780132293701),      # 웨이포인트 2
    (-0.39694878458976746, 2.4117934703826904),   # 웨이포인트 3
    (-0.3820028007030487, 1.3472325801849365),    # 웨이포인트 4
    (-0.5478897094726562, 2.3528730869293213),    # 웨이포인트 5
    (-2.0914320945739746, 2.2400667667388916),    # 웨이포인트 6
    (-2.116553544998169, 0.38230445981025696),    # 웨이포인트 7
    (-0.4361707270145416, 0.121768057346344)      # 웨이포인트 8
]

# FireResponseNode 클래스는 ROS2 노드를 상속받아 TurtleBot4의 순찰, 화재 탐지 및 대응, 침입자 순찰 기능을 구현합니다.
class FireResponseNode(Node):
    def __init__(self, robot_id):
        # 부모 클래스(Node)의 초기화 메서드를 호출하며, 노드 이름을 'fire_response_node_{robot_id}'로 설정합니다.
        super().__init__(f'fire_response_node_{robot_id}')

        # 로봇 ID를 저장합니다 (예: '1' 또는 '2', TurtleBot4-1 또는 TurtleBot4-2를 구분).
        self.robot_id = robot_id
        # TurtleBot4의 내비게이션 기능을 제공하는 TurtleBot4Navigator 객체를 생성합니다.
        self.navigator = TurtleBot4Navigator()
        # 카메라 내재 파라미터 행렬(K), 깊이 이미지, 카메라 좌표 프레임, 화재 깊이 값을 초기화합니다.
        self.K = None
        self.depth_image = None
        self.camera_frame = None
        self.fire_depth = None

        # 좌표 변환을 처리하기 위한 TF2 버퍼와 리스너를 초기화합니다.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # WAYPOINTS를 기반으로 PoseStamped 객체 리스트를 생성하여 순찰 경로를 설정합니다.
        # 모든 웨이포인트는 북쪽 방향(TurtleBot4Directions.NORTH)으로 설정됩니다.
        self.goal_poses = [self.navigator.getPoseStamped([x, y], TurtleBot4Directions.NORTH) for x, y in WAYPOINTS]
        # 현재 순찰 중인 웨이포인트의 인덱스를 초기화합니다.
        self.position_index = 0

        # 순찰 상태, 화재 탐지 상태, 목표 이동 진행 상태, 추적 중인 화재 위치, 화재 대응 단계를 초기화합니다.
        self.patrol_active = True  # 순찰 활성화 여부
        self.fire_detected = False  # 화재 탐지 여부
        self.goal_in_progress = False  # 목표 이동 진행 중 여부
        self.tracked_fire_pose = None  # 추적 중인 화재 위치 (PoseStamped 객체)
        self.fire_response_phase = 0  # 화재 대응 단계 (0: 초기, 1: 이동 완료, 2: 진압 완료, 3: 침입자 순찰 완료)

        # YOLO 객체 탐지 결과를 수신하기 위한 구독자를 생성합니다.
        self.create_subscription(BoundingBoxes, f'/turtlebot{robot_id}/yoloinference', self.yolo_callback, 10)
        # 카메라 정보를 수신하기 위한 구독자를 생성합니다.
        self.create_subscription(CameraInfo, f'/turtlebot{robot_id}/oakd/rgb/camera_info/compressed', self.camera_info_callback, 10)
        # 압축된 깊이 이미지를 수신하기 위한 구독자를 생성합니다.
        self.create_subscription(CompressedImage, f'/turtlebot{robot_id}/oakd/rgb/image_raw/compressedDepth', self.depth_callback, 10)

        # 0.5초 간격으로 patrol_timer_cb 메서드를 호출하는 타이머를 생성합니다.
        self.create_timer(0.5, self.patrol_timer_cb)
        # 초기 로그 메시지를 출력하여 배터리 상태와 순찰 시작을 알립니다.
        self.get_logger().info(f'[BATTERY] 현재 잔량: 90.0% - 순찰 시작! (TurtleBot4-{robot_id})')

    # 카메라 정보 콜백 메서드로, 카메라 캘리브레이션 데이터를 처리합니다.
    def camera_info_callback(self, msg):
        # CameraInfo 메시지에서 내재 파라미터(k)를 3x3 행렬로 변환하여 저장합니다.
        self.K = np.array(msg.k).reshape(3, 3)
        # 카메라 좌표 프레임 ID를 저장합니다.
        self.camera_frame = msg.header.frame_id

    # 깊이 이미지 콜백 메서드로, 압축된 깊이 이미지를 디코딩하여 저장합니다.
    def depth_callback(self, msg):
        # CompressedImage 메시지의 데이터를 numpy 배열로 변환합니다.
        np_arr = np.frombuffer(msg.data, np.uint8)
        # 데이터가 비어 있으면 오류 로그를 출력하고 종료합니다.
        if np_arr.size == 0:
            self.get_logger().error("Received empty depth buffer!")
            return
        try:
            # 압축된 이미지를 디코딩하여 깊이 이미지로 변환합니다.
            depth = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            # 디코딩된 이미지가 유효하면 저장합니다.
            if depth is not None and depth.size > 0:
                self.depth_image = depth
            else:
                # 디코딩된 이미지가 비어 있으면 경고 로그를 출력합니다.
                self.get_logger().warn("Decoded depth image is empty!")
        except Exception as e:
            # 디코딩 실패 시 오류 로그를 출력합니다.
            self.get_logger().error(f"Depth decode failed: {e}")

    # 화재 위치까지 안전 거리(1m)로 접근하는 메서드입니다.
    def get_close_to_fire(self):
        # 추적 중인 화재 위치 또는 깊이 데이터가 없으면 경고 로그를 출력하고 종료합니다.
        if not self.tracked_fire_pose or self.fire_depth is None:
            self.get_logger().warn("Fire position or depth not available!")
            return

        # 화재 위치의 x, y 좌표를 추출합니다.
        fire_x = self.tracked_fire_pose.pose.position.x
        fire_y = self.tracked_fire_pose.pose.position.y
        # 깊이 값을 미터 단위로 변환합니다 (mm → m).
        distance_to_fire = self.fire_depth / 1000.0

        # 안전 거리를 1m로 설정합니다.
        safe_distance = 1.0
        # 현재 화재와의 거리가 안전 거리 이내이면 로그를 출력하고 종료합니다.
        if distance_to_fire <= safe_distance:
            self.get_logger().info(f"[GET_CLOSE_TO_FIRE] Already at safe distance ({distance_to_fire:.2f}m) from fire")
            return

        # 현재 로봇의 위치를 가져옵니다.
        current_pose = self.navigator.getPose()
        # 위치를 가져오지 못하면 오류 로그를 출력하고 종료합니다.
        if current_pose is None:
            self.get_logger().error("Failed to get current pose.")
            return
        # 현재 위치의 x, y 좌표를 추출합니다.
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y

        # 화재와 현재 위치 간의 x, y 방향 거리 차이를 계산합니다.
        dx = fire_x - current_x
        dy = fire_y - current_y
        # 화재와의 총 거리를 계산합니다.
        total_distance = math.sqrt(dx**2 + dy**2)
        # 거리가 0이면(이미 같은 위치) 종료합니다.
        if total_distance == 0:
            return

        # 접근해야 할 거리(총 거리에서 안전 거리를 뺀 값)를 계산합니다.
        approach_distance = total_distance - safe_distance
        # 접근 거리가 0 이하이면 종료합니다.
        if approach_distance <= 0:
            return

        # 접근 비율을 계산합니다.
        ratio = approach_distance / total_distance
        # 새로운 목표 위치를 계산합니다 (현재 위치 + 방향 벡터 * 비율).
        new_x = current_x + dx * ratio
        new_y = current_y + dy * ratio

        # 목표 위치로 이동 시작 로그를 출력합니다.
        self.get_logger().info(f"[GET_CLOSE_TO_FIRE] Approaching fire at ({new_x:.2f}, {new_y:.2f})")
        # 새로운 목표 위치를 PoseStamped 객체로 생성합니다.
        target_pose = self.navigator.getPoseStamped([new_x, new_y], TurtleBot4Directions.NORTH)
        # 목표 위치로 이동을 시작합니다.
        self.navigator.startToPose(target_pose)
        # 이동이 완료될 때까지 대기합니다.
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
        # 이동 완료 로그를 출력합니다.
        self.get_logger().info(f"[GET_CLOSE_TO_FIRE] Approached fire, distance: {safe_distance:.2f}m")

    # 침입자를 탐지하고 대응하는 순찰 메서드입니다.
    def patrol_intruder(self):
        # 침입자 순찰 시작 로그를 출력합니다.
        self.get_logger().info('Starting intruder patrol...')
        # 처음 8개의 웨이포인트를 순회합니다 (goal_poses는 총 8개).
        for pose in self.goal_poses[:8]:
            # 현재 웨이포인트로 이동을 시작합니다.
            self.navigator.startToPose(pose)
            # 이동이 완료될 때까지 대기합니다.
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)
            # 95% 확률로 침입자를 탐지합니다.
            if random.random() < 0.95:
                # 침입자 위치를 현재 웨이포인트 위치로 설정합니다.
                intruder_position = {'x': pose.pose.position.x, 'y': pose.pose.position.y}
                # 침입자 탐지 로그를 출력합니다.
                self.get_logger().info(f'[INTRUDER] Detected at {intruder_position["x"]:.2f}, {intruder_position["y"]:.2f}')
                # 1차 대응: LED 점등 및 경고음 출력 로그.
                self.get_logger().info("[INTRUDER] 1차 대응: LED 점등 및 경고음 출력")
                # 경고 메시지 출력 로그.
                self.get_logger().info("[INTRUDER] 경고: 즉시 나가십시오!")
                # 112 신고 완료 로그.
                self.get_logger().info("[INTRUDER] 112에 침입자 신고 완료")
                # 2차 대응: 테이저건 발사 로그.
                self.get_logger().info("[INTRUDER] 2차 대응: 테이저건 발사")
                # 3차 대응: 포박줄 발사 로그.
                self.get_logger().info("[INTRUDER] 3차 대응: 포박줄 발사")
                # 침입자를 탐지했으므로 순찰을 중단합니다.
                break
        # 순찰 완료 로그를 출력합니다.
        self.get_logger().info('Patrol completed, resuming normal patrol.')

    # 0.5초 간격으로 호출되어 순찰 또는 화재 대응을 관리하는 타이머 콜백 메서드입니다.
    def patrol_timer_cb(self):
        # 화재가 탐지된 경우 화재 대응 로직을 실행합니다.
        if self.fire_detected:
            # 초기 단계(0)에서 화재 위치가 유효하면 화재로 접근합니다.
            if self.fire_response_phase == 0 and self.tracked_fire_pose is not None:
                # 안전 거리까지 접근합니다.
                self.get_close_to_fire()
                # 화재 위치로 이동 시작 로그를 출력합니다.
                self.get_logger().info(f"[NAV] 화재 추적, 좌표=({self.tracked_fire_pose.pose.position.x:.2f},{self.tracked_fire_pose.pose.position.y:.2f})")
                # 화재 위치로 이동을 시작합니다.
                self.navigator.startToPose(self.tracked_fire_pose)
                # 다음 단계(1)로 전환합니다.
                self.fire_response_phase = 1
            # 이동 완료 단계(1)에서 이동이 완료되었는지 확인합니다.
            elif self.fire_response_phase == 1:
                if self.navigator.isTaskComplete():
                    # 화재 도착 및 진압 시작 로그를 출력합니다.
                    self.get_logger().info("[PROTOCOL] 화재 도착, 진압 시작")
                    # 화재 진압을 실행합니다.
                    self.extinguish_fire()
                    # 다음 단계(2)로 전환합니다.
                    self.fire_response_phase = 2
            # 진압 완료 단계(2)에서 침입자 순찰을 시작합니다.
            elif self.fire_response_phase == 2:
                # 침입자 순찰 시작 로그를 출력합니다.
                self.get_logger().info("[PROTOCOL] 화재 진압 완료, 침입자 순찰 시작")
                # 침입자 순찰을 실행합니다.
                self.patrol_intruder()
                # 다음 단계(3)로 전환합니다.
                self.fire_response_phase = 3
            return

        # 화재가 탐지되지 않은 경우 일반 순찰 로직을 실행합니다.
        # 순찰이 활성화되고 목표 이동이 진행 중이 아니면 다음 웨이포인트로 이동합니다.
        if self.patrol_active and not self.goal_in_progress:
            # 현재 이동할 웨이포인트 로그를 출력합니다.
            self.get_logger().info(f'[PATROL] {self.position_index+1}/{len(self.goal_poses)}번 웨이포인트로 이동')
            # 현재 웨이포인트를 가져옵니다.
            pose = self.goal_poses[self.position_index]
            # 해당 웨이포인트로 이동을 시작합니다.
            self.navigator.startToPose(pose)
            # 목표 이동 진행 상태를 True로 설정합니다.
            self.goal_in_progress = True
        # 순찰이 활성화되고 목표 이동이 진행 중이면 이동 완료 여부를 확인합니다.
        elif self.patrol_active and self.goal_in_progress:
            if self.navigator.isTaskComplete():
                # 다음 웨이포인트로 인덱스를 업데이트합니다 (순환 구조).
                self.position_index = (self.position_index + 1) % len(self.goal_poses)
                # 목표 이동 진행 상태를 False로 설정합니다.
                self.goal_in_progress = False

    # YOLO 객체 탐지 결과를 처리하는 콜백 메서드입니다.
    def yolo_callback(self, msg: BoundingBoxes):
        # 이미 화재가 탐지된 경우 추가 탐지를 무시합니다.
        if self.fire_detected:
            return
        # 모든 바운딩 박스를 순회하며 'fire' 클래스를 탐지합니다.
        for box in msg.boxes:
            if box.class_name == 'fire':
                # 바운딩 박스의 중심 좌표를 계산합니다.
                u = (box.x1 + box.x2) // 2
                v = (box.y1 + box.y2) // 2

                # 카메라 정보 또는 깊이 이미지가 없으면 경고 로그를 출력하고 종료합니다.
                if self.K is None or self.depth_image is None:
                    self.get_logger().warn("CameraInfo 또는 Depth 미수신!")
                    return

                # 중심 좌표에서의 깊이 값을 가져옵니다.
                z = float(self.depth_image[v, u])
                # 깊이 값을 저장합니다.
                self.fire_depth = z
                # 깊이 값이 0이면 경고 로그를 출력하고 종료합니다.
                if z == 0.0:
                    self.get_logger().warn("Depth 값 0")
                    return

                # 카메라 내재 파라미터에서 초점 거리(fx, fy)와 중심점(cx, cy)을 추출합니다.
                fx, fy = self.K[0,0], self.K[1,1]
                cx, cy = self.K[0,2], self.K[1,2]
                # 카메라 좌표계에서의 3D 위치(x, y)를 계산합니다.
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                # 3D 포인트를 PointStamped 객체로 생성합니다.
                pt = PointStamped()
                pt.header.frame_id = self.camera_frame
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.point.x, pt.point.y, pt.point.z = x, y, z / 1000.0  # z를 미터 단위로 변환
                try:
                    # 카메라 프레임에서 map 프레임으로 좌표를 변환합니다.
                    pt_map = self.tf_buffer.transform(pt, 'map')
                except Exception as e:
                    # 좌표 변환 실패 시 경고 로그를 출력하고 종료합니다.
                    self.get_logger().warn(f"TF 변환 실패: {e}")
                    return

                # 변환된 좌표를 PoseStamped 객체로 생성합니다.
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = pt_map.point.x
                pose.pose.position.y = pt_map.point.y
                pose.pose.orientation.w = 1.0  # 기본 방향(쿼터니언 w=1)

                # 화재 탐지 상태를 업데이트합니다.
                self.fire_detected = True
                self.patrol_active = False
                self.goal_in_progress = False
                self.tracked_fire_pose = pose
                # 화재 탐지 로그를 출력합니다.
                self.get_logger().info(f"Fire detected at ({pt_map.point.x:.2f}, {pt_map.point.y:.2f})")
                # 현재 순찰 작업을 취소합니다.
                self.navigator.cancelTask()
                break

    # 화재 진압을 시뮬레이션하는 메서드입니다.
    def extinguish_fire(self):
        # 화재 진압 시작 로그를 출력합니다.
        self.get_logger().info('Attempting to extinguish fire...')
        # 화재 진압 완료 로그를 출력합니다.
        self.get_logger().info('Fire extinguished.')

# 메인 함수로, 두 TurtleBot4 노드를 실행합니다.
def main(args=None):
    # ROS2 환경을 초기화합니다.
    rclpy.init(args=args)
    try:
        # TurtleBot4-1과 TurtleBot4-2 노드를 생성합니다.
        node1 = FireResponseNode('1')
        node2 = FireResponseNode('2')

        # 멀티스레드 실행자를 생성하여 두 노드를 병렬로 실행합니다.
        executor = MultiThreadedExecutor()
        executor.add_node(node1)
        executor.add_node(node2)

        # 노드 시작 시간을 로그로 출력합니다 (현재 시간: 2025년 6월 3일 14:59:00 KST).
        print(f"Starting FireResponseNodes at {time.strftime('%H:%M:%S', time.localtime())} on June 03, 2025")
        # 노드를 실행합니다.
        executor.spin()

    except KeyboardInterrupt:
        # 사용자에 의한 종료 시 로그를 출력합니다.
        print("KeyboardInterrupt received, shutting down...")
    except Exception as e:
        # 예외 발생 시 오류 로그를 출력합니다.
        print(f"An error occurred: {e}")
    finally:
        # 노드와 ROS2 환경을 정리합니다.
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()
        # 종료 완료 로그를 출력합니다.
        print("FireResponseNodes shutdown complete.")

# 스크립트가 직접 실행될 때 main 함수를 호출합니다.
if __name__ == '__main__':
    main()