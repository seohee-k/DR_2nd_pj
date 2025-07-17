import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import cv2
import time

class YoloAlignedDepthNavGoal(Node):
    def __init__(self):
        super().__init__('yolo_aligned_depth_nav_goal')

        # OpenCV와 ROS 이미지 메시지 변환용 브리지
        self.bridge = CvBridge()
        # 카메라 내부 파라미터 (intrinsics)
        self.K = None
        # 카메라 프레임 이름
        self.camera_frame = None
        # 최신 map 좌표계의 목표점
        self.latest_map_point = None
        # 목표 전송 여부 플래그
        self.goal_sent = False

        # 사용 환경에 맞는 토픽 이름 설정
        self.rgb_topic = '/oakd/rgb/preview/image_raw'
        self.depth_topic = '/oakd/rgb/preview/aligned_depth_to_color'  # RGB와 align된 depth
        self.info_topic = '/oakd/rgb/preview/camera_info'

        # TF 변환을 위한 버퍼와 리스너 생성
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Nav2로 목표를 보내기 위한 액션 클라이언트 생성
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 카메라 정보, depth, RGB 이미지 구독
        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.create_subscription(Image, self.rgb_topic, self.rgb_callback, 10)

        # YOLO 바운딩박스 예시 (실제 사용시 별도 노드에서 받아와야 함)
        self.target_bbox = None  # (x1, y1, x2, y2, class_name)
        # 목표 좌표 추출 활성화 플래그
        self.capture_enabled = True

        # 디스플레이용 프레임 저장
        self.display_frame = None
        # 마지막 피드백 로그 시간
        self.last_feedback_log_time = 0

    def camera_info_callback(self, msg):
        # 카메라 내부 파라미터 저장 및 프레임 이름 저장
        self.K = np.array(msg.k).reshape(3, 3)
        self.camera_frame = msg.header.frame_id
        self.get_logger().info(f"Camera intrinsics: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")

    def rgb_callback(self, msg):
        # RGB 이미지를 OpenCV 형식으로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # 예시: 중앙에 바운딩박스 생성 (실제는 YOLO 결과를 받아야 함)
        h, w = frame.shape[:2]
        self.target_bbox = (w//2-50, h//2-50, w//2+50, h//2+50, "car")
        self.display_frame = frame

    def depth_callback(self, msg):
        # 카메라 파라미터, 바운딩박스, 활성화 플래그 확인
        if self.K is None or self.target_bbox is None or not self.capture_enabled:
            return

        try:
            # Depth 이미지를 OpenCV 형식으로 변환
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth image conversion failed: {e}")
            return

        # 바운딩박스 중심 좌표 계산 (YOLO 결과 사용)
        x1, y1, x2, y2, class_name = self.target_bbox
        u = int((x1 + x2) // 2)
        v = int((y1 + y2) // 2)

        # Depth 이미지가 RGB와 align되어 있으므로 (u,v) 그대로 사용
        z = float(depth_image[v, u])
        if z == 0.0:
            self.get_logger().warn('Invalid depth at bbox center')
            return

        # 픽셀 좌표(u,v)와 depth(z)로 3D 카메라좌표계 (x, y, z) 계산
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # 카메라 프레임 이름 결정
        camera_frame = self.camera_frame or msg.header.frame_id

        # 카메라좌표계의 3D 포인트 메시지 생성
        pt = PointStamped()
        pt.header.frame_id = camera_frame
        pt.header.stamp = msg.header.stamp
        pt.point.x = x
        pt.point.y = y
        pt.point.z = z

        # TF를 이용해 map 좌표계로 변환
        try:
            pt_latest = PointStamped()
            pt_latest.header.frame_id = camera_frame
            pt_latest.header.stamp = rclpy.time.Time().to_msg()  # 최신 시간 사용
            pt_latest.point = pt.point
            # map 좌표계로 변환 (0.5초 타임아웃)
            pt_map = self.tf_buffer.transform(pt_latest, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
            self.latest_map_point = pt_map
            self.get_logger().info(f"map: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f})")
            # 목표를 아직 보내지 않았다면 Nav2로 목표 전송
            if not self.goal_sent:
                self.send_goal()
                self.goal_sent = True
        except Exception as e:
            self.get_logger().warn(f"TF to map failed: {e}")

    def send_goal(self):
        # map 좌표계의 목표점이 없으면 경고 후 종료
        if self.latest_map_point is None:
            self.get_logger().warn("No map coordinate available to send as goal.")
            return

        # 목표 PoseStamped 메시지 생성 (z=0, w=1로 평면 이동)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.latest_map_point.point.x
        pose.pose.position.y = self.latest_map_point.point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        # Nav2 NavigateToPose 액션 메시지 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f"Sending goal to map position: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        # 액션 서버가 준비될 때까지 대기
        self.action_client.wait_for_server()
        # 비동기로 목표 전송, 피드백 콜백 등록
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Nav2가 목표를 수락했는지 확인
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by Nav2.')
            return
        self.get_logger().info('Goal accepted. Waiting for result...')
        # 결과 비동기 대기, 완료시 콜백 등록
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        # 목표 완료 결과 처리
        result = future.result().result
        self.get_logger().info(f"Goal finished with result code: {future.result().status}")
        self.get_logger().info("Shutting down after goal.")
        # 목표 완료 후 노드 종료
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        # 목표까지 남은 거리 등 피드백 처리 (1초에 한 번만 출력)
        feedback = feedback_msg.feedback
        current_time = time.time()
        if current_time - self.last_feedback_log_time > 1.0:
            self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f} m")
            self.last_feedback_log_time = current_time

def main():
    # ROS2 노드 초기화 및 실행
    rclpy.init()
    node = YoloAlignedDepthNavGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()

if __name__ == '__main__':
    main()
