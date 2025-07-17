import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.action import ActionClient
from cv_bridge import CvBridge
import numpy as np

class NavToTargetNode(Node):
    def __init__(self):
        super().__init__('navto_target_node')
        # OpenCV-ROS 이미지 변환용 브리지
        self.bridge = CvBridge()
        # Nav2 액션 클라이언트 생성 (목표점 이동용)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # 최신 depth 이미지 저장 변수
        self.depth_img = None
        # 최신 바운딩박스 정보 저장 변수 (YOLO 결과)
        self.bbox = None
        # 이동 명령 중복 방지 플래그
        self.moving = False
        # Depth 이미지 구독 (stereo depth topic)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        # YOLO 감지 결과 구독 (class, cx, cy, conf)
        self.create_subscription(String, '/yolo_detection', self.bbox_callback, 10)

    def depth_callback(self, msg):
        # Depth 이미지를 OpenCV 형식으로 변환하여 저장
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # 바운딩박스 정보가 있으면 이동 시도
        self.try_to_move()

    def bbox_callback(self, msg):
        # YOLO 감지 결과 수신 (예: "car,320,240,0.98")
        tokens = msg.data.split(',')
        if len(tokens) == 4:
            class_name, cx, cy, conf = tokens
            # 바운딩박스 정보 저장 (클래스, 중심 x, 중심 y, 신뢰도)
            self.bbox = (class_name, int(cx), int(cy), float(conf))
        # depth 이미지가 있으면 이동 시도
        self.try_to_move()

    def try_to_move(self):
        # 바운딩박스와 depth 이미지가 모두 준비된 경우에만 실행
        if self.bbox and self.depth_img is not None and not self.moving:
            class_name, cx, cy, conf = self.bbox
            # 바운딩박스 중심의 depth 값 추출
            z = float(self.depth_img[cy, cx])
            if z == 0:  # 유효하지 않은 depth 값
                self.get_logger().warn(f"[navto_target] Invalid depth at ({cx},{cy})")
                return
            # 실제로는 카메라 intrinsic, TF 변환 필요 (여기선 단순히 z축만 사용)
            self.get_logger().info(f"[navto_target] Target: {class_name}, Depth: {z:.2f}m")
            # 목표 PoseStamped 메시지 생성 (예시: z축 방향으로 이동)
            pose = PoseStamped()
            pose.header.frame_id = 'map'  # 실제 환경에 맞게 수정
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = z  # 실제로는 변환 필요, 예시로 z만 사용
            pose.pose.orientation.w = 1.0
            # Nav2 액션 목표 메시지 생성
            goal = NavigateToPose.Goal()
            goal.pose = pose
            # 액션 서버가 준비될 때까지 대기
            self.action_client.wait_for_server()
            self.moving = True  # 이동 시작
            # 비동기로 목표 전송 (피드백/결과 콜백은 미구현)
            self._send_goal_future = self.action_client.send_goal_async(goal)
            self._send_goal_future.add_done_callback(self.goal_done_callback)

    def goal_done_callback(self, future):
        self.moving = False  # 이동 완료 후 플래그 해제

def main():
    rclpy.init()
    node = NavToTargetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
