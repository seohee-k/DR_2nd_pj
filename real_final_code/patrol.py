import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
from tf2_ros import Buffer, TransformListener
import threading

WAYPOINTS = [
    (-2.238, -0.081), (-2.099, 2.197), (-0.396, 2.411), (-0.382, 1.347),
    (-0.547, 2.352), (-2.091, 2.240), (-2.116, 0.382), (-0.436, 0.121)
]

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node', namespace='robot7')
        self.bridge = CvBridge()
        self.employee_stack = []
        self.current_waypoint = 0
        self.fire_detected = False
        self.goal_running = False

        self.K = None
        self.rgb_image = None
        self.depth_image = None
        self.camera_frame = None

        self.depth_topic = '/robot7/oakd/stereo/image_raw'
        self.rgb_topic = '/robot7/oakd/rgb/image_raw/compressed'
        self.info_topic = '/robot7/oakd/rgb/camera_info'

        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 1)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 1)
        self.create_subscription(CompressedImage, self.rgb_topic, self.rgb_callback, 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.emp_pub = self.create_publisher(Point, 'employee_stack', 10)
        self.fire_pub = self.create_publisher(Bool, 'fire_flag', 10)
        self.model = YOLO('/home/djqsp2/rokey3_E4_ws/src/best.pt')

        self.get_logger().info('Patrol node initialized!')

        self.timer = self.create_timer(1.0, self.try_send_first_goal)
        self.first_goal_sent = False

        self.fire_timer = self.create_timer(30.0, self.fake_fire)

    def try_send_first_goal(self):
        if not self.first_goal_sent and self.action_client.wait_for_server(timeout_sec=1.0):
            self.first_goal_sent = True
            self.send_next_waypoint()
            self.timer.cancel()

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        self.camera_frame = msg.header.frame_id

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def rgb_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.rgb_image = rgb
        self.detect_employee()

    def send_next_waypoint(self):
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
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected!')
            self.goal_running = False
            return
        self.get_logger().info('Goal accepted!')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info('Goal result: 도착!')
        self.goal_running = False
        self.current_waypoint = (self.current_waypoint + 1) % len(WAYPOINTS)
        self._oneshot_timer = self.create_timer(0.5, self._timer_send_next_waypoint)

    def _timer_send_next_waypoint(self):
        if not self.fire_detected:
            self.send_next_waypoint()
        self._oneshot_timer.cancel()

    def detect_employee(self):
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
                    pt_map = self.tf_buffer.transform(pt, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
                    emp_point = Point()
                    emp_point.x = pt_map.point.x
                    emp_point.y = pt_map.point.y
                    emp_point.z = pt_map.point.z
                    if not any([np.isclose(emp_point.x, e.x, atol=0.1) and np.isclose(emp_point.y, e.y, atol=0.1) for e in self.employee_stack]):
                        self.employee_stack.append(emp_point)
                        self.get_logger().info(f'Employee 발견! 맵좌표=({emp_point.x:.2f}, {emp_point.y:.2f}, {emp_point.z:.2f})')
                        self.emp_pub.publish(emp_point)
                except Exception as e:
                    self.get_logger().warn(f"TF 변환 실패: {e}")

    def fake_fire(self):
        if not self.fire_detected:
            self.fire_detected = True
            self.get_logger().warn('🔥 화재 감지됨! 직원 좌표 스택 publish, 순찰 중지')
            self.fire_pub.publish(Bool(data=True))
            for emp in self.employee_stack:
                self.emp_pub.publish(emp)
                self.get_logger().info(f'직원 좌표 publish: ({emp.x:.2f}, {emp.y:.2f})')

if __name__ == '__main__':
    rclpy.init()
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()