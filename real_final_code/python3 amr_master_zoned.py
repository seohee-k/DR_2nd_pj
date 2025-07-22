import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener
import math
import time

from my_interface.msg import YoloInfo

# 각 구역 웨이포인트
A_POINTS = [
    (-2.238, -0.081), (-2.099, 2.197), (-0.396, 2.411), (-0.382, 1.347),
    (-0.547, 2.352), (-2.091, 2.240), (-2.116, 0.382), (-0.436, 0.121)
]
B_POINTS = [
    (-4.86401844, -2.32932829),
    (-4.88449812, -4.14383363),
    (-3.39365720, -4.30793952),
    (-3.30976581, -3.45941638),
    (-3.52061772, -4.34016227),
    (-4.62198877, -2.28764653),
    (-3.44791960, -2.17049479),
]
EXIT_X = -2.78
EXIT_Y = 2.7

class AMRMasterNode(Node):
    def __init__(self, robot_id='A'):
        super().__init__('amr_master_node', namespace=f'robot{robot_id}')
        self.state = "patrol"
        self.robot_id = robot_id

        # 순찰 경로: A구역/B구역 분리
        self.my_patrol_points = A_POINTS if robot_id == 'A' else B_POINTS
        self.other_patrol_points = B_POINTS if robot_id == 'A' else A_POINTS
        self.full_patrol_points = self.my_patrol_points + self.other_patrol_points

        self.current_waypoint = 0
        self.goal_running = False
        self.goal_handle = None

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.K = None
        self.camera_frame = None
        self.create_subscription(CameraInfo, f'/robot{robot_id}/oakd/rgb/camera_info', self.camera_info_callback, 1)

        self.timer = self.create_timer(1.0, self.main_loop)

        # 직원 감지 좌표 버퍼 및 스택
        self.cur_employee_coords = []
        self.last_employee_time = None
        self.employee_stack = []
        self.employee_detection_timeout = 3.0  # 3초

        # 대피/출구 플래그
        self.resque_executing = False
        self.going_to_exit = False
        self.exit_goal_sent = False

        self.create_subscription(YoloInfo, '/detected_target', self.target_callback, 10)
        # fire_flag pub/sub
        self.fireflag_pub = self.create_publisher(Bool, '/fire_flag', 10)
        self.create_subscription(Bool, '/fire_flag', self.fireflag_callback, 10)

        # fire/침입자 순찰 상태 관리
        self.fire_flag_sent = False
        self.fire_pos = None
        self.intruder_patrol_points = []
        self.intruder_patrol_idx = 0
        self.intruder_protocol_executed = False

        self.get_logger().info(f'AMR Master node (zone patrol/fire/rescue/intruder, id={robot_id}) initialized!')

    def camera_info_callback(self, msg):
        self.K = msg.k
        self.camera_frame = msg.header.frame_id

    def main_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        # 직원 좌표 평균값 저장 로직
        if self.cur_employee_coords and self.last_employee_time is not None:
            if now - self.last_employee_time > self.employee_detection_timeout:
                xs = [x for x, y in self.cur_employee_coords]
                ys = [y for x, y in self.cur_employee_coords]
                avg_x = sum(xs) / len(xs)
                avg_y = sum(ys) / len(ys)
                if not self.employee_stack or math.hypot(avg_x - self.employee_stack[-1][0], avg_y - self.employee_stack[-1][1]) > 0.2:
                    self.employee_stack.append((avg_x, avg_y))
                    self.get_logger().info(f"👷 [평균좌표 저장] 직원: ({avg_x:.2f}, {avg_y:.2f})")
                self.cur_employee_coords.clear()
                self.last_employee_time = None

        # 상태별 step 호출
        if self.state == "patrol":
            self.patrol_step()
        elif self.state == "fire":
            self.fire_step()
        elif self.state == "rescue":
            self.rescue_step()
        elif self.state == "intruder_patrol":
            self.intruder_patrol_step()

    def patrol_step(self):
        if not self.goal_running and self.action_client.wait_for_server(timeout_sec=2.0):
            x, y = self.my_patrol_points[self.current_waypoint]
            self.send_goal(x, y)

    def send_goal(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.goal_running = True
        send_goal_future = self.action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('[GOAL] Goal rejected!')
            self.goal_running = False
            self.current_waypoint = (self.current_waypoint + 1) % len(self.my_patrol_points)
            return
        self.goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.goal_running = False
        self.goal_handle = None
        if self.state == "patrol":
            self.current_waypoint = (self.current_waypoint + 1) % len(self.my_patrol_points)
        elif self.state == "rescue":
            self.resque_executing = False
        elif self.state == "intruder_patrol":
            self.intruder_patrol_idx += 1

    def target_callback(self, msg: YoloInfo):
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.state == "patrol":
            if msg.class_name.lower() == "employee" and msg.confidence >= 0.7:
                self.cur_employee_coords.append((msg.x, msg.y))
                self.last_employee_time = now
                self.get_logger().info(f"👷 직원 감지, 좌표 버퍼링: ({msg.x:.2f}, {msg.y:.2f})")
            elif msg.class_name.lower() == "fire" and msg.confidence >= 0.7:
                self.get_logger().warn(f"🔥 화재 감지! ({msg.x:.2f}, {msg.y:.2f}) → fire_flag 발행, fire 모드")
                self.fireflag_pub.publish(Bool(data=True))
                self.fire_flag_sent = True
                self.fire_pos = (msg.x, msg.y)
                self.change_state("fire")
        elif self.state == "intruder_patrol":
            if msg.class_name.lower() == "car" and msg.confidence >= 0.7 and not self.intruder_protocol_executed:
                self.intruder_protocol_executed = True
                self.get_logger().warn(f"[INTRUDER] 침입자 감지 ({msg.x:.2f}, {msg.y:.2f}) → 대응 프로토콜 시작!")
                self.get_logger().info("[INTRUDER] 1차 대응: LED 점등, 경고음 출력")
                self.get_logger().info("[INTRUDER] 2차 대응: 112 신고")
                self.get_logger().info("[INTRUDER] 3차 대응: 테이저건 발사, 포박줄 발사")
            elif msg.class_name.lower() == "employee" and msg.confidence >= 0.7:
                self.get_logger().info(f"[INTRUDER] 내부 잔여 직원 발견! 출구로 안내")
                self.send_goal(EXIT_X, EXIT_Y)

    def fireflag_callback(self, msg: Bool):
        if msg.data:
            if self.state == "patrol" and not self.fire_flag_sent:
                self.get_logger().warn("🔥 [SUB] fire_flag 수신, rescue 모드 진입")
                self.change_state("rescue")

    def change_state(self, new_state):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self.goal_running = False
        self.state = new_state
        if new_state == "fire":
            self.fire_extinguish_started = False
        if new_state == "rescue":
            self.going_to_exit = False
            self.exit_goal_sent = False
        if new_state == "intruder_patrol":
            self.intruder_patrol_points = self.full_patrol_points
            self.intruder_patrol_idx = 0
            self.intruder_protocol_executed = False

    def fire_step(self):
        if not self.fire_pos:
            return
        if not getattr(self, 'fire_extinguish_started', False):
            self.get_logger().info(f"[FIRE] 화재 진압 위치로 이동: ({self.fire_pos[0]:.2f},{self.fire_pos[1]:.2f})")
            self.send_goal(self.fire_pos[0], self.fire_pos[1])
            self.fire_extinguish_started = True
            return
        if not self.goal_running:
            self.get_logger().info("[FIRE] 화재 진압 완료! 전체 구역 내부 순찰 시작 (침입자, 잔여 직원 탐색)")
            time.sleep(1)
            self.change_state("intruder_patrol")

    def rescue_step(self):
        if not self.going_to_exit:
            if self.resque_executing:
                return
            if self.employee_stack:
                x, y = self.employee_stack.pop(0)
                self.get_logger().info(f"[RESCUE] 직원 대피 좌표로 이동: ({x:.2f},{y:.2f})")
                self.resque_executing = True
                self.send_goal(x, y)
                return
            else:
                self.going_to_exit = True
                self.exit_goal_sent = False
                return

        if self.going_to_exit and not self.exit_goal_sent and not self.resque_executing:
            self.get_logger().info(f"[RESCUE] 모든 직원 대피 완료! 출구({EXIT_X:.2f},{EXIT_Y:.2f})로 이동, 입구 봉쇄")
            self.exit_goal_sent = True
            self.resque_executing = True
            self.send_goal(EXIT_X, EXIT_Y)

        if self.going_to_exit and self.exit_goal_sent and not self.resque_executing:
            self.get_logger().info("[RESCUE] 출구 도착! 입구 봉쇄, patrol 복귀")
            time.sleep(1)
            self.change_state("patrol")

    def intruder_patrol_step(self):
        # 전체 웨이포인트 순회
        if self.intruder_patrol_idx < len(self.intruder_patrol_points) and not self.goal_running:
            x, y = self.intruder_patrol_points[self.intruder_patrol_idx]
            self.get_logger().info(f"[INTRUDER] 내부 순찰지점 이동: ({x:.2f},{y:.2f})")
            self.send_goal(x, y)
        elif self.intruder_patrol_idx >= len(self.intruder_patrol_points) and not self.goal_running:
            self.get_logger().info("[INTRUDER] 내부 순찰 종료, patrol 복귀")
            self.change_state("patrol")

def main():
    rclpy.init()
    import sys
    robot_id = sys.argv[1] if len(sys.argv) > 1 else 'A'  # 'A' or 'B'
    node = AMRMasterNode(robot_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
