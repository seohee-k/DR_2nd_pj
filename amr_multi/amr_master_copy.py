import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener
import math

from my_interface.msg import YoloInfo

WAYPOINTS = [
    (-2.238, -0.081), (-2.099, 2.197), (-0.396, 2.411), (-0.382, 1.347),
    (-0.547, 2.352), (-2.091, 2.240), (-2.116, 0.382), (-0.436, 0.121)
]

class AMRMasterNode(Node):
    def __init__(self, robot_id='7'):
        super().__init__('amr_master_node', namespace=f'robot{robot_id}')
        self.state = "patrol"
        self.robot_id = robot_id

        self.current_waypoint = 0
        self.goal_running = False
        self.goal_handle = None

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.K = None
        self.camera_frame = None
        self.create_subscription(CameraInfo, f'/robot{robot_id}/oakd/rgb/camera_info', self.camera_info_callback, 1)

        self.target_class = None
        self.target_x = None
        self.target_y = None
        self.target_conf = None
        self.target_time = None
        self.close_enough_distance = 1.0
        self.approach_distance = 0.15
        self.block_goal_updates = False

        self.create_subscription(YoloInfo, '/detected_target', self.target_callback, 10)

        self.timer = self.create_timer(1.0, self.main_loop)

        self.employee_stack = []
        self.resque_executing = False

        self.get_logger().info('AMR Master node (patrol/fire/intruder) initialized!')

    def camera_info_callback(self, msg):
        self.K = msg.k
        self.camera_frame = msg.header.frame_id

    def main_loop(self):
        if self.state == "patrol":
            self.patrol_step()
        elif self.state == "firestop":
            self.firestop_step()
        elif self.state == "intruder":
            self.intruder_step()
        elif self.state == "resque":
            self.resque_step()

    # ---------- 순찰 (Waypoint Patrol) ----------
    def patrol_step(self):
        if self.state != "patrol":
            return
        # goal_running이 False일 때만 waypoint goal 전송
        if not self.goal_running and self.action_client.wait_for_server(timeout_sec=1.0):
            self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.state != "patrol":
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
        self.get_logger().info(f"[PATROL] Waypoint 이동: ({x:.2f}, {y:.2f})")
        self.goal_running = True
        send_goal_future = self.action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.patrol_goal_response_callback)

    def patrol_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected!')
            self.goal_running = False
            return
        self.get_logger().info('Goal accepted!')
        self.goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.patrol_result_callback)

    def patrol_result_callback(self, future):
        self.get_logger().info('[PATROL] Waypoint 도착!')
        self.goal_running = False
        self.goal_handle = None
        self.current_waypoint = (self.current_waypoint + 1) % len(WAYPOINTS)
        # 다음 waypoint 전송은 timer loop에서 자동 반복

    # ---------- 타겟 감지 콜백 ----------
    def target_callback(self, msg: YoloInfo):
        now = self.get_clock().now().nanoseconds * 1e-9
        if msg.class_name.lower() == "employee" and msg.confidence >= 0.7:
            # 중복 좌표 방지
            if not any(abs(e[0] - msg.x) < 0.1 and abs(e[1] - msg.y) < 0.1 for e in self.employee_stack):
                self.employee_stack.append((msg.x, msg.y))
                self.get_logger().info(f"👷 직원 감지, 좌표 저장: ({msg.x:.2f}, {msg.y:.2f})")
        elif msg.class_name.lower() == "fire" and msg.confidence >= 0.7:
            self.get_logger().warn(f"🔥 화재 감지! ({msg.x:.2f}, {msg.y:.2f}) → resque 모드 전환")
            self.change_state("resque")
            self.target_class = "fire"
            self.target_x = msg.x
            self.target_y = msg.y
            self.target_conf = msg.confidence
        elif msg.class_name.lower() in ["car", "intruder"] and msg.confidence >= 0.7:
            self.get_logger().warn(f"🚗 침입자 감지! ({msg.x:.2f}, {msg.y:.2f}) → intruder 모드 전환")
            self.change_state("intruder")
            self.target_class = "car"
            self.target_x = msg.x
            self.target_y = msg.y
            self.target_conf = msg.confidence
            self.target_time = now

    def change_state(self, new_state):
        # 상태 전환 시 기존 goal 있으면 cancel
        if self.goal_handle:
            self.get_logger().info("기존 goal 취소 (state change)")
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self.goal_running = False
        self.state = new_state
        self.get_logger().info(f"상태 전환: {new_state}")

    # ---------- 화재 대응 ----------
    def firestop_step(self):
        if self.target_class != "fire" or self.target_x is None or self.target_y is None:
            self.get_logger().warn("[FIRESTOP] 화재 좌표 없음, patrol로 복귀")
            self.change_state("patrol")
            return
        self.get_logger().info(f"[FIRESTOP] 화재 위치 접근: ({self.target_x:.2f}, {self.target_y:.2f})")
        self.approach_target(self.target_x, self.target_y, self.close_enough_distance, mode="fire")

    # ---------- 침입자 추적 ----------
    def intruder_step(self):
        if self.target_class != "car" or self.target_x is None or self.target_y is None:
            self.get_logger().warn("[INTRUDER] 침입자 좌표 없음, patrol로 복귀")
            self.change_state("patrol")
            return
        self.get_logger().info(f"[INTRUDER] 침입자 위치 접근: ({self.target_x:.2f}, {self.target_y:.2f})")
        self.approach_target(self.target_x, self.target_y, self.close_enough_distance, mode="intruder")

    # ---------- 직원 대피 ----------
    def resque_step(self):
        if self.resque_executing or not self.employee_stack:
            return
        x, y = self.employee_stack.pop(0)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(f"[RESQUE] 직원 대피 좌표로 이동: ({x:.2f}, {y:.2f})")
        self.resque_executing = True
        send_goal_future = self.action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.resque_goal_response_callback)

    def resque_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('[RESQUE] Goal rejected!')
            self.resque_executing = False
            return
        self.get_logger().info('[RESQUE] Goal accepted!')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.resque_result_callback)

    def resque_result_callback(self, future):
        self.get_logger().info('[RESQUE] 직원 대피 위치 도착')
        self.resque_executing = False

    # ---------- 공통 접근 함수 ----------
    def approach_target(self, target_x, target_y, close_enough_distance, mode="fire"):
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"TF 변환 실패: {e}")
            return

        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)

        if distance < close_enough_distance:
            if mode == "fire":
                self.get_logger().info("[FIRESTOP] 화재 안전거리 도달! 소화기 투척 및 patrol 복귀")
                self.change_state("patrol")
            elif mode == "intruder":
                self.get_logger().info("[INTRUDER] 침입자 근처 도달! patrol 복귀")
                self.change_state("patrol")
            self.block_goal_updates = True
            return

        if distance > self.approach_distance:
            ratio = (distance - self.approach_distance) / distance
            adjusted_x = current_x + dx * ratio
            adjusted_y = current_y + dy * ratio
        else:
            adjusted_x = current_x
            adjusted_y = current_y

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = adjusted_x
        pose.pose.position.y = adjusted_y

        yaw = math.atan2(adjusted_y - current_y, adjusted_x - current_x)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        goal = NavigateToPose.Goal()
        goal.pose = pose

        if self.goal_handle and not self.block_goal_updates:
            self.get_logger().info("Canceling previous goal...")
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self.goal_running = False

        if not self.block_goal_updates and not self.goal_running:
            self.get_logger().info(f"[{mode.upper()}] 접근 goal 전송: ({adjusted_x:.2f}, {adjusted_y:.2f})")
            self.goal_running = True
            self.action_client.wait_for_server()
            send_goal_future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
            send_goal_future.add_done_callback(self.approach_goal_response_callback)

    def approach_goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal was rejected.")
            self.goal_running = False
            return
        self.get_logger().info("Goal accepted.")
        get_result_future = self.goal_handle.get_result_async()
        get_result_future.add_done_callback(self.approach_result_callback)

    def approach_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Goal finished with result code: {future.result().status}")
        self.goal_handle = None
        self.goal_running = False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_distance = feedback.distance_remaining
        self.get_logger().info(f"Distance remaining: {current_distance:.2f} m")

def main():
    rclpy.init()
    node = AMRMasterNode('7')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
