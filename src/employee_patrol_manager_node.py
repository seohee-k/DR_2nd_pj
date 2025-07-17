import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Bool
from rclpy.action import ActionClient
from collections import deque

class EmployeePatrolManagerNode(Node):
    def __init__(self):
        super().__init__('employee_patrol_manager_node')

        # 순찰 경로 (웨이포인트 좌표 리스트)
        self.patrol_points = [
            (-4.86401844, -2.32932829),
            (-4.88449812, -4.14383363),
            (-3.39365720, -4.30793952),
            (-3.30976581, -3.45941638),
            (-3.52061772, -4.34016227),
            (-4.62198877, -2.28764653),
            (-3.44791960, -2.17049479),
        ]
        self.exit_point = (-5.37321043, -4.49130487)  # 출입구(탈출구) 좌표

        self.current_patrol_idx = 0
        self.patrolling = True
        self.evacuate_active = False
        self.employee_stack = deque()  # 발견한 직원 좌표 저장

        self.action_client = ActionClient(self, NavigateToPose, '/robot7/navigate_to_pose')

        # 직원 좌표(YOLO/Depth 추론 결과) 구독
        self.create_subscription(Float32MultiArray, '/employee_position', self.emp_cb, 10)
        # 화재 신호 구독
        self.create_subscription(Bool, '/fire_alarm', self.fire_cb, 10)

        # 순찰 시작
        self.get_logger().info('[순찰] 시작!')
        self.patrol_next_point()

    def emp_cb(self, msg):
        x, y = msg.data
        self.employee_stack.append((x, y))
        self.get_logger().info(f'[발견] 직원 좌표 스택 추가: ({x:.2f}, {y:.2f}) [{len(self.employee_stack)}명]')

    def fire_cb(self, msg):
        if msg.data and not self.evacuate_active:
            self.evacuate_active = True
            self.patrolling = False
            self.get_logger().warn('[화재] 직원 대피 모드 진입!')
            self.evacuate_next_employee()

    # ---------- 순찰 모드 ----------
    def patrol_next_point(self):
        if not self.patrolling:
            return
        idx = self.current_patrol_idx
        x, y = self.patrol_points[idx]
        self.get_logger().info(f'[순찰] {idx+1}/{len(self.patrol_points)}번 포인트로 이동: ({x:.2f}, {y:.2f})')
        self.send_nav_goal(x, y, self.patrol_goal_callback)

    def patrol_goal_callback(self, future):
        result = future.result().result
        self.get_logger().info('[순찰] 포인트 도착!')
        # 다음 포인트로 순환 이동
        self.current_patrol_idx = (self.current_patrol_idx + 1) % len(self.patrol_points)
        if self.patrolling:
            self.patrol_next_point()

    # ---------- 대피 모드 ----------
    def evacuate_next_employee(self):
        if self.employee_stack:
            x, y = self.employee_stack.pop()  # 최근 직원부터 pop
            self.get_logger().warn(f'[대피] 직원 좌표로 이동: ({x:.2f}, {y:.2f})')
            self.send_nav_goal(x, y, self.evacuate_goal_callback)
        else:
            ex, ey = self.exit_point
            self.get_logger().warn('[대피] 모든 직원 구조 완료! 출입구로 이동')
            self.send_nav_goal(ex, ey, self.exit_goal_callback)

    def evacuate_goal_callback(self, future):
        result = future.result().result
        self.get_logger().info('[대피] 직원 위치 도착! 다음 직원 구조')
        self.evacuate_next_employee()

    def exit_goal_callback(self, future):
        result = future.result().result
        self.get_logger().info('[대피] 출입구 도착! 대피 종료')
        self.evacuate_active = False
        # 필요하다면 여기서 재순찰 또는 종료 처리 가능

    # ---------- 공통: nav2 goal 전송 ----------
    def send_nav_goal(self, x, y, done_cb):
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        goal_msg.pose = pose

        self.get_logger().info(f'[NAV2] goal: ({x:.2f}, {y:.2f})')
        self.action_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda future: self._on_goal_done(future, done_cb))

    def _on_goal_done(self, future, user_cb):
        # nav2 action 결과 콜백
        if user_cb:
            user_cb(future)

def main(args=None):
    rclpy.init(args=args)
    node = EmployeePatrolManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
