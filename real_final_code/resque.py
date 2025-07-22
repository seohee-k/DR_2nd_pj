import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool

class ResqueNode(Node):
    def __init__(self):
        super().__init__('resque_node', namespace='robot7')
        self.employee_stack = []
        self.executing = False
        self.fire_flag = False

        self.create_subscription(Bool, 'fire_flag', self.fire_flag_cb, 10)
        self.create_subscription(Point, 'employee_stack', self.employee_cb, 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(2.0, self.go_next_employee)

        self.get_logger().info('Resque node initialized!')

    def fire_flag_cb(self, msg):
        if msg.data and not self.fire_flag:
            self.fire_flag = True
            self.get_logger().warn('화재 신호 수신, 직원 대피 시작')

    def employee_cb(self, msg):
        if not any([abs(e.x - msg.x) < 0.1 and abs(e.y - msg.y) < 0.1 for e in self.employee_stack]):
            self.employee_stack.append(msg)
            self.get_logger().info(f'받은 직원 좌표 저장: ({msg.x:.2f}, {msg.y:.2f})')

    def go_next_employee(self):
        if not self.fire_flag or self.executing or not self.employee_stack:
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
        self.executing = True
        send_goal_future = self.action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected!')
            self.executing = False
            return
        self.get_logger().info('Goal accepted!')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info('Goal result: 직원 대피 위치 도착')
        self.executing = False

if __name__ == '__main__':
    rclpy.init()
    node = ResqueNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()