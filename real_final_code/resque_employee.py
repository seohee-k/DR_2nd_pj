import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time

WAYPOINTS = [
    (-2.2381389141082764, -0.08192165195941925),
    (-2.099907636642456, 2.197780132293701),
    (-0.39694878458976746, 2.4117934703826904),
    (-0.3820028007030487, 1.3472325801849365),
    (-0.5478897094726562, 2.3528730869293213),
    (-2.0914320945739746, 2.2400667667388916),
    (-2.116553544998169, 0.38230445981025696),
    (-0.4361707270145416, 0.121768057346344)
]

class WaypointPatrol(Node):
    def __init__(self):
        super().__init__('waypoint_patrol', namespace='robot7')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoints = WAYPOINTS
        self.current = 0

    def send_next_goal(self):
        x, y = self.waypoints[self.current]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0  # 2D이동: 방향 중요 없으면 w=1

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f"Navigating to: ({x:.2f}, {y:.2f})")
        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected!')
            return

        self.get_logger().info('Goal accepted.')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Goal result: {future.result().status}")
        # 다음 waypoint로 이동
        self.current = (self.current + 1) % len(self.waypoints)
        time.sleep(1)
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPatrol()
    node.send_next_goal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
