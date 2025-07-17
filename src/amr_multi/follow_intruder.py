import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion

from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions

import numpy as np
import math

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import time

from my_interface.msg import YoloInfo


class FollowIntruder(Node):
    def __init__(self):
        super().__init__('follow_intruder')

        self.ready = False  # TF 준비 여부
        self.start_timer = self.create_timer(5.0, self.enable_tf)

        self.target_topic = '/detected_target'

        self.goal_handle = None
        self.current_distance = None
        self.close_enough_distance = 1  # meters 2.0 -> 1
        self.block_goal_updates = False
        self.close_distance_hit_count = 0  # To avoid reacting to a single bad reading

        self.last_feedback_log_time = time.time()

        self.create_subscription(YoloInfo,
                                 self.target_topic, 
                                 self.target_callback,
                                 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')


    def enable_tf(self):
        self.ready = True
        self.get_logger().info("✅ TF Tree 안정화 완료. 동작 시작.")
        self.start_timer.cancel() 

    def target_callback(self, msg: YoloInfo):

        now = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
        time_diff = (now - msg_time).nanoseconds * 1e-9  # 초 단위

        if time_diff > 1.0:  # 1초 이상 지난 데이터는 무시
            self.get_logger().warn(f"⏱️ 오래된 메시지 수신됨 ({time_diff:.2f}s 전), 무시합니다.")
            return

        self.get_logger().info(
            f"[YoloInfo] Detected: {msg.class_name}, conf: {msg.confidence:.2f}, x: {msg.x:.2f}, y: {msg.y:.2f}"
        )

        if msg.class_name.lower() == "employee" and msg.confidence >= 0.8:
            if self.goal_handle:
                self.get_logger().info("Canceling previous goal...")
                self.goal_handle.cancel_goal_async()

            self.send_goal(msg.x,msg.y)

       
    def send_goal(self,target_x,target_y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=1.0))
        current_x = trans.transform.translation.x
        current_y = trans.transform.translation.y

        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)

        approach_distance = 0.15  # 15cm 앞에서 멈추기

        # 덜 가는 좌표 계산
        if distance > approach_distance:
            ratio = (distance - approach_distance) / distance
            adjusted_x = current_x + dx * ratio
            adjusted_y = current_y + dy * ratio
        else:
            # 너무 가까우면 그냥 현재 위치 사용
            adjusted_x = current_x
            adjusted_y = current_y

        pose.pose.position.x = adjusted_x
        pose.pose.position.y = adjusted_y

        # yaw = math.atan2(self.latest_map_point.point.y - current_y, self.latest_map_point.point.x - current_x)
        yaw = math.atan2(adjusted_y - current_y, adjusted_x - current_x)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f"Sending goal to: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.current_distance = feedback.distance_remaining

        # Require 3 close readings to trigger the lock
        if self.current_distance is not None and self.current_distance < self.close_enough_distance:
            self.close_distance_hit_count += 1
        else:
            self.close_distance_hit_count = 0

        # if self.close_distance_hit_count >= 3 and not self.block_goal_updates:
        if self.close_distance_hit_count >= 3:
            # self.block_goal_updates = True
            self.get_logger().info("Confirmed: within 1 meter — blocking further goal updates.")

        now = time.time()
        if now - self.last_feedback_log_time > 1.0:
            self.get_logger().info(f"Distance remaining: {self.current_distance:.2f} m")
            self.last_feedback_log_time = now

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal was rejected.")
            return
        self.get_logger().info("Goal accepted.")
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Goal finished with result code: {future.result().status}")
        self.goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = FollowIntruder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()