#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

# TurtleBot4Navigator를 직접 사용하지 않으므로 임시로 더미 클래스 정의
class TurtleBot4Navigator:
    def startToPose(self, pose):
        # 실제 로봇 이동 대신 로그 출력
        print(f"[SIM] Moving to pose: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}")
        time.sleep(1)  # 이동 시뮬레이션
        print("[SIM] Move completed.")

    def isTaskComplete(self):
        # 이동 완료 여부 확인 대신 항상 True 반환
        return True

# # 더미 TurtleBot4Directions (필요 시 사용)
# class TurtleBot4Directions:
#     EAST = "EAST"
#     NORTH = "NORTH"

class FireResponseHandler(Node):
    def __init__(self, robot_id):
        super().__init__(f'fire_response_handler_{robot_id}')
        self.robot_id = robot_id
        self.navigator = TurtleBot4Navigator()
        self.tracked_fire_pose = PoseStamped()
        self.tracked_fire_pose.pose.position.x = 1.0
        self.tracked_fire_pose.pose.position.y = 1.0
        self.tracked_fire_pose.pose.orientation.w = 1.0
        self.other_robot_position = {'x': 5.0, 'y': 5.0} if robot_id == '1' else {'x': 0.0, 'y': 0.0}
        self.fire_response_phase = 0
        self.patrol_route = [
            PoseStamped(), PoseStamped(), PoseStamped()
        ]
        # 순찰 경로 하드코딩
        self.patrol_route[0].pose.position.x, self.patrol_route[0].pose.position.y = -0.2326, 0.2989
        self.patrol_route[1].pose.position.x, self.patrol_route[1].pose.position.y = -1.9897, -2.7512
        self.patrol_route[2].pose.position.x, self.patrol_route[2].pose.position.y = -2.3337, 2.4464

    def report_emergency(self):
        msg = f'EMERGENCY from TurtleBot{self.robot_id}: Fire at (1.00, 1.00)'
        self.get_logger().info('Emergency reported to 119 and 112.')

    def assign_response(self):
        # robot1_pose를 하드코딩으로 대체
        robot1_pose_x, robot1_pose_y = 0.0, 0.0  # 가상의 현재 로봇 위치
        robot2_pose = self.other_robot_position
        robot1_dist = ((self.tracked_fire_pose.pose.position.x - robot1_pose_x) ** 2 +
                      (self.tracked_fire_pose.pose.position.y - robot1_pose_y) ** 2) ** 0.5
        robot2_dist = ((self.tracked_fire_pose.pose.position.x - robot2_pose['x']) ** 2 +
                      (self.tracked_fire_pose.pose.position.y - robot2_pose['y']) ** 2) ** 0.5
        if robot1_dist < robot2_dist and self.robot_id == '1':
            self.get_logger().info('[RESPONSE] TurtleBot4A assigned to fire response')
        elif robot2_dist < robot1_dist and self.robot_id == '2':
            self.get_logger().info('[RESPONSE] TurtleBot4B assigned to fire response')

    def extinguish_fire(self):
        self.get_logger().info('Extinguishing fire...')
        time.sleep(2)
        self.get_logger().info('Fire extinguished.')

    def patrol_intruder(self):
        self.get_logger().info('Starting intruder patrol...')
        for pose in self.patrol_route:
            self.navigator.startToPose(pose)
        self.get_logger().info('Intruder patrol completed.')

    def handle_fire_response(self):
        if not self.tracked_fire_pose:
            self.get_logger().error("No fire position!")
            return
        self.report_emergency()
        self.assign_response()
        self.get_logger().info(f"[NAV] Moving to fire at (1.00, 1.00)")
        self.navigator.startToPose(self.tracked_fire_pose)
        self.get_logger().info("[PROTOCOL] Extinguishing fire")
        self.extinguish_fire()
        self.get_logger().info("[PROTOCOL] Starting intruder patrol")
        self.patrol_intruder()

def main(args=None):
    rclpy.init(args=args)
    handler1 = FireResponseHandler('1')
    handler2 = FireResponseHandler('2')

    # 화재 탐지 시 119, 112 신고 및 대응
    if True:  # 화재 탐지 시뮬레이션 (fire detect)
        print("Fire detected, initiating emergency response...")
        handler1.report_emergency()  # 119, 112 신고
        handler1.assign_response()   # 로봇 할당
        handler2.report_emergency()
        handler2.assign_response()
        handler1.handle_fire_response()  # 전체 대응 (이동, 진압, 순찰)
        handler2.handle_fire_response()

    # 스핀 대신 로그 확인 후 종료
    handler1.destroy_node()
    handler2.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()