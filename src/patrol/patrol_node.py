import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import random
import time
from yoloinference.msg import BoundingBoxes, BoundingBox

BATTERY_HIGH = 0.95
BATTERY_LOW = 0.2
BATTERY_CRITICAL = 0.1

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        # 배터리 잔량을 임의로 시뮬레이션 (실제 토픽 구독 X)
        self.battery_percent = 1.0
        # YOLO 감지 결과 구독 (BoundingBoxes 타입)
        self.create_subscription(BoundingBoxes, '/yolov8/bounding_boxes', self.yolo_callback, 10)
        # 이동 명령 퍼블리셔
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # TurtleBot4Navigator 인스턴스
        self.navigator = TurtleBot4Navigator()
        # 순찰 경로 (실제 좌표로 수정)
        self.goal_poses = [
            self.navigator.getPoseStamped([-2.44, 0.02], TurtleBot4Directions.EAST),
            self.navigator.getPoseStamped([-2.17, -2.75], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-0.19, -2.57], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-0.30, -1.25], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-0.63, -1.92], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-1.43, -2.03], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-1.49, -1.98], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-1.63, -1.00], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-0.55, -0.01], TurtleBot4Directions.NORTH),
        ]
        self.position_index = 0
        self.intruder_protocol_running = False
        self.fire_protocol_running = False
        self.docking = False

        # 초기화 및 도킹 상태 확인
        self.init_robot()

        # 1초마다 순찰 및 배터리 체크
        self.create_timer(1.0, self.patrol_logic)

    def init_robot(self):
        if not self.navigator.getDockedStatus():
            self.get_logger().info('[INIT] Docking before initializing pose')
            self.navigator.dock()
        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()
        self.get_logger().info('[INIT] 초기화 및 언도킹 완료')

    def patrol_logic(self):
        self.battery_percent -= random.uniform(0.005, 0.02)
        self.battery_percent = max(self.battery_percent, 0.0)
        self.get_logger().info(f'[BATTERY] 현재 잔량: {self.battery_percent*100:.1f}%')

        if self.battery_percent < BATTERY_CRITICAL:
            self.get_logger().error('[BATTERY] 배터리 critically low! 로봇을 종료하거나 충전하세요.')
            return
        elif self.battery_percent < BATTERY_LOW:
            if not self.docking:
                self.get_logger().info('[DOCK] 배터리 부족! 도킹 및 충전 시작 (시뮬레이션)')
                self.docking = True
                self.navigator.startToPose(self.navigator.getPoseStamped([-1.0, 1.0], TurtleBot4Directions.EAST))
                self.navigator.dock()
            if self.navigator.getDockedStatus():
                self.get_logger().info('[DOCK] 충전 중... (시뮬레이션)')
                self.battery_percent += 0.05
                if self.battery_percent >= BATTERY_HIGH:
                    self.get_logger().info('[DOCK] 충전 완료! 언도킹 및 순찰 재개 (시뮬레이션)')
                    self.navigator.undock()
                    self.docking = False
                    self.position_index = 0
        else:
            if not self.intruder_protocol_running and not self.fire_protocol_running and not self.docking:
                self.get_logger().info(f'[PATROL] {self.position_index+1}/{len(self.goal_poses)}번 웨이포인트로 이동')
                self.navigator.startToPose(self.goal_poses[self.position_index])
                self.position_index = (self.position_index + 1) % len(self.goal_poses)

    def yolo_callback(self, msg):
        # msg.boxes는 BoundingBox[] 배열
        for box in msg.boxes:
            self.get_logger().info(
                f"YOLO 감지: {box.class_name} (신뢰도: {box.confidence:.2f}) "
                f"좌표: ({box.xmin},{box.ymin})~({box.xmax},{box.ymax})"
            )
            if box.class_name == "car" and not self.intruder_protocol_running:
                self.intruder_protocol_running = True
                self.stop_robot()
                self.intruder_protocol()
            elif box.class_name == "fire" and not self.fire_protocol_running:
                self.fire_protocol_running = True
                self.stop_robot()
                self.fire_protocol()

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)
        self.navigator.cancelTask()
        self.get_logger().info('[ACTION] 로봇 정지! (이동 중단)')

    def intruder_protocol(self):
        self.get_logger().warn("[ALERT] 침입자(car) 감지! 1단계: 음성+LED 경고 (로그만)")
        stay_time = random.randint(5, 10)
        self.get_logger().info(f"[INTRUDER] 침입자 행동(랜덤): {stay_time}초 대기 (불응?)")
        time.sleep(stay_time)
        if stay_time >= 8:
            self.get_logger().warn("[ALERT] 2단계: 112신고, 테이저 발사! (로그만)")
            move_after_taser = random.choice([True, False])
            if move_after_taser:
                self.get_logger().warn("[ALERT] 침입자 움직임 → 3단계: Rope gun(포박줄) 발사! (로그만)")
            else:
                self.get_logger().info("[INTRUDER] 침입자 움직이지 않음. 경찰 대기 (로그만).")
        else:
            self.get_logger().info("[INTRUDER] 침입자 도주/퇴장. 순찰 재개 (로그만).")
        police_time = random.randint(2, 5)
        self.get_logger().info(f"[ALERT] 경찰(112) 도착까지 {police_time}초 대기 (로그만).")
        time.sleep(police_time)
        self.get_logger().info("[PATROL] 경찰 도착, 순찰 복귀 (로그만).")
        self.intruder_protocol_running = False

    def fire_protocol(self):
        self.get_logger().warn("[FIRE] 화재 감지! 119 신고, 소화기 발사 (로그만)")
        self.get_logger().info("[FIRE] 진압 시도 후 순찰 복귀 (로그만).")
        self.fire_protocol_running = False

def main():
    rclpy.init()
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
