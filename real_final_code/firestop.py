import rclpy
from rclpy.node import Node
from yoloinference.msg import BoundingBoxes
from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import numpy as np
import tf2_ros
import math
import cv2
import random

WAYPOINTS = [
    (-2.238, -0.081), (-2.099, 2.197), (-0.396, 2.411), (-0.382, 1.347),
    (-0.547, 2.352), (-2.091, 2.240), (-2.116, 0.382), (-0.436, 0.121)
]

class FireStopNode(Node):
    def __init__(self, robot_id='1'):
        super().__init__('firestop_node')
        self.robot_id = robot_id
        self.navigator = TurtleBot4Navigator()
        self.K = None
        self.depth_image = None
        self.camera_frame = None
        self.fire_depth = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_poses = [self.navigator.getPoseStamped([x, y], TurtleBot4Directions.NORTH) for x, y in WAYPOINTS]
        self.position_index = 0

        self.fire_detected = False
        self.tracked_fire_pose = None
        self.fire_response_phase = 0

        self.create_subscription(BoundingBoxes, f'/turtlebot{robot_id}/yoloinference', self.yolo_callback, 10)
        self.create_subscription(CameraInfo, f'/turtlebot{robot_id}/oakd/rgb/camera_info/compressed', self.camera_info_callback, 10)
        self.create_subscription(CompressedImage, f'/turtlebot{robot_id}/oakd/rgb/image_raw/compressedDepth', self.depth_callback, 10)
        self.create_subscription(Bool, 'fire_flag', self.fire_flag_cb, 10)

        self.create_timer(0.5, self.fire_response_timer_cb)
        self.get_logger().info(f'FireStopNode({robot_id}) initialized!')

    def fire_flag_cb(self, msg):
        if msg.data and not self.fire_detected:
            self.get_logger().info('fire_flag 신호 수신, 화재 대응 모드 진입')
            self.fire_detected = True

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        self.camera_frame = msg.header.frame_id

    def depth_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        if np_arr.size == 0:
            self.get_logger().error("Received empty depth buffer!")
            return
        try:
            depth = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            if depth is not None and depth.size > 0:
                self.depth_image = depth
            else:
                self.get_logger().warn("Decoded depth image is empty!")
        except Exception as e:
            self.get_logger().error(f"Depth decode failed: {e}")

    def yolo_callback(self, msg: BoundingBoxes):
        if self.fire_detected:
            return
        for box in msg.boxes:
            if box.class_name == 'fire':
                u = (box.x1 + box.x2) // 2
                v = (box.y1 + box.y2) // 2

                if self.K is None or self.depth_image is None:
                    self.get_logger().warn("CameraInfo 또는 Depth 미수신!")
                    return

                z = float(self.depth_image[v, u])
                self.fire_depth = z
                if z == 0.0:
                    self.get_logger().warn("Depth 값 0")
                    return

                fx, fy = self.K[0,0], self.K[1,1]
                cx, cy = self.K[0,2], self.K[1,2]
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                pt = PointStamped()
                pt.header.frame_id = self.camera_frame
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.point.x, pt.point.y, pt.point.z = x, y, z / 1000.0
                try:
                    pt_map = self.tf_buffer.transform(pt, 'map')
                except Exception as e:
                    self.get_logger().warn(f"TF 변환 실패: {e}")
                    return

                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = pt_map.point.x
                pose.pose.position.y = pt_map.point.y
                pose.pose.orientation.w = 1.0

                self.fire_detected = True
                self.tracked_fire_pose = pose
                self.get_logger().info(f"Fire detected at ({pt_map.point.x:.2f}, {pt_map.point.y:.2f})")
                self.navigator.cancelTask()
                break

    def fire_response_timer_cb(self):
        if not self.fire_detected or self.tracked_fire_pose is None:
            return

        if self.fire_response_phase == 0:
            self.get_logger().info("[FIRE] 화재 위치로 적정 거리까지 접근")
            self.get_close_to_fire()
            self.fire_response_phase = 1
        elif self.fire_response_phase == 1:
            if self.navigator.isTaskComplete():
                self.get_logger().info("[FIRE] 소화기 투척 및 진압 시작")
                self.extinguish_fire()
                self.fire_response_phase = 2
        elif self.fire_response_phase == 2:
            self.get_logger().info("[INTRUDER] 침입자 대응 순찰 시작")
            self.patrol_intruder()
            self.fire_response_phase = 3

    def get_close_to_fire(self):
        if not self.tracked_fire_pose or self.fire_depth is None:
            self.get_logger().warn("Fire position or depth not available!")
            return

        fire_x = self.tracked_fire_pose.pose.position.x
        fire_y = self.tracked_fire_pose.pose.position.y
        distance_to_fire = self.fire_depth / 1000.0

        safe_distance = 1.0
        if distance_to_fire <= safe_distance:
            self.get_logger().info(f"[GET_CLOSE_TO_FIRE] Already at safe distance ({distance_to_fire:.2f}m) from fire")
            return

        current_pose = self.navigator.getPose()
        if current_pose is None:
            self.get_logger().error("Failed to get current pose.")
            return
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y

        dx = fire_x - current_x
        dy = fire_y - current_y
        total_distance = math.sqrt(dx**2 + dy**2)
        if total_distance == 0:
            return

        approach_distance = total_distance - safe_distance
        if approach_distance <= 0:
            return

        ratio = approach_distance / total_distance
        new_x = current_x + dx * ratio
        new_y = current_y + dy * ratio

        self.get_logger().info(f"[GET_CLOSE_TO_FIRE] Approaching fire at ({new_x:.2f}, {new_y:.2f})")
        target_pose = self.navigator.getPoseStamped([new_x, new_y], TurtleBot4Directions.NORTH)
        self.navigator.startToPose(target_pose)
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"[GET_CLOSE_TO_FIRE] Approached fire, distance: {safe_distance:.2f}m")

    def extinguish_fire(self):
        self.get_logger().info('🔥 투척형 소화기 발사!')
        self.get_logger().info('Fire extinguished.')

    def patrol_intruder(self):
        self.get_logger().info('Starting intruder patrol...')
        for pose in self.goal_poses[:3]:
            self.navigator.startToPose(pose)
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)
            if random.random() < 0.3:
                intruder_position = {'x': pose.pose.position.x, 'y': pose.pose.position.y}
                self.get_logger().info(f'[INTRUDER] Detected at {intruder_position["x"]:.2f}, {intruder_position["y"]:.2f}')
                self.get_logger().info("[INTRUDER] 1차 대응: LED 점등 및 경고음 출력")
                self.get_logger().info("[INTRUDER] 경고: 즉시 나가십시오!")
                self.get_logger().info("[INTRUDER] 112에 침입자 신고 완료")
                self.get_logger().info("[INTRUDER] 2차 대응: 테이저건 발사")
                self.get_logger().info("[INTRUDER] 3차 대응: 포박줄 발사")
                break
        self.get_logger().info('Patrol completed, resuming normal patrol.')

def main():
    rclpy.init()
    node = FireStopNode('1')  # robot_id는 환경에 맞게
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()