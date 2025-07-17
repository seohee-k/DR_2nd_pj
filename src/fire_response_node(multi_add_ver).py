#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from yoloinference.msg import BoundingBoxes
from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped, PoseStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import numpy as np
import tf2_ros
import math
import random
import cv2
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

class FireResponseNode(Node):
    def __init__(self, robot_id):
        super().__init__(f'fire_response_node_{robot_id}')

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

        self.patrol_active = True
        self.fire_detected = False
        self.goal_in_progress = False
        self.tracked_fire_pose = None
        self.fire_response_phase = 0

        self.create_subscription(BoundingBoxes, f'/turtlebot{robot_id}/yoloinference', self.yolo_callback, 10)
        self.create_subscription(CameraInfo, f'/turtlebot{robot_id}/oakd/rgb/camera_info/compressed', self.camera_info_callback, 10)
        self.create_subscription(CompressedImage, f'/turtlebot{robot_id}/oakd/rgb/image_raw/compressedDepth', self.depth_callback, 10)

        self.create_timer(0.5, self.patrol_timer_cb)
        self.get_logger().info(f'[BATTERY] 현재 잔량: 90.0% - 순찰 시작! (TurtleBot4-{robot_id})')

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

    def patrol_timer_cb(self):
        if self.fire_detected:
            if self.fire_response_phase == 0 and self.tracked_fire_pose is not None:
                self.get_close_to_fire()
                self.get_logger().info(f"[NAV] 화재 추적, 좌표=({self.tracked_fire_pose.pose.position.x:.2f},{self.tracked_fire_pose.pose.position.y:.2f})")
                self.navigator.startToPose(self.tracked_fire_pose)
                self.fire_response_phase = 1
            elif self.fire_response_phase == 1:
                if self.navigator.isTaskComplete():
                    self.get_logger().info("[PROTOCOL] 화재 도착, 진압 시작")
                    self.extinguish_fire()
                    self.fire_response_phase = 2
            elif self.fire_response_phase == 2:
                self.get_logger().info("[PROTOCOL] 화재 진압 완료, 침입자 순찰 시작")
                self.patrol_intruder()
                self.fire_response_phase = 3
            return

        if self.patrol_active and not self.goal_in_progress:
            self.get_logger().info(f'[PATROL] {self.position_index+1}/{len(self.goal_poses)}번 웨이포인트로 이동')
            pose = self.goal_poses[self.position_index]
            self.navigator.startToPose(pose)
            self.goal_in_progress = True
        elif self.patrol_active and self.goal_in_progress:
            if self.navigator.isTaskComplete():
                self.position_index = (self.position_index + 1) % len(self.goal_poses)
                self.goal_in_progress = False

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
                self.patrol_active = False
                self.goal_in_progress = False
                self.tracked_fire_pose = pose
                self.get_logger().info(f"Fire detected at ({pt_map.point.x:.2f}, {pt_map.point.y:.2f})")
                self.navigator.cancelTask()
                break

    def extinguish_fire(self):
        self.get_logger().info('Attempting to extinguish fire...')
        self.get_logger().info('Fire extinguished.')

def main(args=None):
    rclpy.init(args=args)
    try:
        node1 = FireResponseNode('1')
        node2 = FireResponseNode('2')

        executor = MultiThreadedExecutor()
        executor.add_node(node1)
        executor.add_node(node2)

        print(f"Starting FireResponseNodes at {time.strftime('%H:%M:%S', time.localtime())} on June 03, 2025")
        executor.spin()

    except KeyboardInterrupt:
        print("KeyboardInterrupt received, shutting down...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()
        print("FireResponseNodes shutdown complete.")

if __name__ == '__main__':
    main()