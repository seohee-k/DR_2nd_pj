import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import CompressedImage,Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from std_msgs.msg import Header

from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener

import cv2
from cv_bridge import CvBridge

import numpy as np
import threading
import math

from ultralytics import YOLO

from my_interface.msg import YoloInfo

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.rgb_image = None
        self.gui_thread_stop = threading.Event()

        ns = self.get_namespace()
        # rgb topic 
        self.rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed'
        # depth topic
        self.depth_topic = f'{ns}/oakd/stereo/image_raw/compressedDepth'
        # camera info topic
        self.info_topic = f'{ns}/oakd/rgb/camera_info'
        
        self.depth_image=None
        self.rgb_image=None

        self.last_x = None
        self.last_y = None
        self.epsilon = 0.05  # 허용 오차 (단위: meter)

        # call back - 카메라 로깅 플래그
        self.logged_intrinsics = False
        self.logged_rgb_shape = False
        self.logged_depth_shape = False

        # transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #
        self.processing = False
        self.frame_count = 0  # __init__에서 선언
        self.process_every_n = 3  # 3프레임 중 1개만 처리


        # Load YOLOv8 model
        self.model = YOLO("/home/djqsp2/rokey3_E4_ws/src/best.pt")

        self.create_subscription(CompressedImage,
                                 self.rgb_topic, 
                                 self.rgb_callback,
                                 10)
        self.create_subscription(Image, 
                                 self.depth_topic, 
                                 self.depth_callback, 
                                 10)
        self.create_subscription(CameraInfo, 
                                 self.info_topic, 
                                 self.camera_info_callback, 
                                 10)

        self.start_timer = self.create_timer(5.0, self.start_transform)

        # 토픽 발행
        self.target_pub = self.create_publisher(YoloInfo, '/detected_target', 10)

        # self.gui_thread = threading.Thread(target=self.gui_loop, daemon=True)
        # self.gui_thread.start()

    def start_transform(self):
        self.get_logger().info("TF Tree \uc548\uc815\ud654 \uc644\ub8cc. \ubcc0\ud658 \uc2dc\uc791\ud569\ub2c8\ub2e4.")
        # self.timer = self.create_timer(0.5, self.cal_target_point)

        #yolo 추론을 비동기로 실행
        self.timer = self.create_timer(1, self.enqueue_inference)
        self.start_timer.cancel()

    def enqueue_inference(self):
        if not self.processing:
            threading.Thread(target=self.cal_target_point, daemon=True).start()
        # self.frame_count += 1
        # if self.frame_count % self.process_every_n != 0:
        #     return  # 스킵
        # if not self.processing:
        #     threading.Thread(target=self.cal_target_point, daemon=True).start()

    def rgb_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if rgb is not None and rgb.size > 0:
                if not self.logged_rgb_shape:
                    self.get_logger().info(f"RGB image decoded: {rgb.shape}")
                    self.logged_rgb_shape = True
                with self.lock:
                    self.rgb_image = rgb
        except Exception as e:
            self.get_logger().error(f"Compressed RGB decode failed: {e}")

    def camera_info_callback(self, msg):
        with self.lock:
            self.K = np.array(msg.k).reshape(3, 3)
            if not self.logged_intrinsics:
                self.get_logger().info(
                    f"Camera intrinsics received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}"
                )
                self.logged_intrinsics = True

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if depth is not None and depth.size > 0:
                if not self.logged_depth_shape:
                    self.get_logger().info(f"Depth image received: {depth.shape}")
                    self.logged_depth_shape = True
                with self.lock:
                    self.depth_image = depth
                    self.camera_frame = msg.header.frame_id
        except Exception as e:
            self.get_logger().error(f"Depth CV bridge conversion failed: {e}")
    
    def cal_target_point(self):

        self.processing = True

        with self.lock:
            rgb = self.rgb_image.copy() if self.rgb_image is not None else None
            depth = self.depth_image.copy() if self.depth_image is not None else None
            frame_id = getattr(self, 'camera_frame', None)
            

        if rgb is not None and depth is not None and frame_id:
            # results = self.model(self.rgb_image, verbose=False)[0]
            results = self.model(rgb, verbose=False)[0]
            try:
                for det in results.boxes:
                        cls = int(det.cls[0])
                        label = self.model.names[cls]
                        conf = float(det.conf[0])
                        x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())

                        cv2.rectangle(rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(rgb, f"{label} {conf:.2f}", (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                        if label.lower() == "employee" and conf >= 0.2:

                            u = int((x1 + x2) // 2)
                            v = int((y1 + y2) // 2)
                            z = float(depth[v, u]) / 1000.0

                            fx, fy = self.K[0, 0], self.K[1, 1]
                            cx, cy = self.K[0, 2], self.K[1, 2]
                            x = (u - cx) * z / fx
                            y = (v - cy) * z / fy

                            pt = PointStamped()
                            pt.header.frame_id = self.camera_frame
                            pt.header.stamp = rclpy.time.Time().to_msg()
                            pt.point.x, pt.point.y, pt.point.z = x, y, z
                            pt_map = self.tf_buffer.transform(pt, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
                            # logger
                            self.get_logger().info(f"Detected: {label} at x: {pt_map.point.x},y: {pt_map.point.y}")

                            # YoloInfo pub
                            msg = YoloInfo()
                            msg.header = Header()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.header.frame_id = "map"
                            msg.class_name = label
                            msg.confidence = conf
                            msg.x = pt_map.point.x
                            msg.y = pt_map.point.y
                            # self.target_pub.publish(msg)

                            # 이전 값과 비교하여 변화가 있으면 publish
                            if (
                                self.last_x is None or
                                self.last_y is None or
                                abs(msg.x - self.last_x) > self.epsilon or
                                abs(msg.y - self.last_y) > self.epsilon
                            ):
                                self.target_pub.publish(msg)
                                self.last_x = msg.x
                                self.last_y = msg.y
                                self.get_logger().info(f"Publish: ({msg.x:.2f}, {msg.y:.2f})")
                            else:
                                self.get_logger().info("위치 변화 없음: publish 생략")

            except Exception as e:
                self.get_logger().warn(f"TF or goal error: {e}")
            finally:
                    self.processing = False

    # def gui_loop(self):
    #     cv2.namedWindow('RGB Viewer', cv2.WINDOW_NORMAL)
    #     cv2.resizeWindow('RGB Viewer', 640, 480)

    #     while not self.gui_thread_stop.is_set():
    #         with self.lock:
    #             img = self.rgb_image.copy() if self.rgb_image is not None else None

    #         if img is not None:
    #             cv2.imshow('RGB Viewer', img)
    #             key = cv2.waitKey(10)
    #             if key == ord('q'):
    #                 self.get_logger().info("Shutdown requested.")
    #                 self.gui_thread_stop.set()
    #                 rclpy.shutdown()
    #         else:
    #             cv2.waitKey(10)

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # node.gui_thread_stop.set()
    # node.gui_thread.join()

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
