# publisher_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from yoloinference.msg import BoundingBoxes, BoundingBox
import cv2
import numpy as np
from ultralytics import YOLO

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.logged_rgb_shape = False
        self.logged_depth_shape = False

        # YOLO 모델 로드
        self.yolo_model = YOLO('/home/rokey/patrol_ws/src/my_patrol/my_patrol/best.pt')

        # RGB CompressedImage 구독
        self.create_subscription(
            CompressedImage,
            '/robot7/oakd/rgb/image_raw/compressed',
            self.rgb_callback,
            10)
        # **Depth CompressedImage 토픽을 rgb 네임스페이스로 변경**
        self.create_subscription(
            CompressedImage,
            '/robot7/oakd/rgb/image_raw/compressedDepth',
            self.depth_callback,
            10)

        self.bbox_pub = self.create_publisher(BoundingBoxes, '/yoloinference', 10)
        self.depth_image = None

    def rgb_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if rgb is not None and rgb.size > 0:
                if not self.logged_rgb_shape:
                    self.get_logger().info(f"RGB image decoded: {rgb.shape}")
                    self.logged_rgb_shape = True

                # YOLO 추론
                yolo_results = self.yolo_model(rgb)
                boxes = []
                for r in yolo_results:
                    for box in r.boxes:
                        bb = BoundingBox()
                        cid = int(box.cls[0])
                        bb.class_name = self.yolo_model.names[cid]
                        bb.x1, bb.y1, bb.x2, bb.y2 = map(int, box.xyxy[0])
                        boxes.append(bb)
                        # 시각화 (옵션)
                        cv2.rectangle(rgb, (bb.x1, bb.y1), (bb.x2, bb.y2), (0, 255, 0), 2)
                        cv2.putText(rgb, bb.class_name, (bb.x1, bb.y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                # BoundingBoxes 퍼블리시
                out = BoundingBoxes()
                out.boxes = boxes
                self.bbox_pub.publish(out)

                # GUI (옵션)
                cv2.imshow("YOLO Detection", rgb)
                cv2.waitKey(1)
            else:
                self.get_logger().warn("Decoded RGB image is None or empty!")
        except Exception as e:
            self.get_logger().error(f"RGB decode or YOLO failed: {e}")

    def depth_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            depth = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            if depth is not None and depth.size > 0:
                if not self.logged_depth_shape:
                    self.get_logger().info(f"Depth image decoded: {depth.shape}, dtype={depth.dtype}")
                    self.logged_depth_shape = True
                self.depth_image = depth
            else:
                self.get_logger().warn("Decoded depth image is None or empty!")
        except Exception as e:
            self.get_logger().error(f"Depth decode failed: {e}")

def main():
    rclpy.init()
    node = PublisherNode()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
