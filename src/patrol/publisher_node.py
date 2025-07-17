import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.bridge = CvBridge()
        # 실제 카메라 토픽 구독 (RGB, Depth)
        self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        # YOLO bbox 결과 퍼블리셔 (테스트용)
        self.bbox_pub = self.create_publisher(String, '/yolo_detection', 10)
        self.timer = self.create_timer(1.0, self.publish_dummy_bbox)

    def rgb_callback(self, msg):
        # 실제로는 YOLO 노드에서 처리
        pass

    def depth_callback(self, msg):
        # 실제로는 YOLO 노드에서 처리
        pass

    def publish_dummy_bbox(self):
        # 실제 YOLO 노드가 없을 때만 사용 (테스트용)
        self.bbox_pub.publish(String(data="car,320,240,0.98"))

def main():
    rclpy.init()
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
