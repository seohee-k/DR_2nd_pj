import rclpy
from rclpy.node import Node
from yoloinference.msg import BoundingBoxes, BoundingBox

class FakeYOLOPublisher(Node):
    def __init__(self):
        super().__init__('fake_yolo_publisher')
        # 퍼블리셔: 실제 YOLO 노드가 publish하는 토픽과 동일하게 맞춰야 함
        self.pub = self.create_publisher(BoundingBoxes, '/yoloinference', 10)
        # 5초마다 fire 객체를 publish
        self.timer = self.create_timer(5.0, self.publish_fire_box)
        self.get_logger().info('Fake YOLO Publisher Node Started!')

    def publish_fire_box(self):
        """
        테스트용: fire 객체가 감지된 것처럼 BoundingBoxes 메시지 publish
        """
        boxes = BoundingBoxes()
        box = BoundingBox()
        box.class_name = 'fire'
        box.x1 = 100.0
        box.y1 = 100.0
        box.x2 = 200.0
        box.y2 = 200.0
        boxes.boxes.append(box)
        self.pub.publish(boxes)
        self.get_logger().info('Published fake fire bounding box!')

def main():
    rclpy.init()
    node = FakeYOLOPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()