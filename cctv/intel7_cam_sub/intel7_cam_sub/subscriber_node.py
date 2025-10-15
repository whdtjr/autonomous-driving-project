import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class DetSubscriber(Node):
    def __init__(self):
        super().__init__('det_subscriber')
        self.sub = self.create_subscription(String, 'detections_json', self.cb, 10)
        self.get_logger().info('Subscribed to /detections_json')

    def cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            dets = data.get('dets', [])
            self.get_logger().info(f'Received {len(dets)} dets')
        except Exception as e:
            self.get_logger().warn(f'Parse error: {e}')

def main():
    rclpy.init()
    node = DetSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
