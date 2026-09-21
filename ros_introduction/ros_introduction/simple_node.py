import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')

        self.counter = 0
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('Nodo simple iniciado...')

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Conteo: {self.counter}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
