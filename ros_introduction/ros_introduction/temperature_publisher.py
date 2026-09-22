import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TemperaturePublisher(Node):
	def __init__(self):
		super().__init__("publicador_temperatura")
		self.publisher = self.create_publisher(
			Float32,
			'/sensor/temperature',
			10
			)
		self.timer = self.create_timer(1.0, self.publish_temp)

		self.temperature = 25.0

		self.get_logger().info("Publicador iniciado")

	def publish_temp(self):
		self.temperature += random.uniform(-0.5, 0.5)

		msg = Float32()
		msg.data = float(self.temperature)

		self.publisher.publish(msg)

		self.get_logger().info(f"El valor medido y enviado es: {msg.data:.2f} °C")

def main(args=None):
	rclpy.init(args=args)
	node = TemperaturePublisher()

	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
