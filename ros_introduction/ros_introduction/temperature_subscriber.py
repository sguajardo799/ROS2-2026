import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class TemperatureSubscriber(Node):
    """
    Recibe temperaturas y genera mensajes según el valor medido.

    Situación representativa:
    Un nodo supervisor que recibe datos de sensores y detecta condiciones
    potencialmente anómalas.
    """

    def __init__(self):
        super().__init__('temperature_subscriber')

        self.subscription = self.create_subscription(
            Float32,
            '/sensor/temperature',
            self.temperature_callback,
            10
        )

        self.get_logger().info(
            'Suscriptor escuchando /sensor/temperature'
        )

    def temperature_callback(self, msg):
        temperature = msg.data

        if temperature >= 30.0:
            self.get_logger().warning(
                f'Temperatura alta: {temperature:.2f} °C'
            )
        else:
            self.get_logger().info(
                f'Temperatura recibida: {temperature:.2f} °C'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
