import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class ActuatorClient(Node):
    """
    Cliente del servicio /actuator/enable.

    Uso:
        ros2 run ros_introduction actuator_client true
        ros2 run ros_introduction actuator_client false
    """

    def __init__(self):
        super().__init__('actuator_client')

        self.client = self.create_client(
            SetBool,
            '/actuator/enable'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Esperando al servicio /actuator/enable...'
            )

    def send_request(self, enable):
        request = SetBool.Request()
        request.data = enable

        return self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 2 or sys.argv[1].lower() not in ('true', 'false'):
        print(
            'Uso: ros2 run ros_introduction actuator_client [true|false]'
        )
        rclpy.shutdown()
        return

    enable = sys.argv[1].lower() == 'true'

    node = ActuatorClient()
    future = node.send_request(enable)

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        response = future.result()
        node.get_logger().info(
            f'Respuesta: success={response.success}, '
            f'message="{response.message}"'
        )
    else:
        node.get_logger().error('No fue posible obtener respuesta.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
