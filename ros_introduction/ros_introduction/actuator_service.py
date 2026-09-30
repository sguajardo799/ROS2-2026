import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class ActuatorService(Node):
    """
    Servicio que habilita o deshabilita un actuador.

    Situación representativa:
    Un nodo encargado de recibir comandos discretos para habilitar o detener
    un motor, bomba, relé u otro actuador.
    """

    def __init__(self):
        super().__init__('actuator_service')

        self.actuator_enabled = False

        self.service = self.create_service(
            SetBool,
            '/actuator/enable',
            self.handle_enable_request
        )

        self.get_logger().info(
            'Servidor disponible en /actuator/enable'
        )

    def handle_enable_request(self, request, response):
        self.actuator_enabled = request.data

        if self.actuator_enabled:
            response.success = True
            response.message = 'Actuador habilitado.'
        else:
            response.success = True
            response.message = 'Actuador deshabilitado.'

        self.get_logger().info(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
