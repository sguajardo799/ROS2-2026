#!/usr/bin/env python3
"""Barrido de duty en lazo abierto (Lab 1).

Publica escalones de duty en /motor_duty, mide la velocidad estacionaria en
/motor_rpm y ajusta la recta que usará el feedforward del Lab 2:

    duty = FF_STATIC + rpm / MOTOR_MAX_RPM

Uso (con el agente corriendo y la rueda EN EL AIRE):
    python3 duty_sweep.py
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

DUTIES = [round(0.05 * i, 2) for i in range(1, 21)]  # 0.05, 0.10, ..., 1.00
STEP_S = 2.0       # duración de cada escalón [s]
SETTLE_S = 1.0     # se descarta el primer segundo (transitorio)
MOVING_RPM = 3.0   # bajo este valor la rueda se considera detenida


class DutySweep(Node):

    def __init__(self):
        super().__init__('duty_sweep')
        self.pub = self.create_publisher(Float32, 'motor_duty', 10)
        self.create_subscription(Float32, 'motor_rpm', self.on_rpm, 10)
        self.step = 0
        self.t_step = self.now()
        self.samples = []
        self.results = []  # (duty, rpm promedio)
        self.create_timer(0.05, self.tick)  # 20 Hz
        self.get_logger().info('Barrido iniciado. Rueda en el aire.')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_rpm(self, msg):
        if self.step < len(DUTIES) and self.now() - self.t_step > SETTLE_S:
            self.samples.append(msg.data)

    def tick(self):
        if self.step >= len(DUTIES):
            return
        if self.now() - self.t_step > STEP_S:
            duty = DUTIES[self.step]
            rpm = sum(self.samples) / len(self.samples) if self.samples else 0.0
            self.results.append((duty, rpm))
            self.get_logger().info(f'duty={duty:4.2f}  rpm={rpm:7.1f}')
            self.step += 1
            self.samples = []
            self.t_step = self.now()
            if self.step >= len(DUTIES):
                self.pub.publish(Float32(data=0.0))
                raise SystemExit
        self.pub.publish(Float32(data=DUTIES[self.step]))


def fit(results):
    """Mínimos cuadrados de duty = a + b * rpm usando sólo los puntos en movimiento."""
    pts = [(d, r) for d, r in results if r > MOVING_RPM]
    if len(pts) < 2:
        print('Muy pocos puntos con la rueda en movimiento: revisa el cableado y ENC_DIR.')
        return
    n = len(pts)
    mr = sum(r for _, r in pts) / n
    md = sum(d for d, _ in pts) / n
    b = sum((r - mr) * (d - md) for d, r in pts) / sum((r - mr) ** 2 for _, r in pts)
    a = md - b * mr
    first_moving = min(d for d, _ in pts)
    print('\n===== Resultado (copiar a config.h del Lab 2) =====')
    print(f'Duty mínimo que movió la rueda: {first_moving:.2f}')
    print(f'#define FF_STATIC      {a:.3f}f')
    print(f'#define MOTOR_MAX_RPM  {1.0 / b:.1f}f')
    print(f'RPM medida con duty 1.0: {results[-1][1]:.1f}')


def main():
    rclpy.init()
    node = DutySweep()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    try:
        node.pub.publish(Float32(data=0.0))  # dejar el motor detenido
    except Exception:
        pass  # tras Ctrl+C el contexto ya está cerrado: publica 0 a mano
    fit(node.results)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
