#!/usr/bin/env python3
"""Prueba de escalones de velocidad (Lab 2 y Lab 3).

Publica una secuencia de setpoints a 20 Hz, registra la respuesta y por cada
escalón muestra: tiempo de subida (10-90 %), sobreimpulso y error estacionario.
Guarda todo en step_test.csv para graficarlo.

Uso:
    python3 step_test.py                       # Lab 2: /motor_setpoint, /motor_rpm
    python3 step_test.py --ros-args -r __ns:=/robot1 \\
        -p setpoint_topic:=motorR_speed -p feedback_topic:=motorR_feedback   # Lab 3
"""
import csv

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32

STEPS = [(0.0, 2.0), (60.0, 3.0), (120.0, 3.0), (180.0, 3.0), (-120.0, 3.0), (0.0, 2.0)]


class StepTest(Node):

    def __init__(self):
        super().__init__('step_test')
        sp_topic = self.declare_parameter('setpoint_topic', 'motor_setpoint').value
        fb_topic = self.declare_parameter('feedback_topic', 'motor_rpm').value
        self.pub = self.create_publisher(Float32, sp_topic, 10)
        # sensor_data = best-effort: hace match con publishers reliable (Lab 2)
        # y best-effort (Lab 3).
        self.create_subscription(Float32, fb_topic, self.on_rpm, qos_profile_sensor_data)
        self.i = 0
        self.t0 = self.now()
        self.t_step = self.t0
        self.log = []  # (t, setpoint, rpm, índice de escalón)
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_rpm(self, msg):
        if self.i < len(STEPS):
            self.log.append((self.now() - self.t0, STEPS[self.i][0], msg.data, self.i))

    def tick(self):
        if self.now() - self.t_step > STEPS[self.i][1]:
            self.i += 1
            self.t_step = self.now()
            if self.i >= len(STEPS):
                self.pub.publish(Float32(data=0.0))
                raise SystemExit
            self.get_logger().info(f'setpoint = {STEPS[self.i][0]:.0f} RPM')
        self.pub.publish(Float32(data=STEPS[self.i][0]))


def metrics(log):
    print(f'\n{"escalón":>16} {"t_subida[s]":>12} {"sobreimp[%]":>12} {"e_ss[RPM]":>10}')
    for k in range(1, len(STEPS)):
        rows = [r for r in log if r[3] == k]
        if not rows:
            continue
        prev = [r for r in log if r[3] == k - 1]
        y0 = prev[-1][2] if prev else 0.0  # velocidad al inicio del escalón
        sp, t_start = rows[0][1], rows[0][0]
        span = sp - y0
        tail = [r[2] for r in rows if r[0] > rows[-1][0] - 1.0]  # último segundo
        e_ss = sp - sum(tail) / len(tail)
        rise, over = float('nan'), 0.0
        if abs(span) > 1e-3:
            norm = [((r[2] - y0) / span, r[0]) for r in rows]
            t10 = next((t for v, t in norm if v >= 0.1), None)
            t90 = next((t for v, t in norm if v >= 0.9), None)
            if t10 is not None and t90 is not None:
                rise = t90 - t10
            over = max(0.0, (max(v for v, _ in norm) - 1.0) * 100.0)
        label = f'{y0:6.0f} -> {sp:4.0f}'
        print(f'{label:>16} {rise:12.3f} {over:12.1f} {e_ss:10.1f}  (inicio t={t_start:.1f}s)')


def main():
    rclpy.init()
    node = StepTest()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    try:
        node.pub.publish(Float32(data=0.0))
    except Exception:
        pass
    with open('step_test.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'setpoint', 'rpm', 'step'])
        w.writerows(node.log)
    metrics(node.log)
    print('\nDatos guardados en step_test.csv')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
