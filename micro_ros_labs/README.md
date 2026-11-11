# Laboratorios micro-ROS: control de velocidad FF + PI en ESP32

Tres laboratorios progresivos para implementar, en un **ESP32**, el control de velocidad de las ruedas de un robot diferencial (**feedforward + PI**) e integrarlo con **ROS 2 Jazzy** mediante **micro-ROS**. Son una versión didáctica del firmware `ROS_botV2`: se mantiene lo esencial del control y de micro-ROS, y se deja fuera la cinemática y la odometría.

```
 Lab 1 ─────────────────────► Lab 2 ─────────────────────► Lab 3
 Primer nodo micro-ROS        Lazo cerrado FF + PI         Dos motores, WiFi, robustez
 motor en lazo abierto        tarea FreeRTOS 100 Hz        watchdog, reconexión, servicio,
 identificación del FF        setpoint en RPM              QoS, tiempo, namespace
 Serial · 1 motor             Serial · 1 motor             WiFi UDP · 2 motores
```

| | [Lab 1](lab1_uros_basico/) | [Lab 2](lab2_control_ff_pi/) | [Lab 3](lab3_uros_robusto/) |
|---|---|---|---|
| **micro-ROS** | allocator, support, nodo, pub/sub, timer, executor | varios publishers, convivencia con una tarea de tiempo real | servicio, QoS best-effort, ping y reconexión, sincronización de tiempo, namespace, mensajes con arreglos |
| **Control** | Lazo abierto (duty), estimación de velocidad | FF + PI, rampa, zona muerta, anti-windup | Igual, por rueda, con watchdog |
| **Transporte** | Serial USB | Serial USB | WiFi UDP |
| **Motores** | 1 | 1 | 2 |
| **Tópicos** | `motor_duty` → / `motor_rpm`, `encoder_count` ← | `motor_setpoint` → / `motor_rpm`, `motor_reference`, `motor_duty` ← | `motorX_speed` → / `motorX_feedback`, `joint_states` ← / srv `reset_motors` |
| **Resultado** | `FF_STATIC` y `MOTOR_MAX_RPM` | Ganancias `Kp`, `Ki` ajustadas | Firmware base del robot |

---

## Librerías y cómo se reutilizan

Arduino no permite incluir archivos de otra carpeta de sketch, así que **cada laboratorio trae su copia** de las librerías. Las que ya existían pasan **sin cambios**; cada lab agrega las suyas:

| Librería | Lab 1 | Lab 2 | Lab 3 | Responsabilidad |
|---|:-:|:-:|:-:|---|
| `encoder.h/.cpp` | ● nuevo | = | = | `QuadratureEncoder`: cuadratura x4 por hardware (PCNT) |
| `motor_driver.h/.cpp` | ● nuevo | = | = | `MotorDriver`: TB6612, duty con signo `[-1, 1]` |
| `speed_estimator.h/.cpp` | ● nuevo | = | = | `SpeedEstimator`: RPM por ventana deslizante con Δt medido |
| `microros_time_fix.cpp` | ● | = | = | Corrección del reloj de `libmicroros.a` |
| `speed_controller.h/.cpp` | | ● nuevo | = | `SpeedController`: FF + PI, rampa, anti-windup (sin hardware) |
| `motor.h/.cpp` | | | ● nuevo | `Motor`: une las 4 anteriores + watchdog |
| `ros_interface.h/.cpp` | | | ● nuevo | Entidades micro-ROS y máquina de estados del agente |
| `config.h` | ● | ampliado | ampliado | Todos los parámetros del robot |

Para comprobar que una librería no cambió: `diff lab1_uros_basico/encoder.cpp lab3_uros_robusto/encoder.cpp`.

---

## Requisitos

| | |
|---|---|
| Placa | ESP32 clásico (ESP32-WROOM / DevKit), **ESP32 Dev Module** en el IDE |
| Driver | TB6612FNG con `STBY` en HIGH |
| Motores | Reductora 50:1, encoder 7 PPR (1400 cuentas por vuelta de rueda) |
| Arduino core ESP32 | **3.x** (probado con 3.3.12) |
| Librería | [`micro_ros_arduino`](https://github.com/micro-ROS/micro_ros_arduino) **2.0.8-jazzy** |
| PC | ROS 2 Jazzy + agente micro-ROS (Docker `microros/micro-ros-agent:jazzy` o compilado) |

### Parche de micro_ros_arduino (obligatorio)

`libmicroros.a` viene precompilada con ESP-IDF 4.4 (`time_t` de 32 bits), pero el core ESP32 3.x usa ESP-IDF 5 (`time_t` de 64 bits). La librería lee mal el reloj y **avanza en saltos de 1 s**: los timers corren a 1 Hz y `spin_some` se bloquea ~1 s.

```bash
tools/patch_microros_time.sh            # una vez, y de nuevo si se reinstala la librería
```

El script redirige `clock_gettime` de la librería hacia `uros_clock_gettime`, definida en `microros_time_fix.cpp` (incluido en cada laboratorio). Es idempotente y deja un respaldo `libmicroros.a.orig`.

---

## El agente micro-ROS

El ESP32 no ejecuta DDS. El **agente** (en el PC) crea en DDS las entidades que pide el ESP32 y reenvía los mensajes. Sin agente, el nodo no existe en el grafo ROS 2.

| Transporte | Labs | Comando |
|---|---|---|
| Serial | 1, 2 | `docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0 -b 115200 -v4` |
| UDP | 3 | `docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v4` |

Con un workspace propio: `ros2 run micro_ros_agent micro_ros_agent <serial|udp4> ...` con los mismos argumentos.

---

## Mapa de conceptos: rclpy ↔ rclc

Los nodos de `ros_introduction` (rclpy) y los de estos laboratorios (rclc) usan los mismos conceptos:

| Concepto ROS 2 | rclpy (`ros_introduction`) | rclc (ESP32) | Lab |
|---|---|---|---|
| Inicialización | `rclpy.init()` | `allocator` + `rclc_support_init()` | 1 |
| Nodo | `Node("nombre")` | `rclc_node_init_default()` | 1 |
| Tópico (pub) | `create_publisher()` + `publish()` | `rclc_publisher_init_*()` + `rcl_publish()` | 1 |
| Tópico (sub) | `create_subscription(cb)` | `rclc_subscription_init_*()` + `rclc_executor_add_subscription(cb)` | 1 |
| Timer | `create_timer(cb)` | `rclc_timer_init_default2()` + `rclc_executor_add_timer()` | 1 |
| Spin | `rclpy.spin(node)` | `rclc_executor_spin_some()` en `loop()` | 1 |
| Servicio | `create_service(cb)` | `rclc_service_init_default()` + `rclc_executor_add_service(cb)` | 3 |
| QoS | `qos_profile_sensor_data` | `rclc_*_init_best_effort()` | 3 |
| Namespace | `--ros-args -r __ns:=/robot1` | 3er argumento de `rclc_node_init_default()` | 3 |
| Fin | `destroy_node()`, `shutdown()` | `rcl_*_fini()`, `rclc_support_fini()` | 3 |

---

## Hardware por defecto

| Función | Motor derecho (Labs 1–3) | Motor izquierdo (Lab 3) |
|---|---|---|
| PWM | 19 | 27 |
| IN1 / IN2 | 15 / 26 | 32 / 33 |
| Encoder A / B | 25 / 14 | 34 / 13 |

> 🛑 Haz siempre las primeras pruebas **con las ruedas en el aire**.

---

## Estructura

```
micro_ros_labs/
├── README.md
├── tools/patch_microros_time.sh
├── lab1_uros_basico/      README · .ino · config · encoder · motor_driver · speed_estimator · tools/duty_sweep.py
├── lab2_control_ff_pi/    README · .ino · config · (Lab 1) + speed_controller · tools/step_test.py
└── lab3_uros_robusto/     README · .ino · config · secrets.h.example · (Lab 2) + motor · ros_interface · tools/step_test.py
```

Compilación verificada con `arduino-cli` 1.5.1, core esp32 3.3.12 y micro_ros_arduino 2.0.8-jazzy (parchada).
