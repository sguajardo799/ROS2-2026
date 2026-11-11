# Laboratorio 1: primer nodo micro-ROS (motor en lazo abierto)

**Objetivo:** convertir el ESP32 en un **nodo ROS 2** que recibe un duty de PWM por un tópico y publica la velocidad medida por el encoder. Al terminar tendrás la curva *duty → RPM* del motor, que el Lab 2 usará como **feedforward**.

| Ítem | Valor |
|---|---|
| Transporte | **Serial USB** (115200 baudios) |
| Motores | 1 (canal A del TB6612, motor derecho del robot) |
| Control | Ninguno: **lazo abierto** |
| Librerías nuevas | `encoder`, `motor_driver`, `speed_estimator` |

---

## 1. Conceptos de micro-ROS

### 1.1 ¿Dónde está el nodo?

```
 ┌──────────── ESP32 ────────────┐         ┌──────────────── PC ─────────────────┐
 │  lab1_motor_node (rclc / rcl)  │  USB    │  micro_ros_agent  ◄──DDS──►  ros2 CLI │
 │  cliente Micro XRCE-DDS        │◄──────►│  (traductor)                rqt, rclpy │
 └────────────────────────────────┘ serial  └─────────────────────────────────────┘
```

El ESP32 no tiene memoria suficiente para DDS. Ejecuta un **cliente XRCE-DDS**, que le pide al **agente** del PC que cree en DDS el nodo, los tópicos y los servicios en su nombre. Para el resto del sistema ROS 2, `lab1_motor_node` es un nodo como cualquier otro: aparece en `ros2 node list` y sus tópicos se usan con `ros2 topic`.

> Sin agente no hay nodo: si el agente no corre, el ESP32 espera (LED parpadeando lento).

### 1.2 Las piezas de rclc y su equivalente en rclpy

Compara con `ros_introduction/temperature_publisher.py`:

| rclc (C, ESP32) | rclpy (Python, PC) | ¿Qué es? |
|---|---|---|
| `rcl_allocator_t allocator` | — (Python maneja la memoria) | Define **cómo se reserva memoria** (por defecto `malloc`). Se usa solo durante la inicialización. |
| `rclc_support_t support` + `rclc_support_init()` | `rclpy.init()` | Contexto rcl y reloj: la **sesión con el agente**. Todas las entidades dependen de él. |
| `rcl_node_t` + `rclc_node_init_default()` | `Node("nombre")` | El **nodo**: nombre + namespace. |
| `rclc_publisher_init_default()` | `create_publisher()` | Publicador de un **tópico**. |
| `rclc_subscription_init_default()` | `create_subscription()` | Suscriptor de un **tópico**. |
| `rclc_service_init_default()` (Lab 3) | `create_service()` | Servidor de un **servicio**. |
| `rclc_timer_init_default2()` | `create_timer()` | Callback periódico. |
| `rclc_executor_t` + `rclc_executor_spin_some()` | `rclpy.spin(node)` | El **executor** revisa si llegaron mensajes o vencieron timers y llama a los callbacks. |
| `ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32)` | `from std_msgs.msg import Float32` | Descripción del **tipo** (cómo serializarlo). |

Diferencias importantes respecto de rclpy:

1. **En rclpy, `create_subscription` ya registra el callback. En rclc son dos pasos:** se crea la suscripción y luego se *agrega al executor* junto con el callback y el mensaje donde se copiarán los datos (`rclc_executor_add_subscription`).
2. **El executor tiene un número fijo de *handles*.** Cada suscripción, timer o servicio ocupa uno. Si agregas más de los declarados en `rclc_executor_init(..., N, ...)`, la llamada falla. Los **publishers no son handles**: publicar (`rcl_publish`) se hace directamente, sin pasar por el executor.
3. **Memoria estática.** Los mensajes (`msg_duty`, `msg_rpm`…) son variables globales que se reservan una sola vez. No se crean mensajes nuevos en cada callback, como en Python.
4. **El *spin* lo haces tú:** `loop()` llama a `rclc_executor_spin_some(&executor, timeout)`. Los callbacks corren **dentro** de esa llamada, en la misma tarea que `loop()`.
5. **Los errores no lanzan excepciones:** cada función devuelve `rcl_ret_t`. La macro `RCCHECK` lo revisa y, si algo falla, detiene el motor y hace parpadear el LED rápido.

### 1.3 Orden de inicialización (`setup()`)

```
hardware (motor detenido)
  → set_microros_transports()      transporte: Serial
  → rmw_uros_ping_agent() en bucle  esperar al agente
  → allocator → support → node
  → subscription / publishers / timer
  → executor (2 handles) + add_subscription + add_timer
loop(): rclc_executor_spin_some()
```

---

## 2. Archivos

| Archivo | Descripción |
|---|---|
| `lab1_uros_basico.ino` | Nodo micro-ROS: entidades, callbacks y `loop()` con el *spin* |
| `config.h` | Pines, encoder, PWM y período de publicación |
| `encoder.h/.cpp` | `QuadratureEncoder`: cuadratura x4 por hardware (**PCNT**) |
| `motor_driver.h/.cpp` | `MotorDriver`: TB6612, duty con signo en `[-1, 1]` |
| `speed_estimator.h/.cpp` | `SpeedEstimator`: RPM por diferencia de cuentas con Δt medido |
| `microros_time_fix.cpp` | Corrección del reloj de `libmicroros.a` para el core 3.x ([ver README general](../README.md#parche-de-micro_ros_arduino-obligatorio)) |
| `tools/duty_sweep.py` | Barrido de duty e identificación del feedforward |

Las cuatro librerías pasan **sin cambios** a los laboratorios 2 y 3.

---

## 3. Conexiones

| Señal | GPIO | Notas |
|---|---|---|
| `PWMA` | 19 | |
| `AIN2` → `PIN_IN1` | 15 | IN1/IN2 intercambiados para que duty > 0 sea "adelante" |
| `AIN1` → `PIN_IN2` | 26 | |
| Encoder A | 25 | |
| Encoder B | 14 | |
| `STBY` | 3.3 V | Debe estar en HIGH |

---

## 4. Compilar y cargar

1. Parchear `micro_ros_arduino` (una sola vez): `../tools/patch_microros_time.sh`
2. Placa **ESP32 Dev Module** y abrir `lab1_uros_basico.ino`, o con `arduino-cli`:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 lab1_uros_basico
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/ttyUSB0 lab1_uros_basico
```

> ⚠️ **Cierra el monitor serie.** El puerto USB lo usa el agente; si el monitor está abierto, el agente no puede abrirlo. Por la misma razón el código **no usa `Serial.print()`**: los bytes de texto se mezclarían con el protocolo XRCE y romperían la comunicación.

---

## 5. Agente micro-ROS (serial)

**Docker:**
```bash
docker run -it --rm -v /dev:/dev --privileged --net=host \
  microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0 -b 115200 -v4
```

**Compilado en un workspace** (`micro_ros_setup`):
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200 -v4
```

El orden no importa: el ESP32 espera con `rmw_uros_ping_agent()` hasta que el agente responde. Con `-v4`, el agente muestra `session established` y la creación de cada entidad.

| LED | Estado |
|---|---|
| Parpadeo lento (400 ms) | Esperando al agente |
| Encendido fijo | Nodo creado, conectado |
| Parpadeo rápido (100 ms) | Error (encoder o una llamada `RCCHECK`). Presiona **EN/RESET** |

> Si en tu placa abrir el puerto reinicia el ESP32 (por las líneas DTR/RTS), no pasa nada: vuelve a esperar al agente.

---

## 6. Pruebas desde ROS 2

> 🛑 **Rueda en el aire.**

```bash
ros2 node list                       # /lab1_motor_node
ros2 topic list                      # /motor_duty /motor_rpm /encoder_count
ros2 topic info /motor_duty -v       # QoS reliable
ros2 topic hz /motor_rpm             # ~20 Hz
```

### 6.1 Verificar el encoder (obligatorio)

1. `ros2 topic echo /encoder_count`
2. Gira la rueda **una vuelta a mano** hacia adelante: el contador debe aumentar en **≈ 1400** (28 CPR × 50).
3. Si disminuye, invierte `ENC_DIR` en `config.h`.

### 6.2 Mover el motor (lazo abierto)

```bash
ros2 topic pub --once /motor_duty std_msgs/msg/Float32 "{data: 0.4}"    # 40 % adelante
ros2 topic echo /motor_rpm
ros2 topic pub --once /motor_duty std_msgs/msg/Float32 "{data: -0.4}"   # 40 % atrás
ros2 topic pub --once /motor_duty std_msgs/msg/Float32 "{data: 0.0}"    # detener
```

Con duty positivo, la RPM debe ser positiva. Si el motor gira hacia atrás, intercambia `PIN_IN1`/`PIN_IN2` (o los cables del motor) **y** el signo de `ENC_DIR`.

Graficar:
```bash
ros2 run rqt_plot rqt_plot /motor_rpm/data
```

### 6.3 Identificar el feedforward

```bash
python3 tools/duty_sweep.py
```

El script sube el duty de 0.05 a 1.0 en pasos de 0.05 (2 s cada uno), promedia la RPM y ajusta la recta:

```
duty = FF_STATIC + rpm / MOTOR_MAX_RPM
```

- `FF_STATIC`: duty que se "pierde" venciendo la fricción estática (bajo ese valor la rueda no gira).
- `MOTOR_MAX_RPM`: pendiente de la recta (RPM por unidad de duty).

**Guarda los dos valores**: van en el `config.h` del Lab 2.

---

## 7. Precisiones de implementación

- **Lazo abierto:** el duty no garantiza una velocidad. Con la misma entrada, la RPM cambia con la carga (rueda en el aire o en el suelo), la batería y la fricción. Compruébalo frenando la rueda con la mano. Eso es lo que corrige el PI del Lab 2.
- **Velocidad:** se calcula en el *timer* cada 50 ms, con la diferencia de cuentas y el Δt **medido** con `esp_timer_get_time()`. El timer de rclc no es exacto (depende de cuándo se llama `spin_some`), por eso no se supone Δt = 50 ms. La resolución es de ~0.86 RPM por cuenta.
- **Encoder por hardware (PCNT):** el conteo no depende de la CPU ni de interrupciones, así que no se pierden cuentas aunque micro-ROS esté ocupado.
- **QoS:** se usa `_default` (*reliable*), suficiente por serial. En el Lab 3, con WiFi, se pasa a *best-effort*.
- **Sin protección:** si se corta el agente o el cable, **el motor sigue con el último duty**, y si el agente se reinicia, el ESP32 no se reconecta (hay que presionar RESET). El Lab 3 resuelve ambos problemas con un *watchdog* y una máquina de estados.

---

## 8. Actividades

1. Dibuja el grafo con `rqt_graph` mientras corre `duty_sweep.py`. ¿Qué nodos y tópicos aparecen?
2. Cambia `PUBLISH_PERIOD_MS` a 10 ms. ¿Qué pasa con el ruido de `/motor_rpm`? Explícalo con la resolución por cuenta.
3. Agrega un tercer publisher (por ejemplo `/motor_duty_applied`). ¿Hay que cambiar el número de handles del executor? ¿Por qué?
4. Repite el barrido con la rueda en el suelo. ¿Cambian `FF_STATIC` y `MOTOR_MAX_RPM`?

---

## 9. Solución de problemas

| Síntoma | Causa probable |
|---|---|
| LED parpadeando lento para siempre | Agente apagado, puerto equivocado, monitor serie abierto o baudrate ≠ 115200 |
| LED parpadeando rápido | Falló el PCNT (pines del encoder) o la creación de una entidad |
| `ros2 topic echo` sin datos | Reinicia el daemon: `ros2 daemon stop; ros2 daemon start` |
| `/motor_rpm` a ~1 Hz o con saltos | Falta el parche de `libmicroros.a` |
| RPM siempre 0 al girar a mano | Encoder sin alimentación o pines mal configurados |
| RPM negativa con duty positivo | Signo de `ENC_DIR` o IN1/IN2 invertidos (sección 6.2) |
