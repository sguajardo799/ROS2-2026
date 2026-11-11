# Laboratorio 3: dos motores, micro-ROS por WiFi y operación robusta

**Objetivo:** llevar el control del Lab 2 a las **dos ruedas** del robot, comunicarse por **WiFi** y hacer que el sistema sea **seguro ante fallas**: si el host deja de enviar comandos o el agente se cae, los motores se detienen, y cuando el agente vuelve el ESP32 se reconecta solo.

| Ítem | Lab 2 | **Lab 3** |
|---|---|---|
| Transporte | Serial | **WiFi UDP** |
| Motores | 1 | **2** (clase `Motor`) |
| Si el host deja de publicar | sigue con el último setpoint | **watchdog: se detiene en 0.5 s** |
| Si el agente se reinicia | hay que presionar RESET | **reconexión automática** |
| Interfaces | 1 sub, 3 pub | 2 sub, 3 pub, **1 servicio** |
| QoS | reliable | **best-effort** (configurable) |
| Tiempo | — | **`joint_states` con timestamp sincronizado** |
| Librerías nuevas | `speed_controller` | **`motor`, `ros_interface`** |

---

## 1. Interfaz ROS 2

Nodo: **`/robot1/lab3_motors_node`** (namespace según `ROBOT_ID`)

| Nombre | Tipo | Dirección | Descripción |
|---|---|---|---|
| `/robot1/motorR_speed` | `std_msgs/Float32` | Sub | Setpoint rueda derecha [RPM]. **Renueva el watchdog** |
| `/robot1/motorL_speed` | `std_msgs/Float32` | Sub | Setpoint rueda izquierda [RPM] |
| `/robot1/motorR_feedback` | `std_msgs/Float32` | Pub 20 Hz | Velocidad medida derecha [RPM] |
| `/robot1/motorL_feedback` | `std_msgs/Float32` | Pub 20 Hz | Velocidad medida izquierda [RPM] |
| `/robot1/joint_states` | `sensor_msgs/JointState` | Pub 20 Hz | `wheel_left_joint`, `wheel_right_joint`: posición [rad] y velocidad [rad/s], con `header.stamp` |
| `/robot1/reset_motors` | `std_srvs/Trigger` | Servicio | Detiene ambos motores y limpia rampa e integral |

Executor: 2 suscripciones + 1 servicio + 1 timer = **4 handles**.

---

## 2. Conceptos nuevos

### 2.1 Clase `Motor`: reutilizar lo construido

```
Motor ─┬─ QuadratureEncoder   (Lab 1)
       ├─ SpeedEstimator      (Lab 1)
       ├─ SpeedController     (Lab 2)
       ├─ MotorDriver         (Lab 1)
       └─ watchdog + reset    (Lab 3)
```

Las cuatro librerías anteriores se copian **sin cambios**. `Motor::update()` hace lo mismo que el cuerpo de `control_task` del Lab 2, más la verificación de seguridad. Dos ruedas = dos objetos (`motorR`, `motorL`).

### 2.2 Watchdog de comandos

Cada mensaje en `motorX_speed` llama a `setTarget()`, que guarda `lastCmdMs_ = millis()`. En cada paso de control:

```
si (ahora − lastCmdMs_) > CMD_TIMEOUT_MS  →  parada inmediata (sin rampa), integral = 0
```

**Consecuencia: el host debe publicar de forma continua** (10–20 Hz). `ros2 topic pub --once` mueve el motor solo ~0.5 s. Esto protege ante un nodo que se cae, un corte de la red WiFi o un `Ctrl+C`: el robot no queda andando solo.

### 2.3 Máquina de estados del agente (`ros_interface.cpp`)

```
WAITING_AGENT ──ping OK──► AGENT_AVAILABLE ──create_entities OK──► AGENT_CONNECTED
      ▲                          │ falla                                  │ 4 pings fallidos seguidos (~2 s)
      │                          ▼                                         ▼
      └──────────────── destroy_entities ◄──────────────────────── AGENT_DISCONNECTED
                                                                   (detiene los motores)
```

- **Ping** (`rmw_uros_ping_agent`): pregunta "¿estás ahí?" al agente cada 500 ms, sin necesidad de un nodo.
- **Crear entidades** cuando aparece el agente: support, nodo, tópicos, servicio, timer y executor, lo mismo que hacía `setup()` en los labs anteriores, ahora en `create_entities()`.
- **Destruir entidades** cuando se pierde: la sesión XRCE vive en el agente; si el agente se reinicia, la sesión vieja ya no existe y hay que crear todo de nuevo. Con `rmw_uros_set_context_entity_destroy_session_timeout(…, 0)` no se espera respuesta de un agente que ya no está.
- **Tolerancia:** en WiFi un ping perdido es normal. Solo se declara la desconexión tras `AGENT_PING_MAX_FAILS` fallos **consecutivos**.

### 2.4 QoS: reliable vs best-effort

| | Reliable (`_default`) | Best-effort (`_best_effort`) |
|---|---|---|
| Entrega | Reintenta hasta recibir confirmación (ACK) | Envía y no espera |
| Con WiFi con pérdidas | `rcl_publish` puede **bloquear** esperando el ACK | Nunca bloquea; un mensaje perdido se reemplaza por el siguiente |
| Uso típico | Comandos únicos, servicios | Sensores y setpoints periódicos |

Los setpoints y el feedback se repiten a 10–20 Hz: perder uno no importa, y bloquear sí. Por eso se usa **best-effort** (`ROS_QOS_BEST_EFFORT 1`). El servicio siempre es reliable.

> ⚠️ **Compatibilidad:** un suscriptor *reliable* **no recibe** de un publicador *best-effort*. `ros2 topic echo` se adapta solo, pero en un nodo propio usa `qos_profile_sensor_data`. Al revés sí funciona: `ros2 topic pub` (reliable) hacia la suscripción best-effort del ESP32.

> **Límite de tamaño:** con best-effort un mensaje no se fragmenta y debe caber en el MTU de micro-ROS (512 B). `Float32` y este `JointState` de 2 ruedas caben; un `nav_msgs/Odometry` (~700 B) no, y necesitaría un publisher reliable.

### 2.5 Sincronización de tiempo

El reloj del ESP32 cuenta desde el encendido. `rmw_uros_sync_session()` lo sincroniza con el del agente (al conectar y cada 60 s), y `rmw_uros_epoch_nanos()` entrega la hora ROS para el `header.stamp` de `joint_states`. Sin esto, el timestamp sería de 1970 y herramientas como `tf` o `rosbag` no podrían relacionarlo con otros datos.

### 2.6 Memoria estática en mensajes con arreglos

`JointState` contiene arreglos (`name[]`, `position[]`, `velocity[]`). En micro-ROS no se reserva memoria en tiempo de ejecución: los campos `.data` apuntan a buffers estáticos (`js_names`, `js_pos`, `js_vel`) y se fijan `.size` y `.capacity` una sola vez en `init_static_msgs()`.

### 2.7 Namespace (`ROBOT_ID`)

Con `ROBOT_ID 1`, el nodo se crea con namespace `robot1` y los nombres relativos (`"motorR_speed"`) quedan como `/robot1/motorR_speed`. **Cada grupo debe usar un ID distinto**: así varios robots comparten la red y **un solo agente** sin mezclar tópicos.

---

## 3. Archivos

| Archivo | Estado | Descripción |
|---|---|---|
| `lab3_uros_robusto.ino` | **nuevo** | Tarea de control (2 motores), LED y `loop()` |
| `config.h` | **ampliado** | Pines de 2 motores, watchdog, conexión, QoS, `ROBOT_ID` |
| `secrets.h.example` | **nuevo** | Plantilla de credenciales WiFi e IP del agente |
| `motor.h/.cpp` | **nuevo** | `Motor`: encoder + estimador + FF/PI + driver + watchdog |
| `ros_interface.h/.cpp` | **nuevo** | Entidades, callbacks, QoS, tiempo y máquina de estados |
| `speed_controller.*` | igual al Lab 2 | |
| `encoder.*`, `motor_driver.*`, `speed_estimator.*`, `microros_time_fix.cpp` | igual al Lab 1 | |
| `tools/step_test.py` | igual al Lab 2 | Se usa con parámetros (ver 6.3) |

---

## 4. Configuración

### 4.1 Credenciales

```bash
cp secrets.h.example secrets.h
```

```c
#define WIFI_SSID      "mi_red"          // 2.4 GHz
#define WIFI_PASSWORD  "mi_password"
#define AGENT_IP       "192.168.1.100"   // IP del PC del agente: ip -4 addr
#define AGENT_PORT     8888
```

`secrets.h` está en `.gitignore`.

### 4.2 `config.h`

- `ROBOT_ID`: uno distinto por grupo.
- Copia los parámetros del controlador ajustados en el Lab 2.
- Verifica `ENC_DIR_L` igual que hiciste con el motor derecho en el Lab 1 (sección 6.1).

### 4.3 Conexiones del segundo motor

| Señal | GPIO | Notas |
|---|---|---|
| `PWMB` | 27 | |
| `BIN2` → `PIN_L_IN1` | 32 | |
| `BIN1` → `PIN_L_IN2` | 33 | |
| Encoder A | 34 | **Sin pull-up interno**: agrega uno externo (10 kΩ a 3.3 V) si el encoder no lo trae |
| Encoder B | 13 | |

---

## 5. Compilar, cargar y agente (UDP)

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 lab3_uros_robusto
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/ttyUSB0 lab3_uros_robusto
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200     # ¡ahora sí se puede!
```

**Agente UDP** (mismo puerto que `AGENT_PORT`):
```bash
docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v4
# o bien
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4
```

Salida esperada en el monitor serie:
```
[lab3] Iniciando ...
[micro-ROS] Conectando a la red mi_red ...
[micro-ROS] WiFi OK, IP 192.168.1.50. Agente esperado en 192.168.1.100:8888
[lab3] Listo. Esperando al agente ...
[micro-ROS] Conectado: /robot1/lab3_motors_node
```

| LED | Estado |
|---|---|
| Parpadeo lento (500 ms) | Esperando al agente |
| Encendido fijo | Conectado, motores detenidos |
| Encendido con cortes breves | Conectado y con un setpoint vigente |
| Parpadeo rápido | Error de encoder (ver monitor serie) |

**Recomendaciones de red:**
- El ESP32 y el PC deben estar en la **misma red** y el firewall debe permitir UDP en `AGENT_PORT`.
- Si el PC usa WiFi, desactiva su ahorro de energía: `sudo iw dev wlo1 set power_save off`.
- El ESP32 desactiva su *modem-sleep* (`WIFI_DISABLE_SLEEP`); si no, la latencia sube a 100–400 ms.

---

## 6. Pruebas

> 🛑 **Ruedas en el aire.**

### 6.1 Interfaz

```bash
ros2 node list                                 # /robot1/lab3_motors_node
ros2 topic list
ros2 topic info /robot1/motorR_feedback -v     # Reliability: BEST_EFFORT
ros2 topic hz /robot1/joint_states             # ~20 Hz
ros2 topic echo /robot1/joint_states --once    # header.stamp con la hora actual
```

### 6.2 Watchdog

```bash
# Continuo (-r 10): el motor se mantiene
ros2 topic pub -r 10 /robot1/motorR_speed std_msgs/msg/Float32 "{data: 100.0}"
# Ctrl+C -> se detiene en < 0.5 s

# Un solo mensaje: se mueve ~0.5 s y se detiene
ros2 topic pub --once /robot1/motorR_speed std_msgs/msg/Float32 "{data: 100.0}"
```

### 6.3 Ambos motores, escalones y servicio

```bash
ros2 topic pub -r 10 /robot1/motorR_speed std_msgs/msg/Float32 "{data: 120.0}" &
ros2 topic pub -r 10 /robot1/motorL_speed std_msgs/msg/Float32 "{data: -120.0}" &
ros2 run rqt_plot rqt_plot /robot1/motorR_feedback/data /robot1/motorL_feedback/data

ros2 service call /robot1/reset_motors std_srvs/srv/Trigger
# Con los publishers activos, el siguiente mensaje vuelve a mover los motores.
kill $(jobs -p)

# Escalones en el motor derecho
python3 tools/step_test.py --ros-args -r __ns:=/robot1 \
  -p setpoint_topic:=motorR_speed -p feedback_topic:=motorR_feedback
```

### 6.4 Reconexión

1. Con un `pub -r 10` activo, **detén el agente** (`Ctrl+C`).
2. En ~2 s el monitor muestra `Agente perdido` y los motores se detienen (además, el watchdog ya los habría detenido a los 0.5 s, porque dejan de llegar mensajes).
3. **Vuelve a lanzar el agente**: en 1–3 s aparece `Conectado` y el motor sigue al setpoint, sin tocar el ESP32.

> Si después de reiniciar el agente `ros2 topic echo` no muestra datos, espera unos segundos o reinicia el daemon: `ros2 daemon stop; ros2 daemon start`.

---

## 7. Precisiones de implementación

- **Un solo hilo para micro-ROS:** callbacks, publicaciones, ping y creación de entidades ocurren en `loop()` (`rosSpin()`). La tarea de control solo usa la API de `Motor`. Cada variable compartida tiene un único escritor (tabla en `motor.h`).
- **Watchdog vs. pérdida de agente:** son dos protecciones distintas. El watchdog (0.5 s) cubre cualquier corte de comandos, incluso con el agente vivo (un nodo que se cae). La máquina de estados (~2 s) cubre la pérdida del agente y permite reconectar.
- **El reset es un pedido:** el callback del servicio solo activa `resetRequested_`, y la tarea de control limpia su propio estado en el paso siguiente. Así el estado del controlador tiene un único dueño.
- **Parada del watchdog sin rampa:** si el host se perdió, frenar de inmediato es más seguro que seguir la rampa.
- **`joint_states` sin cinemática:** solo se convierten unidades (cuentas → rad, RPM → rad/s). Los nombres coinciden con un URDF típico, de modo que `robot_state_publisher` puede animar las ruedas.
- **Parche obligatorio:** sin `patch_microros_time.sh`, los timers corren a 1 Hz, `spin_some` bloquea ~1 s y el watchdog se activa de forma intermitente.

---

## 8. Actividades

1. Cambia `ROS_QOS_BEST_EFFORT` a 0, aléjate del router y compara `ros2 topic hz` y la respuesta a los comandos. ¿Qué pasa con la latencia?
2. Escribe un nodo rclpy que publique `motorR_speed` y `motorL_speed` a 20 Hz y se suscriba a `joint_states`. ¿Qué QoS necesitas en la suscripción?
3. Dos grupos con `ROBOT_ID` 1 y 2, **un solo agente**. Verifica con `rqt_graph` que los tópicos no se mezclan.
4. Mide cuánto tarda el robot en detenerse al desconectar el agente con y sin watchdog (pon `CMD_TIMEOUT_MS` en 10000).

---

## 9. Solución de problemas

| Síntoma | Causa probable |
|---|---|
| Se queda en `Conectando a la red` | SSID o contraseña incorrectos, o red de 5 GHz |
| WiFi OK pero nunca `Conectado` | Agente apagado, IP o puerto incorrectos, firewall, o agente en `udp6` |
| El motor se mueve ~0.5 s y para | Watchdog: publica con `-r 10` |
| `echo` sin datos con el publisher visible | Suscriptor reliable contra un publisher best-effort, o descubrimiento DDS lento |
| Feedback a ~1 Hz, comandos con retardo | Falta el parche de `libmicroros.a` |
| Latencia alta | Ahorro de energía del WiFi (ESP32 o PC) |
| `[micro-ROS] Error ... ros_interface.cpp:NN` | Falló la creación de una entidad; revisa la línea indicada (¿suficientes handles?) |
| Encoder izquierdo siempre en 0 | Falta pull-up externo en GPIO 34 |
