# Laboratorio 2: control de velocidad Feedforward + PI

**Objetivo:** cerrar el lazo. Desde ROS 2 se pide una velocidad en **RPM**, y el ESP32 la mantiene con un controlador **feedforward + PI** que corre a **100 Hz** en una tarea de FreeRTOS, independiente de micro-ROS.

| Ítem | Lab 1 | **Lab 2** |
|---|---|---|
| Transporte | Serial | Serial |
| Motores | 1 | 1 |
| Comando | duty (lazo abierto) | **RPM (lazo cerrado)** |
| Tareas | solo `loop()` | **`control_task` (100 Hz)** + `loop()` |
| Librerías nuevas | `encoder`, `motor_driver`, `speed_estimator` | **`speed_controller`** |

---

## 1. Interfaz ROS 2

Nodo: **`/lab2_motor_node`**

| Tópico | Tipo | Dirección | Descripción |
|---|---|---|---|
| `/motor_setpoint` | `std_msgs/Float32` | Sub | Velocidad deseada de la rueda [RPM], con signo |
| `/motor_rpm` | `std_msgs/Float32` | Pub 20 Hz | Velocidad medida [RPM] (ventana de 40 ms) |
| `/motor_reference` | `std_msgs/Float32` | Pub 20 Hz | Referencia después de la rampa [RPM]: es lo que el PI intenta seguir |
| `/motor_duty` | `std_msgs/Float32` | Pub 20 Hz | Esfuerzo de control `u` en `[-1, 1]` |

Executor: 1 suscripción + 1 timer = **2 handles** (los 3 publishers no cuentan).

---

## 2. El controlador

```
 setpoint ─► [zona muerta] ─► [rampa] ─► ref ─┬─────────► [ FF ] ─────────┐
  (ROS)       |sp|<RPM_MIN→0   600 RPM/s       │                           ▼
                                               └─►(+)─ e ─► [ Kp + Ki/s ] ─(+)─► [sat ±1] ─► u ─► TB6612
                                                   ▲ −                                  │
                                RPM medida ────────┘◄── [ventana 40 ms] ◄── PCNT ◄──────┘ motor
```

```
u = FF_STATIC·signo(ref) + ref / MOTOR_MAX_RPM   +   Kp·e  +  ∫ Ki·e dt
    └──────────── feedforward ───────────────┘       └────── PI ──────┘
```

| Bloque | Para qué sirve |
|---|---|
| **Feedforward** | Usa el modelo identificado en el Lab 1 (`duty_sweep.py`) para dar **de inmediato** casi todo el duty necesario. No espera a que exista error. |
| **P** | Reacciona al error actual. Con FF, basta un `Kp` pequeño. |
| **I** | Elimina el error estacionario que deja el modelo (carga, batería, fricción). |
| **Sin D** | La derivada del error amplificaría el ruido de cuantización del encoder (±1 cuenta ≈ ±1 RPM). |
| **Rampa** | Limita la aceleración pedida: evita picos de corriente y que las ruedas patinen, y hace la respuesta predecible. |
| **Zona muerta** (`RPM_MIN`) | A muy baja velocidad hay pocas cuentas por ventana y domina la fricción: esos setpoints se tratan como 0. |
| **Saturación ±1** | El PWM no puede superar el 100 %. |
| **Anti-windup** | Si `u` está saturado y el error empuja hacia la misma saturación, el integral **no crece**. Sin esto, el integral se "carga" durante la saturación y luego produce un sobreimpulso grande. Además, se limita a `±CTRL_I_LIMIT`. |

Todo esto está en `speed_controller.cpp`, en cinco pasos numerados. La clase **no toca hardware**: recibe RPM y entrega `u`, así que se puede probar por separado y se reutiliza tal cual en el Lab 3.

---

## 3. Dos tareas: control y comunicación

```
core 1 │ control_task (prio 3) ██      ██      ██      ██      ██   ← cada 10 ms exactos
       │ loop() / micro-ROS (1)   ░░░░░░  ░░░░░░  ░░░░░░  ░░░░░░     ← usa el tiempo restante
       └────────────────────────────────────────────────────────────► t
```

- **`control_task`** (prioridad 3) usa `vTaskDelayUntil()`: despierta en múltiplos exactos de 10 ms aunque cada vuelta tarde distinto. En cada vuelta: **medir → calcular → actuar**.
- **`loop()`** también es una tarea de FreeRTOS (`loopTask`, prioridad 1). Ahí corre `rclc_executor_spin_some()`. Si la comunicación se demora, la tarea de control la **interrumpe**: el período de control no depende de la red.
- **Δt medido:** el controlador y el estimador usan el Δt real (`esp_timer_get_time()`), no el nominal.

### Datos compartidos

| Variable | Escribe | Lee |
|---|---|---|
| `g_setpointRPM` | `loop()` (callback de `/motor_setpoint`) | `control_task` |
| `g_measuredRPM`, `g_refRPM`, `g_duty` | `control_task` | `loop()` (timer de publicación) |

**Regla: un solo escritor por variable.** Un `float` de 32 bits se lee y se escribe de forma atómica en el ESP32, así que no hace falta un mutex. `volatile` evita que el compilador guarde el valor en un registro. El estado del controlador (`ref`, `integral`) solo lo toca `control_task`.

> ⚠️ La API de micro-ROS **no es thread-safe**: todas las llamadas `rcl_*`/`rclc_*` se hacen desde `loop()`. La tarea de control nunca publica directamente.

---

## 4. Archivos

| Archivo | Estado | Descripción |
|---|---|---|
| `lab2_control_ff_pi.ino` | **nuevo** | Tarea de control + nodo micro-ROS |
| `config.h` | **ampliado** | Agrega tarea de control, FF, PI y rampa |
| `speed_controller.h/.cpp` | **nuevo** | `SpeedController`: FF + PI, rampa, zona muerta y anti-windup |
| `encoder.*`, `motor_driver.*`, `speed_estimator.*`, `microros_time_fix.cpp` | igual al Lab 1 | |
| `tools/step_test.py` | **nuevo** | Escalones de velocidad, métricas y CSV |

---

## 5. Compilar, cargar y agente

Igual que en el Lab 1 (transporte serial, **monitor serie cerrado**):

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 lab2_control_ff_pi
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/ttyUSB0 lab2_control_ff_pi

docker run -it --rm -v /dev:/dev --privileged --net=host \
  microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0 -b 115200 -v4
```

**Antes de compilar**, copia en `config.h` los valores `FF_STATIC` y `MOTOR_MAX_RPM` que obtuviste en el Lab 1.

---

## 6. Pruebas

> 🛑 **Rueda en el aire** para las primeras pruebas.

```bash
ros2 topic pub --once /motor_setpoint std_msgs/msg/Float32 "{data: 120.0}"
ros2 topic pub --once /motor_setpoint std_msgs/msg/Float32 "{data: -80.0}"
ros2 topic pub --once /motor_setpoint std_msgs/msg/Float32 "{data: 0.0}"

ros2 run rqt_plot rqt_plot /motor_reference/data /motor_rpm/data /motor_duty/data
```

- **Frena la rueda con los dedos**: en el Lab 1 la velocidad caía; ahora `/motor_duty` sube y la RPM vuelve a la referencia. Esa es la acción del PI.
- Un setpoint de 5 RPM deja el motor detenido (zona muerta).
- Un setpoint de 1000 RPM satura `u` en 1.0: la RPM llega al máximo y, al bajar el setpoint, no hay sobreimpulso gracias al anti-windup.

### Prueba de escalones

```bash
python3 tools/step_test.py
```

Publica la secuencia 0 → 60 → 120 → 180 → −120 → 0 RPM y muestra, para cada escalón, el tiempo de subida (10–90 %), el sobreimpulso y el error estacionario. Guarda `step_test.csv`. El tiempo de subida incluye la rampa: con 600 RPM/s, un escalón de 60 RPM ya tarda 0.08 s solo por la rampa.

---

## 7. Procedimiento de ajuste

Cada cambio requiere volver a compilar y cargar.

1. **Solo feedforward:** `CTRL_KP 0` y `CTRL_KI 0`. Prueba 40, 100 y 150 RPM. Si la RPM medida queda **bajo** el setpoint en todo el rango, baja `MOTOR_MAX_RPM`; si a baja velocidad la rueda no arranca, sube `FF_STATIC`. Un buen FF deja un error de pocas RPM.
2. **P:** con `CTRL_KI 0`, sube `CTRL_KP` (0.002 → 0.005 → 0.01…) hasta que la respuesta sea rápida **sin oscilar**. Observa `/motor_duty`: si salta mucho, `Kp` amplifica el ruido.
3. **I:** sube `CTRL_KI` hasta eliminar el error estacionario. Si aparece sobreimpulso u oscilación lenta, bájalo.
4. **Ruido:** si `/motor_rpm` es muy ruidoso, sube `VEL_WINDOW_SAMPLES` (menos ruido, más retardo).
5. Repite con la rueda **en el suelo**: la carga cambia.

Valores de partida (`ROS_botV2`): `MOTOR_MAX_RPM 220`, `FF_STATIC 0.05`, `Kp 0.005`, `Ki 0.05`.

---

## 8. Precisiones de implementación

- **Unidades:** el error está en RPM y la salida en fracción de duty. Por eso `Kp = 0.005` significa "0.5 % de duty por cada RPM de error". Si cambias `PWM_RESOLUTION`, las ganancias no cambian, porque `MotorDriver` escala `u` a `PWM_MAX_DUTY`.
- **¿Por qué 100 Hz?** El motor con reductora tiene una constante de tiempo de decenas de ms; 10 ms basta, y una ventana de 40 ms da una resolución de ~1 RPM. Controlar más rápido solo aumentaría el ruido de la medición.
- **El timer de publicación (20 Hz) es independiente del control (100 Hz).** Los datos de ROS son muestras de lo que hace el lazo, no el lazo mismo.
- **Sin protección todavía:** si el agente o el cable se cortan, el motor **mantiene el último setpoint** indefinidamente. Ese es el problema principal que resuelve el Lab 3 (watchdog de comandos y reconexión).

---

## 9. Actividades

1. Grafica la respuesta a 120 RPM con solo FF, con FF + P y con FF + PI. Compara el error estacionario.
2. Desactiva el feedforward (`MOTOR_MAX_RPM` muy grande y `FF_STATIC 0`) y reajusta el PI. ¿Qué ganancias necesitas ahora y cómo cambia la respuesta?
3. Comenta las líneas del anti-windup, pide 1000 RPM durante 3 s y luego 100 RPM. Explica el sobreimpulso.
4. Reemplaza `vTaskDelayUntil` por `vTaskDelay`. ¿Qué pasa con el período real? (Pista: mide `dt`.)
