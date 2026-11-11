// =====================================================================
//  Laboratorio 2: control de velocidad FF + PI con setpoint por ROS 2
// =====================================================================
//    /motor_setpoint  (std_msgs/Float32, Sub)  velocidad deseada de la rueda [RPM]
//    /motor_rpm       (std_msgs/Float32, Pub)  velocidad medida [RPM]            20 Hz
//    /motor_reference (std_msgs/Float32, Pub)  referencia tras la rampa [RPM]     20 Hz
//    /motor_duty      (std_msgs/Float32, Pub)  esfuerzo de control u en [-1, 1]   20 Hz
//
//  Hay DOS tareas FreeRTOS en el core 1:
//
//    control_task  (prioridad 3, cada 10 ms exactos)   loop()  (prioridad 1)
//    ─────────────────────────────────────────────     ─────────────────────────────
//    encoder -> velocidad -> FF+PI -> PWM              rclc_executor_spin_some():
//                                                      callback de setpoint + timer
//
//  El lazo de control NO puede depender de la red: si micro-ROS se demora,
//  la tarea de mayor prioridad lo interrumpe y mantiene su período de 10 ms.
//  Las dos tareas comparten solo 4 variables (ver "Datos compartidos").
//
//  Transporte: Serial USB, igual que en el Lab 1 (no usar Serial.print).
// =====================================================================

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>

#include "config.h"
#include "encoder.h"
#include "motor_driver.h"
#include "speed_estimator.h"
#include "speed_controller.h"   // NUEVO en el Lab 2

// ---------------- Hardware y control ----------------
QuadratureEncoder encoder;
MotorDriver       driver;
SpeedEstimator    estimator;
SpeedController   controller;

// ---------------- Datos compartidos entre tareas ----------------
// Regla: cada variable tiene UN SOLO escritor. Un float de 32 bits se lee y escribe
// de forma atómica en el ESP32, así que no hace falta un mutex. "volatile" obliga al
// compilador a leer la memoria cada vez (otra tarea puede haberla cambiado).
volatile float g_setpointRPM = 0;  // escribe: loop() (callback ROS)  | lee: control_task
volatile float g_measuredRPM = 0;  // escribe: control_task           | lee: timer ROS
volatile float g_refRPM = 0;       // escribe: control_task           | lee: timer ROS
volatile float g_duty = 0;         // escribe: control_task           | lee: timer ROS

// ---------------- Entidades micro-ROS (como en el Lab 1) ----------------
rcl_allocator_t allocator;
rclc_support_t  support;
rcl_node_t      node;
rclc_executor_t executor;

rcl_subscription_t sub_setpoint;
rcl_publisher_t    pub_rpm, pub_ref, pub_duty;
rcl_timer_t        timer;

std_msgs__msg__Float32 msg_setpoint;
std_msgs__msg__Float32 msg_rpm, msg_ref, msg_duty;

#define RCCHECK(fn) { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = (fn); (void)rc; }

void error_loop() {
  g_setpointRPM = 0;  // la tarea de control lleva el motor a 0 con la rampa
  while (true) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// =====================================================================
//  Tarea de control: 100 Hz con vTaskDelayUntil
// =====================================================================
// vTaskDelayUntil despierta la tarea en múltiplos exactos del período, aunque el
// cuerpo tarde distinto en cada vuelta (delay() acumularía ese error).
void control_task(void *arg) {
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
  TickType_t lastWake = xTaskGetTickCount();
  int64_t lastUs = esp_timer_get_time();

  while (true) {
    vTaskDelayUntil(&lastWake, period);

    int64_t nowUs = esp_timer_get_time();
    float dt = (nowUs - lastUs) * 1e-6f;   // Δt real (~0.010 s)
    lastUs = nowUs;

    float rpm = estimator.update(encoder.count(), nowUs);    // 1. medir
    float u = controller.update(g_setpointRPM, rpm, dt);     // 2. calcular
    driver.setDuty(u);                                       // 3. actuar

    g_measuredRPM = rpm;
    g_refRPM = controller.ref();
    g_duty = u;
  }
}

// =====================================================================
//  Callbacks micro-ROS (se ejecutan en loop(), dentro de spin_some)
// =====================================================================
void setpoint_callback(const void *msgin) {
  g_setpointRPM = ((const std_msgs__msg__Float32 *)msgin)->data;
}

void timer_callback(rcl_timer_t *t, int64_t last_call_time) {
  (void)last_call_time;
  if (t == NULL) return;
  msg_rpm.data = g_measuredRPM;
  msg_ref.data = g_refRPM;
  msg_duty.data = g_duty;
  RCSOFTCHECK(rcl_publish(&pub_rpm, &msg_rpm, NULL));
  RCSOFTCHECK(rcl_publish(&pub_ref, &msg_ref, NULL));
  RCSOFTCHECK(rcl_publish(&pub_duty, &msg_duty, NULL));
}

// =====================================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // 1) Hardware: motor detenido y encoder contando.
  driver.begin(PIN_PWM, PIN_IN1, PIN_IN2);
  if (!encoder.begin(PIN_ENC_A, PIN_ENC_B, ENC_DIR, ENC_GLITCH_NS)) error_loop();
  estimator.begin(encoder.count(), esp_timer_get_time());

  // 2) Tarea de control: arranca ya (con setpoint 0 el motor queda detenido).
  xTaskCreatePinnedToCore(control_task, "control", CONTROL_TASK_STACK, NULL,
                          CONTROL_TASK_PRIORITY, NULL, CONTROL_TASK_CORE);

  // 3) micro-ROS por Serial, igual que en el Lab 1.
  set_microros_transports();
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(400);
  }

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  RCCHECK(rclc_subscription_init_default(&sub_setpoint, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor_setpoint"));
  RCCHECK(rclc_publisher_init_default(&pub_rpm, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor_rpm"));
  RCCHECK(rclc_publisher_init_default(&pub_ref, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor_reference"));
  RCCHECK(rclc_publisher_init_default(&pub_duty, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor_duty"));
  RCCHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS),
                                   timer_callback, true));

  // Handles: 1 suscripción + 1 timer = 2 (los 3 publishers no cuentan).
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_setpoint, &msg_setpoint,
                                         &setpoint_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  digitalWrite(LED_PIN, HIGH);
}

// loop() es también una tarea FreeRTOS ("loopTask", prioridad 1, core 1).
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
