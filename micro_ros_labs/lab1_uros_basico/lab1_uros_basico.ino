// =====================================================================
//  Laboratorio 1: primer nodo micro-ROS (motor en lazo ABIERTO)
// =====================================================================
//  El ESP32 es un nodo ROS 2 más, conectado al agente micro-ROS por USB serial:
//
//    /motor_duty     (std_msgs/Float32, Sub)  duty con signo [-1, 1] -> PWM del motor
//    /motor_rpm      (std_msgs/Float32, Pub)  velocidad medida de la rueda [RPM], 20 Hz
//    /encoder_count  (std_msgs/Int32,   Pub)  cuentas acumuladas del encoder, 20 Hz
//
//  Comparación con un nodo rclpy (ros_introduction/temperature_publisher.py):
//    rclpy.init()                 -> allocator + rclc_support_init()
//    Node("nombre")               -> rclc_node_init_default()
//    create_publisher()           -> rclc_publisher_init_default()
//    create_subscription()        -> rclc_subscription_init_default() + executor_add_subscription()
//    create_timer()               -> rclc_timer_init_default2() + executor_add_timer()
//    rclpy.spin(node)             -> rclc_executor_spin_some() dentro de loop()
//
//  ¡OJO! El transporte serial usa el puerto Serial (USB). NO uses Serial.print():
//  rompe la comunicación con el agente. El estado se indica con el LED.
// =====================================================================

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

#include "config.h"
#include "encoder.h"
#include "motor_driver.h"
#include "speed_estimator.h"

// ---------------- Hardware ----------------
QuadratureEncoder encoder;
MotorDriver       driver;
SpeedEstimator    estimator;

// ---------------- Entidades micro-ROS ----------------
// Todas son estáticas: micro-ROS está pensado para no usar memoria dinámica
// después de la inicialización.
rcl_allocator_t allocator;   // quién reserva memoria para rcl (malloc por defecto)
rclc_support_t  support;     // contexto rcl + reloj: la "sesión" con el agente
rcl_node_t      node;        // el nodo ROS 2 (igual que en rclpy)
rclc_executor_t executor;    // decide qué callback ejecutar cuando hay datos o vence un timer

rcl_subscription_t sub_duty;
rcl_publisher_t    pub_rpm;
rcl_publisher_t    pub_count;
rcl_timer_t        timer;

// Mensajes: memoria propia, reservada una sola vez.
std_msgs__msg__Float32 msg_duty;   // lo llena el executor al llegar un mensaje
std_msgs__msg__Float32 msg_rpm;
std_msgs__msg__Int32   msg_count;

// ---------------- Manejo de errores ----------------
// Si una llamada rcl/rclc falla, detenemos el motor y parpadeamos el LED para siempre.
// (No hay Serial disponible para imprimir el error.)
#define RCCHECK(fn) { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = (fn); (void)rc; }  // ignora el error a propósito

void error_loop() {
  driver.stop();
  while (true) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ---------------- Callbacks ----------------
// Suscripción: se ejecuta DENTRO de rclc_executor_spin_some(), cada vez que llega
// un mensaje a /motor_duty. msgin apunta a msg_duty.
void duty_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  driver.setDuty(msg->data);   // lazo ABIERTO: el duty va directo al PWM
}

// Timer: también lo dispara el executor, cada PUBLISH_PERIOD_MS.
void timer_callback(rcl_timer_t *t, int64_t last_call_time) {
  (void)last_call_time;
  if (t == NULL) return;

  int32_t count = encoder.count();
  float rpm = estimator.update(count, esp_timer_get_time());

  msg_rpm.data = rpm;
  msg_count.data = count;
  RCSOFTCHECK(rcl_publish(&pub_rpm, &msg_rpm, NULL));
  RCSOFTCHECK(rcl_publish(&pub_count, &msg_count, NULL));
}

// ---------------- setup ----------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // 1) Hardware primero: el motor queda detenido antes de cualquier comunicación.
  driver.begin(PIN_PWM, PIN_IN1, PIN_IN2);
  if (!encoder.begin(PIN_ENC_A, PIN_ENC_B, ENC_DIR, ENC_GLITCH_NS)) error_loop();
  estimator.begin(encoder.count(), esp_timer_get_time());

  // 2) Transporte: micro-ROS hablará con el agente por Serial (USB, 115200 baudios).
  set_microros_transports();

  // 3) Esperar al agente. El LED parpadea lento mientras no responde.
  //    (En el Lab 3 esto se convierte en una máquina de estados con reconexión.)
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(400);
  }

  // 4) allocator + support: equivalen a rclpy.init(). Crean la sesión con el agente.
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // 5) Nodo: nombre y namespace ("" = sin namespace).
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // 6) Tópicos. ROSIDL_GET_MSG_TYPE_SUPPORT entrega la descripción del tipo de mensaje
  //    (cómo serializarlo). "_default" = QoS reliable, igual que rclpy con depth 10.
  RCCHECK(rclc_subscription_init_default(&sub_duty, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor_duty"));
  RCCHECK(rclc_publisher_init_default(&pub_rpm, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor_rpm"));
  RCCHECK(rclc_publisher_init_default(&pub_count, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "encoder_count"));

  // 7) Timer periódico (el último argumento lo deja habilitado desde el inicio).
  RCCHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS),
                                   timer_callback, true));

  // 8) Executor: hay que decirle CUÁNTOS "handles" atenderá (suscripciones + timers +
  //    servicios). Aquí: 1 suscripción + 1 timer = 2. Los publishers NO son handles:
  //    publicar no requiere al executor.
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_duty, &msg_duty,
                                         &duty_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  digitalWrite(LED_PIN, HIGH);  // LED fijo = conectado al agente
}

// ---------------- loop ----------------
// Equivale a rclpy.spin(): revisa si llegaron mensajes o vencieron timers y ejecuta
// sus callbacks. Espera como máximo 10 ms por datos nuevos.
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
