#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>
#include <std_srvs/srv/trigger.h>
#include <sensor_msgs/msg/joint_state.h>

#include "config.h"
#include "secrets.h"
#include "ros_interface.h"

// =====================================================================
//  Interfaz ROS 2 (namespace /robot<ID>):
//    motorR_speed, motorL_speed        std_msgs/Float32      Sub  setpoint [RPM]
//    motorR_feedback, motorL_feedback  std_msgs/Float32      Pub  velocidad medida [RPM]
//    joint_states                      sensor_msgs/JointState Pub posición [rad] y vel. [rad/s]
//    reset_motors                      std_srvs/Trigger      Srv  detiene y limpia el control
// =====================================================================

// Con WiFi el Serial queda libre: se puede imprimir el error.
#define RCCHECK(fn) { rcl_ret_t rc_ = (fn); if (rc_ != RCL_RET_OK) { \
  Serial.printf("[micro-ROS] Error %d en %s:%d\n", (int)rc_, __FILE__, __LINE__); return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc_ = (fn); (void)rc_; }

// Ejecuta X como máximo una vez cada MS milisegundos (un temporizador por cada uso).
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static int64_t t_ = -1; \
  if (t_ == -1) t_ = millis(); \
  if ((int32_t)(millis() - t_) > (MS)) { X; t_ = millis(); } \
} while (0)

// QoS elegido en config.h. Best-effort = "envía y olvida": rcl_publish no espera
// confirmación. Reliable = reintenta hasta recibir ACK (puede bloquear con WiFi malo).
#if ROS_QOS_BEST_EFFORT
  #define SUB_INIT rclc_subscription_init_best_effort
  #define PUB_INIT rclc_publisher_init_best_effort
#else
  #define SUB_INIT rclc_subscription_init_default
  #define PUB_INIT rclc_publisher_init_default
#endif

static Motor *motorR = nullptr;
static Motor *motorL = nullptr;

// ---------------- Entidades ----------------
static rcl_allocator_t allocator;
static rclc_support_t  support;
static rcl_node_t      node;
static rclc_executor_t executor;
static rcl_timer_t     feedback_timer;

static rcl_subscription_t sub_r, sub_l;
static std_msgs__msg__Float32 msg_sub_r, msg_sub_l;

static rcl_publisher_t pub_fb_r, pub_fb_l, pub_js;
static std_msgs__msg__Float32 msg_fb_r, msg_fb_l;

// JointState tiene arreglos (nombres, posiciones, velocidades). En micro-ROS no hay
// memoria dinámica: los arreglos apuntan a buffers estáticos propios.
static sensor_msgs__msg__JointState msg_js;
static rosidl_runtime_c__String js_names[2];
static double js_pos[2], js_vel[2];

static rcl_service_t reset_srv;
static std_srvs__srv__Trigger_Request  reset_req;
static std_srvs__srv__Trigger_Response reset_resp;

// ---------------- Máquina de estados ----------------
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static AgentState state = WAITING_AGENT;
static uint8_t ping_fails = 0;

// =====================================================================
//  Mensajes con memoria estática
// =====================================================================
static void set_string(rosidl_runtime_c__String &s, const char *text) {
  s.data = (char *)text;
  s.size = strlen(text);
  s.capacity = s.size + 1;
}

static void init_static_msgs() {
  set_string(msg_js.header.frame_id, "");
  set_string(js_names[0], JOINT_LEFT_NAME);
  set_string(js_names[1], JOINT_RIGHT_NAME);
  msg_js.name.data = js_names;    msg_js.name.size = msg_js.name.capacity = 2;
  msg_js.position.data = js_pos;  msg_js.position.size = msg_js.position.capacity = 2;
  msg_js.velocity.data = js_vel;  msg_js.velocity.size = msg_js.velocity.capacity = 2;
  msg_js.effort.data = NULL;      msg_js.effort.size = msg_js.effort.capacity = 0;
}

// Timestamp en el reloj del agente (tiempo ROS) gracias a rmw_uros_sync_session().
static void fill_stamp(builtin_interfaces__msg__Time &t) {
  int64_t ns = rmw_uros_epoch_nanos();
  t.sec = (int32_t)(ns / 1000000000LL);
  t.nanosec = (uint32_t)(ns % 1000000000LL);
}

static void sync_time() {
  if (rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS) != RMW_RET_OK) {
    Serial.println("[micro-ROS] Aviso: no se pudo sincronizar el reloj con el agente");
  }
}

// =====================================================================
//  Callbacks (los ejecuta rclc_executor_spin_some, en loop())
// =====================================================================
static void motor_r_callback(const void *msgin) {
  motorR->setTarget(((const std_msgs__msg__Float32 *)msgin)->data);  // renueva el watchdog
}

static void motor_l_callback(const void *msgin) {
  motorL->setTarget(((const std_msgs__msg__Float32 *)msgin)->data);
}

// Servicio: (request, response). Igual que en rclpy, se llena la respuesta y el
// executor la envía al cliente al terminar el callback.
static void reset_callback(const void *req, void *res) {
  (void)req;
  auto response = (std_srvs__srv__Trigger_Response *)res;
  motorR->requestReset();
  motorL->requestReset();
  response->success = true;
  set_string(response->message, "Motores detenidos y controlador reiniciado");
  Serial.println("[micro-ROS] reset_motors atendido");
}

static void feedback_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) return;

  msg_fb_r.data = motorR->measuredRPM();
  msg_fb_l.data = motorL->measuredRPM();
  RCSOFTCHECK(rcl_publish(&pub_fb_r, &msg_fb_r, NULL));
  RCSOFTCHECK(rcl_publish(&pub_fb_l, &msg_fb_l, NULL));

  fill_stamp(msg_js.header.stamp);
  js_pos[0] = motorL->positionRad();   js_pos[1] = motorR->positionRad();
  js_vel[0] = motorL->velocityRadS();  js_vel[1] = motorR->velocityRadS();
  RCSOFTCHECK(rcl_publish(&pub_js, &msg_js, NULL));
}

// =====================================================================
//  Crear / destruir entidades
// =====================================================================
// Se crean cada vez que aparece el agente y se destruyen cuando se pierde: la
// sesión vieja ya no existe en el agente nuevo, así que hay que empezar de cero.
static bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // Namespace: los nombres relativos ("motorR_speed") quedan en /robot<ID>/motorR_speed
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, ROBOT_NAMESPACE, &support));

  const rosidl_message_type_support_t *float32_ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);
  RCCHECK(SUB_INIT(&sub_r, &node, float32_ts, "motorR_speed"));
  RCCHECK(SUB_INIT(&sub_l, &node, float32_ts, "motorL_speed"));
  RCCHECK(PUB_INIT(&pub_fb_r, &node, float32_ts, "motorR_feedback"));
  RCCHECK(PUB_INIT(&pub_fb_l, &node, float32_ts, "motorL_feedback"));
  RCCHECK(PUB_INIT(&pub_js, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states"));
  // Los servicios siempre son reliable (la respuesta debe llegar).
  RCCHECK(rclc_service_init_default(&reset_srv, &node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "reset_motors"));
  RCCHECK(rclc_timer_init_default2(&feedback_timer, &support, RCL_MS_TO_NS(FEEDBACK_PERIOD_MS),
                                   feedback_timer_callback, true));

  // Handles: 2 suscripciones + 1 servicio + 1 timer = 4
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_r, &msg_sub_r, motor_r_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_l, &msg_sub_l, motor_l_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_service(&executor, &reset_srv, &reset_req, &reset_resp, reset_callback));
  RCCHECK(rclc_executor_add_timer(&executor, &feedback_timer));

  sync_time();
  return true;
}

static void destroy_entities() {
  // El agente ya no responde: no esperar confirmación al destruir cada entidad.
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  if (rmw_context != NULL) {
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  }
  // Orden inverso a la creación.
  RCSOFTCHECK(rcl_timer_fini(&feedback_timer));
  RCSOFTCHECK(rcl_service_fini(&reset_srv, &node));
  RCSOFTCHECK(rcl_publisher_fini(&pub_js, &node));
  RCSOFTCHECK(rcl_publisher_fini(&pub_fb_l, &node));
  RCSOFTCHECK(rcl_publisher_fini(&pub_fb_r, &node));
  RCSOFTCHECK(rcl_subscription_fini(&sub_l, &node));
  RCSOFTCHECK(rcl_subscription_fini(&sub_r, &node));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}

static bool ping_agent() {
  return rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1) == RMW_RET_OK;
}

// =====================================================================
//  API pública
// =====================================================================
void rosBegin(Motor &right, Motor &left) {
  motorR = &right;
  motorL = &left;
  init_static_msgs();

  // Transporte UDP sobre WiFi: bloquea hasta conectarse a la red.
  Serial.printf("[micro-ROS] Conectando a la red %s ...\n", WIFI_SSID);
  set_microros_wifi_transports((char *)WIFI_SSID, (char *)WIFI_PASSWORD, (char *)AGENT_IP, AGENT_PORT);
#if WIFI_DISABLE_SLEEP
  WiFi.setSleep(false);
#endif
  Serial.printf("[micro-ROS] WiFi OK, IP %s. Agente esperado en %s:%d\n",
                WiFi.localIP().toString().c_str(), AGENT_IP, AGENT_PORT);
}

bool rosConnected() { return state == AGENT_CONNECTED; }

//   WAITING_AGENT ──ping OK──► AGENT_AVAILABLE ──create OK──► AGENT_CONNECTED
//        ▲                           │ falla                         │ N pings fallidos
//        └────── destroy_entities ◄──┴──────────── AGENT_DISCONNECTED ◄┘ (detiene motores)
void rosSpin() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(AGENT_PING_PERIOD_MS,
        state = ping_agent() ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      if (create_entities()) {
        Serial.printf("[micro-ROS] Conectado: /%s%s%s\n",
                      ROBOT_NAMESPACE, ROBOT_ID > 0 ? "/" : "", NODE_NAME);
        ping_fails = 0;
        state = AGENT_CONNECTED;
      } else {
        destroy_entities();
        state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      // Un ping perdido en WiFi es normal: solo se desconecta tras N fallos seguidos.
      EXECUTE_EVERY_N_MS(AGENT_PING_PERIOD_MS, {
        if (ping_agent()) ping_fails = 0;
        else if (++ping_fails >= AGENT_PING_MAX_FAILS) state = AGENT_DISCONNECTED;
      });
      EXECUTE_EVERY_N_MS(TIME_SYNC_PERIOD_MS, sync_time(););
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      break;

    case AGENT_DISCONNECTED:
      Serial.println("[micro-ROS] Agente perdido: motores detenidos, reconectando ...");
      motorR->requestReset();
      motorL->requestReset();
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}
