#pragma once
// =====================================================================
//  Laboratorio 3: dos motores, micro-ROS por WiFi y operación robusta
// =====================================================================
// Respecto del Lab 2: pines del segundo motor, watchdog de comandos,
// parámetros de conexión con el agente, QoS y namespace del robot.

// --- Identidad ---
// Con ROBOT_ID > 0 todo queda bajo /robot<ID> (ej. /robot1/motorR_speed).
// Cada grupo usa un ID distinto: así varios robots comparten la red y el agente.
#define ROBOT_ID          1
#define NODE_NAME         "lab3_motors_node"

// --- Motor derecho (canal A del TB6612) ---
#define PIN_R_PWM         19
#define PIN_R_IN1         15
#define PIN_R_IN2         26
#define PIN_R_ENC_A       25
#define PIN_R_ENC_B       14
#define ENC_DIR_R         (-1)

// --- Motor izquierdo (canal B del TB6612) ---
#define PIN_L_PWM         27
#define PIN_L_IN1         32
#define PIN_L_IN2         33
#define PIN_L_ENC_A       34     // GPIO34 no tiene pull-up interno: usar uno externo
#define PIN_L_ENC_B       13
#define ENC_DIR_L         (-1)

#define ENC_GLITCH_NS     1000

// --- Mecánica ---
#define ENCODER_CPR       28
#define GEAR_RATIO        50.0f

// --- PWM ---
#define PWM_FREQ_HZ       20000
#define PWM_RESOLUTION    8
#define PWM_MAX_DUTY      ((1 << PWM_RESOLUTION) - 1)

// --- Tarea de control ---
#define CONTROL_PERIOD_MS      10
#define CONTROL_TASK_PRIORITY  3
#define CONTROL_TASK_CORE      1
#define CONTROL_TASK_STACK     4096
#define VEL_WINDOW_SAMPLES     4

// --- Controlador (copiar los valores ajustados en el Lab 2) ---
#define MOTOR_MAX_RPM     220.0f
#define FF_STATIC         0.05f
#define CTRL_KP           0.005f
#define CTRL_KI           0.05f
#define CTRL_I_LIMIT      0.5f
#define MAX_ACCEL_RPM_S   600.0f
#define RPM_MIN           10.0f

// --- Seguridad: watchdog de comandos ---
// Si un motor no recibe setpoint en este tiempo, se detiene de inmediato.
// El host debe publicar de forma continua (> 1000/CMD_TIMEOUT_MS Hz, idealmente 10-20 Hz).
#define CMD_TIMEOUT_MS    500

// --- micro-ROS: publicación ---
#define FEEDBACK_PERIOD_MS     50     // motorX_feedback + joint_states (20 Hz)
#define JOINT_LEFT_NAME        "wheel_left_joint"
#define JOINT_RIGHT_NAME       "wheel_right_joint"

// --- micro-ROS: conexión con el agente ---
#define AGENT_PING_PERIOD_MS   500    // cada cuánto se verifica el agente
#define AGENT_PING_TIMEOUT_MS  200
#define AGENT_PING_MAX_FAILS   4      // pings fallidos SEGUIDOS antes de darlo por perdido (~2 s)
#define TIME_SYNC_PERIOD_MS    60000  // resincronizar el reloj con el agente cada 60 s
#define TIME_SYNC_TIMEOUT_MS   200

// --- Red / QoS ---
#define WIFI_DISABLE_SLEEP     1      // el modem-sleep del WiFi agrega 100-400 ms de latencia
#define ROS_QOS_BEST_EFFORT    1      // 1: best-effort (no espera ACK)  0: reliable

#define LED_PIN                2

// ===== Derivados (no editar) =====
#define CFG_STR_(x) #x
#define CFG_STR(x)  CFG_STR_(x)
#if ROBOT_ID > 0
  #define ROBOT_NAMESPACE  "robot" CFG_STR(ROBOT_ID)   // "robot1"
#else
  #define ROBOT_NAMESPACE  ""
#endif
