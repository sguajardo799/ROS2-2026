#pragma once
// =====================================================================
//  Laboratorio 2: parámetros del hardware y del controlador (un motor)
// =====================================================================
// Respecto del Lab 1: se agregan la tarea de control y el controlador FF+PI.

// --- Driver TB6612FNG (canal A = motor derecho del robot) ---
#define PIN_PWM           19
#define PIN_IN1           15
#define PIN_IN2           26

// --- Encoder ---
#define PIN_ENC_A         25
#define PIN_ENC_B         14
#define ENC_DIR           (-1)   // verificado en el Lab 1
#define ENC_GLITCH_NS     1000

// --- Mecánica ---
#define ENCODER_CPR       28
#define GEAR_RATIO        50.0f

// --- PWM ---
#define PWM_FREQ_HZ       20000
#define PWM_RESOLUTION    8
#define PWM_MAX_DUTY      ((1 << PWM_RESOLUTION) - 1)

// --- Tarea de control (FreeRTOS) ---
#define CONTROL_PERIOD_MS      10    // 100 Hz, período fijo con vTaskDelayUntil
#define CONTROL_TASK_PRIORITY  3     // mayor que loop() (prioridad 1), donde corre micro-ROS
#define CONTROL_TASK_CORE      1
#define CONTROL_TASK_STACK     4096  // bytes

// --- Estimación de velocidad ---
// Ventana deslizante de 4 muestras x 10 ms = 40 ms (~1.07 RPM de resolución).
#define VEL_WINDOW_SAMPLES     4

// --- Feedforward (valores obtenidos con duty_sweep.py del Lab 1) ---
//   duty_ff = FF_STATIC·signo(ref) + ref / MOTOR_MAX_RPM
#define MOTOR_MAX_RPM     220.0f  // RPM de la rueda por unidad de duty (pendiente)
#define FF_STATIC         0.05f   // duty para vencer la fricción estática

// --- PI (salida en fracción de duty, error en RPM) ---
#define CTRL_KP           0.005f  // duty por RPM de error
#define CTRL_KI           0.05f   // duty por (RPM·s) de error acumulado
#define CTRL_I_LIMIT      0.5f    // |término integral| máximo (fracción de duty)

// --- Referencia ---
#define MAX_ACCEL_RPM_S   600.0f  // rampa: 0 -> 120 RPM toma 0.2 s
#define RPM_MIN           10.0f   // |setpoint| menor se trata como 0

// --- micro-ROS ---
#define NODE_NAME         "lab2_motor_node"
#define PUBLISH_PERIOD_MS 50      // feedback a 20 Hz
#define LED_PIN           2
