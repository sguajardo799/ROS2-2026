#pragma once
// =====================================================================
//  Laboratorio 1: parámetros del hardware (un solo motor)
// =====================================================================
// Todo lo que depende del robot está aquí. Las librerías (encoder,
// motor_driver, speed_estimator) leen estos #define.

// --- Driver TB6612FNG (canal A = motor derecho del robot) ---
// El pin STBY del driver debe estar conectado a 3.3 V (HIGH).
#define PIN_PWM           19
#define PIN_IN1           15   // GPIO15 es pin de strapping: sirve como salida
#define PIN_IN2           26

// --- Encoder en cuadratura ---
#define PIN_ENC_A         25
#define PIN_ENC_B         14
// Sentido de conteo (+1 / -1). Al girar la rueda "hacia adelante" a mano,
// la velocidad publicada debe ser POSITIVA. Si no, cambia el signo.
#define ENC_DIR           (-1)
#define ENC_GLITCH_NS     1000   // filtro de pulsos espurios del PCNT [ns]

// --- Mecánica ---
// Cuentas por vuelta del eje del MOTOR en cuadratura x4 (= 4 x 7 PPR).
#define ENCODER_CPR       28
#define GEAR_RATIO        50.0f  // reductora 50:1 -> 28 x 50 = 1400 cuentas por vuelta de rueda

// --- PWM ---
#define PWM_FREQ_HZ       20000  // 20 kHz: fuera del rango audible
#define PWM_RESOLUTION    8      // bits -> duty de 0 a 255
#define PWM_MAX_DUTY      ((1 << PWM_RESOLUTION) - 1)

// --- Estimación de velocidad ---
// En este laboratorio la velocidad se calcula una vez por cada publicación,
// con la diferencia de cuentas desde la publicación anterior (ventana = 1 muestra).
#define VEL_WINDOW_SAMPLES 1

// --- micro-ROS ---
#define NODE_NAME         "lab1_motor_node"
#define PUBLISH_PERIOD_MS 50     // timer de publicación: 20 Hz
#define LED_PIN           2      // LED integrado del ESP32 DevKit
