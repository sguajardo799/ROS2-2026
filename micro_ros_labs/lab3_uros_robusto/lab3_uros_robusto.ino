// =====================================================================
//  Laboratorio 3: dos motores con FF+PI, micro-ROS por WiFi y operación robusta
// =====================================================================
//  Respecto del Lab 2:
//    - Dos ruedas: la clase Motor agrupa encoder + estimador + FF/PI + driver.
//    - Transporte WiFi (UDP): el Serial queda libre para mensajes de estado.
//    - Watchdog de comandos: sin setpoints por CMD_TIMEOUT_MS, el motor se detiene.
//    - Máquina de estados: el ESP32 detecta si el agente se cae y se reconecta solo.
//    - Servicio reset_motors, QoS best-effort, namespace /robot<ID> y
//      joint_states con timestamp sincronizado con el agente.
//
//    control_task (prioridad 3, 100 Hz)          loop() (prioridad 1)
//    motorR.update(dt); motorL.update(dt)        rosSpin(): máquina de estados + executor
// =====================================================================

#include "config.h"
#include "motor.h"
#include "ros_interface.h"

Motor motorR, motorL;

// Tarea de control: igual que en el Lab 2, pero para dos motores.
void control_task(void *arg) {
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
  TickType_t lastWake = xTaskGetTickCount();
  int64_t lastUs = esp_timer_get_time();

  while (true) {
    vTaskDelayUntil(&lastWake, period);
    int64_t nowUs = esp_timer_get_time();
    float dt = (nowUs - lastUs) * 1e-6f;
    lastUs = nowUs;

    motorR.update(dt);
    motorL.update(dt);
  }
}

// LED de estado: parpadeo = esperando al agente, fijo = conectado,
// fijo con cortes breves = conectado y moviéndose.
void update_led() {
  uint32_t t = millis();
  bool on;
  if (!rosConnected())                              on = (t / 500) % 2;
  else if (motorR.isActive() || motorL.isActive())  on = (t % 500) < 400;
  else                                              on = true;
  digitalWrite(LED_PIN, on);
}

void setup() {
  Serial.begin(115200);          // con WiFi, el Serial es solo para mensajes
  pinMode(LED_PIN, OUTPUT);
  Serial.println("\n[lab3] Iniciando ...");

  // 1) Motores detenidos y encoders contando antes de cualquier comunicación.
  bool okR = motorR.begin({PIN_R_PWM, PIN_R_IN1, PIN_R_IN2, PIN_R_ENC_A, PIN_R_ENC_B, ENC_DIR_R});
  bool okL = motorL.begin({PIN_L_PWM, PIN_L_IN1, PIN_L_IN2, PIN_L_ENC_A, PIN_L_ENC_B, ENC_DIR_L});
  if (!okR || !okL) {
    Serial.printf("[lab3] ERROR: encoder %s no inicializado\n", !okR ? "derecho" : "izquierdo");
    while (true) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
  }

  // 2) Lazo de control a 100 Hz (con el watchdog vencido, los motores quedan detenidos).
  xTaskCreatePinnedToCore(control_task, "control", CONTROL_TASK_STACK, NULL,
                          CONTROL_TASK_PRIORITY, NULL, CONTROL_TASK_CORE);

  // 3) WiFi + transporte micro-ROS. El agente se busca en loop().
  rosBegin(motorR, motorL);
  Serial.println("[lab3] Listo. Esperando al agente ...");
}

void loop() {
  rosSpin();
  update_led();
  vTaskDelay(1);   // cede la CPU un tick para no acaparar el core
}
