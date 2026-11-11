#include "motor.h"

bool Motor::begin(const Pins &pins) {
  driver_.begin(pins.pwm, pins.in1, pins.in2);   // motor detenido
  bool ok = encoder_.begin(pins.encA, pins.encB, pins.encDir, ENC_GLITCH_NS);
  count_ = encoder_.count();
  estimator_.begin(count_, esp_timer_get_time());
  return ok;
}

void Motor::setTarget(float rpm) {
  targetRPM_ = rpm;
  lastCmdMs_ = millis();   // "el host sigue vivo"
}

void Motor::requestReset() {
  targetRPM_ = 0;
  resetRequested_ = true;
}

bool Motor::isActive() const {
  return targetRPM_ != 0.0f && (millis() - lastCmdMs_) <= CMD_TIMEOUT_MS;
}

void Motor::update(float dt) {
  // 1) Medir (siempre, incluso detenido: el feedback debe seguir publicándose)
  int32_t c = encoder_.count();
  measuredRPM_ = estimator_.update(c, esp_timer_get_time());
  count_ = c;

  // 2) Seguridad: reset pedido o watchdog vencido -> parada INMEDIATA (sin rampa)
  bool timedOut = (millis() - lastCmdMs_) > CMD_TIMEOUT_MS;
  if (resetRequested_ || timedOut) {
    resetRequested_ = false;
    controller_.reset();
    driver_.stop();
    duty_ = 0;
    return;
  }

  // 3) Controlar y actuar (igual que en el Lab 2)
  float u = controller_.update(targetRPM_, measuredRPM_, dt);
  driver_.setDuty(u);
  duty_ = u;
}
