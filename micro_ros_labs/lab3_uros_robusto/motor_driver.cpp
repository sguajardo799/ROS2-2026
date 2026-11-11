#include "motor_driver.h"

void MotorDriver::begin(uint8_t pinPwm, uint8_t pinIn1, uint8_t pinIn2) {
  pwm_ = pinPwm;
  in1_ = pinIn1;
  in2_ = pinIn2;
  ledcAttach(pwm_, PWM_FREQ_HZ, PWM_RESOLUTION);  // API LEDC del core ESP32 3.x
  pinMode(in1_, OUTPUT);
  pinMode(in2_, OUTPUT);
  stop();
}

void MotorDriver::stop() {
  digitalWrite(in1_, LOW);
  digitalWrite(in2_, LOW);
  ledcWrite(pwm_, 0);
  duty_ = 0;
}

void MotorDriver::setDuty(float u) {
  u = constrain(u, -1.0f, 1.0f);
  if (u == 0.0f) { stop(); return; }

  // El signo elige el sentido de giro; la magnitud, el ancho de pulso.
  digitalWrite(in1_, u > 0 ? HIGH : LOW);
  digitalWrite(in2_, u > 0 ? LOW : HIGH);
  ledcWrite(pwm_, (uint32_t)lroundf(fabsf(u) * PWM_MAX_DUTY));
  duty_ = u;
}
