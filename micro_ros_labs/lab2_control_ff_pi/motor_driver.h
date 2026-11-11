#pragma once
#include <Arduino.h>
#include "config.h"

// ---------------------------------------------------------------------
//  MotorDriver: un canal del puente H TB6612FNG.
// ---------------------------------------------------------------------
//   IN1  IN2  PWM   -> resultado
//    H    L   duty  -> avanza  (duty = velocidad)
//    L    H   duty  -> retrocede
//    L    L    -    -> rueda libre (stop)
//
// La entrada es un "duty con signo" u en [-1, 1]:
//   u = 0.5 -> 50 % hacia adelante, u = -0.3 -> 30 % en reversa, u = 0 -> stop.
// Es la misma unidad que entrega el controlador FF+PI del Lab 2.
//
// Librería reutilizada sin cambios en los laboratorios 1, 2 y 3.
class MotorDriver {
public:
  void  begin(uint8_t pinPwm, uint8_t pinIn1, uint8_t pinIn2);
  void  setDuty(float u);   // satura a [-1, 1]; u == 0 deja la rueda libre
  void  stop();             // rueda libre, duty 0
  float duty() const { return duty_; }

private:
  uint8_t pwm_ = 0, in1_ = 0, in2_ = 0;
  float duty_ = 0;
};
