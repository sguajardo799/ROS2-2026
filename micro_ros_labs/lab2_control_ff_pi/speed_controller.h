#pragma once
#include <Arduino.h>
#include "config.h"

// ---------------------------------------------------------------------
//  SpeedController: control de velocidad Feedforward + PI.
// ---------------------------------------------------------------------
// No toca hardware: recibe el setpoint y la velocidad medida, y devuelve
// el duty con signo u en [-1, 1] para MotorDriver::setDuty().
//
//   setpoint ──► [rampa] ──► ref ──┬──► [ FF ] ────────────┐
//                                  │                       ▼
//                                  └──►(+)── e ──► [ PI ] ──(+)──► [sat ±1] ──► u
//                                       ▲ -
//                         medida (RPM) ─┘
//
//   u = FF_STATIC·signo(ref) + ref/MOTOR_MAX_RPM  +  Kp·e  +  ∫ Ki·e dt
//       └──────────── feedforward ────────────┘    └────── PI ──────┘
//
// - Feedforward: lleva el motor "casi" a la velocidad pedida desde el primer
//   instante, usando la recta identificada en el Lab 1 (sin esperar error).
// - PI: corrige lo que el modelo no predice (carga, batería, fricción). Como el
//   FF hace la mayor parte del trabajo, Kp y Ki pueden ser pequeños.
// - Sin término D: amplificaría el ruido de cuantización del encoder.
//
// Librería nueva en el Lab 2; se reutiliza sin cambios en el Lab 3.
class SpeedController {
public:
  // Un paso del controlador. dt: tiempo real desde el paso anterior [s].
  // Devuelve 0 cuando el motor debe quedar detenido (rueda libre).
  float update(float setpointRPM, float measuredRPM, float dt);

  // Limpia la rampa y el integral (motor detenido).
  void reset();

  // Valores internos del último paso (para graficar y ajustar).
  float ref() const      { return ref_; }
  float ffTerm() const   { return ff_; }
  float pTerm() const    { return p_; }
  float integral() const { return integral_; }

private:
  float ref_ = 0;       // setpoint después de la rampa [RPM]
  float integral_ = 0;  // término integral ya multiplicado por Ki [fracción de duty]
  float ff_ = 0, p_ = 0;
};
