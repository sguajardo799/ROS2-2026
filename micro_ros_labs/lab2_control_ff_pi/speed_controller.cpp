#include "speed_controller.h"

void SpeedController::reset() {
  ref_ = 0;
  integral_ = 0;
  ff_ = 0;
  p_ = 0;
}

float SpeedController::update(float setpointRPM, float measuredRPM, float dt) {
  // 0) Zona muerta: velocidades muy bajas no se pueden controlar bien
  //    (pocas cuentas por ventana y fricción estática), así que se tratan como 0.
  float target = (fabsf(setpointRPM) >= RPM_MIN) ? setpointRPM : 0.0f;

  // 1) Rampa: la referencia se acerca al setpoint con pendiente máxima
  //    MAX_ACCEL_RPM_S. Evita picos de corriente y que las ruedas patinen.
  float maxStep = MAX_ACCEL_RPM_S * dt;
  ref_ += constrain(target - ref_, -maxStep, maxStep);

  // 2) Parada: setpoint nulo y la rampa ya bajó del umbral -> rueda libre.
  if (target == 0.0f && fabsf(ref_) < RPM_MIN) {
    reset();
    return 0.0f;
  }

  // 3) Feedforward: modelo estático duty = f(RPM) identificado en el Lab 1.
  ff_ = (ref_ > 0 ? FF_STATIC : -FF_STATIC) + ref_ / MOTOR_MAX_RPM;

  // 4) PI sobre el error de velocidad.
  float error = ref_ - measuredRPM;
  p_ = CTRL_KP * error;
  float uUnsat = ff_ + p_ + integral_;
  float u = constrain(uUnsat, -1.0f, 1.0f);

  // 5) Anti-windup por integración condicional: si la salida está saturada y el
  //    error empuja hacia la MISMA saturación, el integral no crece (si creciera,
  //    después tardaría en "descargarse" y provocaría un sobreimpulso grande).
  bool saturating = (u != uUnsat) && ((uUnsat > 0) == (error > 0));
  if (!saturating) {
    integral_ += CTRL_KI * error * dt;
    integral_ = constrain(integral_, -CTRL_I_LIMIT, CTRL_I_LIMIT);
  }

  return u;
}
