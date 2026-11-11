#pragma once
#include <Arduino.h>
#include "config.h"
#include "encoder.h"
#include "motor_driver.h"
#include "speed_estimator.h"
#include "speed_controller.h"

// ---------------------------------------------------------------------
//  Motor: junta las librerías de los Labs 1 y 2 en un objeto por rueda.
// ---------------------------------------------------------------------
//   Motor = QuadratureEncoder + SpeedEstimator + SpeedController + MotorDriver
//           + watchdog de comandos + reset pedido desde otra tarea
//
// Con dos ruedas basta con dos objetos: motorR y motorL (ver .ino).
//
// Modelo de concurrencia (un único escritor por dato):
//   - setTarget(), requestReset():  tarea micro-ROS (loop)
//   - update():                     tarea de control (dueña del controlador)
//   - getters:                      cualquier tarea (lecturas de 32 bits)
//
// Librería nueva en el Lab 3.
class Motor {
public:
  struct Pins {
    uint8_t pwm, in1, in2;   // TB6612
    uint8_t encA, encB;      // encoder
    int8_t  encDir;          // +1 / -1
  };

  bool begin(const Pins &pins);   // false si el encoder no se pudo inicializar

  // Un paso del lazo (solo desde la tarea de control). dt en segundos.
  void update(float dt);

  // Nuevo setpoint [RPM]. Cada llamada RENUEVA el watchdog.
  void setTarget(float rpm);

  // Detiene el motor y limpia rampa e integral. Lo aplica update().
  void requestReset();

  float   measuredRPM() const { return measuredRPM_; }
  float   duty() const        { return duty_; }
  int32_t count() const       { return count_; }
  // Posición [rad] y velocidad [rad/s] de la rueda (para joint_states).
  float   positionRad() const { return count_ * (2.0f * PI / (ENCODER_CPR * GEAR_RATIO)); }
  float   velocityRadS() const{ return measuredRPM_ * (2.0f * PI / 60.0f); }
  // true si tiene un setpoint no nulo y el watchdog no venció.
  bool    isActive() const;

private:
  QuadratureEncoder encoder_;
  MotorDriver       driver_;
  SpeedEstimator    estimator_;
  SpeedController   controller_;

  // Escritos por la tarea micro-ROS
  volatile float    targetRPM_ = 0;
  volatile uint32_t lastCmdMs_ = 0;
  volatile bool     resetRequested_ = false;

  // Escritos por la tarea de control
  volatile float    measuredRPM_ = 0;
  volatile float    duty_ = 0;
  volatile int32_t  count_ = 0;
};
