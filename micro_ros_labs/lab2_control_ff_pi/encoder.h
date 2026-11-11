#pragma once
#include <Arduino.h>
#include <driver/pulse_cnt.h>

// ---------------------------------------------------------------------
//  QuadratureEncoder: encoder en cuadratura x4 contado por HARDWARE.
// ---------------------------------------------------------------------
// Usa el periférico PCNT (Pulse CouNTer) del ESP32: cuenta los flancos de
// los canales A y B sin interrupciones y sin usar la CPU. Esto es clave
// porque la CPU también atiende micro-ROS (y el WiFi en el Lab 3).
//
//   A: _|‾‾|__|‾‾|__        Cada flanco de A o de B suma o resta 1
//   B: __|‾‾|__|‾‾|_        según el nivel del otro canal -> x4.
//
// Librería reutilizada sin cambios en los laboratorios 1, 2 y 3.
// No imprime por Serial: en los Labs 1 y 2 el Serial lo usa micro-ROS.
class QuadratureEncoder {
public:
  // dir: +1 / -1 para invertir el sentido. Devuelve false si el PCNT falla.
  bool begin(uint8_t pinA, uint8_t pinB, int8_t dir, uint32_t glitchNs);

  // Cuentas acumuladas (32 bits, con signo). Se puede llamar desde cualquier tarea.
  int32_t count() const;

  bool ok() const { return unit_ != nullptr; }

private:
  pcnt_unit_handle_t unit_ = nullptr;
  int8_t dir_ = 1;
};
