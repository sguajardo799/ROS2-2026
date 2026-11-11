#pragma once
#include <Arduino.h>
#include "config.h"

// ---------------------------------------------------------------------
//  SpeedEstimator: velocidad de la rueda [RPM] a partir de las cuentas.
// ---------------------------------------------------------------------
// Diferencia de posición en una ventana deslizante de N muestras:
//
//            cuentas[k] - cuentas[k-N]         60
//   RPM  =  ---------------------------  x  -------
//            CPR x reducción (= 1400)        Δt [s]
//
// - Δt se MIDE (esp_timer, en µs); no se supone que el período es exacto.
// - N = VEL_WINDOW_SAMPLES (config.h). Una ventana más larga reduce el ruido
//   de cuantización (1 cuenta = pocos RPM), pero agrega retardo.
//   Resolución ≈ 60 / (1400 x Δt_ventana): 0.86 RPM con 50 ms, 1.07 RPM con 40 ms.
//
// Librería reutilizada sin cambios en los laboratorios 1, 2 y 3.
class SpeedEstimator {
public:
  // Llena el historial con la posición actual (velocidad inicial = 0).
  void  begin(int32_t count, int64_t nowUs);
  // Agrega una muestra y devuelve la velocidad estimada en RPM de la rueda.
  float update(int32_t count, int64_t nowUs);
  float rpm() const { return rpm_; }

private:
  static constexpr float kCountsPerWheelRev = ENCODER_CPR * GEAR_RATIO;
  int32_t counts_[VEL_WINDOW_SAMPLES + 1] = {0};
  int64_t times_[VEL_WINDOW_SAMPLES + 1] = {0};
  uint8_t idx_ = 0;
  float   rpm_ = 0;
};
