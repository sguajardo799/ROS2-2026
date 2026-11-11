// ===== Compatibilidad de time_t entre micro_ros_arduino y Arduino-ESP32 core 3.x =====
//
// libmicroros.a (micro_ros_arduino 2.0.8-jazzy) está precompilada con ESP-IDF 4.4
// (GCC 8.4, time_t de 32 bits), pero el core 3.x usa ESP-IDF 5 (time_t de 64 bits).
// La librería lee struct timespec como {int32 sec; int32 nsec}, y la de IDF 5 es
// {int64 sec; long nsec}: interpreta la parte alta de tv_sec como tv_nsec (= 0),
// así que su reloj avanza en saltos de 1 s. Consecuencias: timers rclc a 1 Hz,
// rclc_executor_spin_some bloqueando ~1 s, timeouts y sincronización de tiempo incorrectos.
//
// Solución: en libmicroros.a se renombra la referencia a clock_gettime por
// uros_clock_gettime (objcopy --redefine-sym, ver README) y esta función entrega
// el struct con el layout de 32 bits que la librería espera.
// Si la librería no está parcheada, esta función simplemente no se usa.

#include <time.h>
#include <stdint.h>

struct timespec32 {
  int32_t tv_sec;
  int32_t tv_nsec;
};

extern "C" int uros_clock_gettime(clockid_t clock_id, struct timespec32 *tp) {
  struct timespec ts;
  int rc = clock_gettime(clock_id, &ts);
  if (rc == 0 && tp != nullptr) {
    tp->tv_sec = (int32_t)ts.tv_sec;   // reloj desde el arranque: no hay riesgo de 2038
    tp->tv_nsec = (int32_t)ts.tv_nsec;
  }
  return rc;
}
