#include "encoder.h"

// Configuración de cuadratura x4 tomada del ejemplo "rotary_encoder" de ESP-IDF.
// Si una llamada falla, begin() devuelve false (el .ino decide cómo avisar).
#define ENC_CHECK(x) do { if ((x) != ESP_OK) return false; } while (0)

bool QuadratureEncoder::begin(uint8_t pinA, uint8_t pinB, int8_t dir, uint32_t glitchNs) {
  dir_ = dir;
  pcnt_unit_handle_t unit = nullptr;

  // 1) Unidad de conteo. El registro del hardware es de 16 bits; con accum_count
  //    el driver acumula los desbordes y entrega un contador efectivo de 32 bits.
  //    Nota: el driver activa el pull-up interno, pero GPIO34-39 no lo tienen.
  pcnt_unit_config_t unit_cfg = {};
  unit_cfg.low_limit = -32768;
  unit_cfg.high_limit = 32767;
  unit_cfg.flags.accum_count = 1;
  ENC_CHECK(pcnt_new_unit(&unit_cfg, &unit));

  // 2) Filtro: pulsos más cortos que glitchNs se ignoran (ruido eléctrico).
  pcnt_glitch_filter_config_t filter_cfg = { .max_glitch_ns = glitchNs };
  ENC_CHECK(pcnt_unit_set_glitch_filter(unit, &filter_cfg));

  // 3) Dos canales: uno mira los flancos de A (con el nivel de B) y el otro al revés.
  pcnt_chan_config_t cha_cfg = {};
  cha_cfg.edge_gpio_num = pinA;
  cha_cfg.level_gpio_num = pinB;
  pcnt_channel_handle_t cha = nullptr;
  ENC_CHECK(pcnt_new_channel(unit, &cha_cfg, &cha));

  pcnt_chan_config_t chb_cfg = {};
  chb_cfg.edge_gpio_num = pinB;
  chb_cfg.level_gpio_num = pinA;
  pcnt_channel_handle_t chb = nullptr;
  ENC_CHECK(pcnt_new_channel(unit, &chb_cfg, &chb));

  // 4) Reglas de cuadratura: el nivel del otro canal decide si se suma o se resta.
  ENC_CHECK(pcnt_channel_set_edge_action(cha, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ENC_CHECK(pcnt_channel_set_level_action(cha, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ENC_CHECK(pcnt_channel_set_edge_action(chb, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ENC_CHECK(pcnt_channel_set_level_action(chb, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  // 5) accum_count necesita "watch points" en los límites para detectar los desbordes.
  ENC_CHECK(pcnt_unit_add_watch_point(unit, unit_cfg.high_limit));
  ENC_CHECK(pcnt_unit_add_watch_point(unit, unit_cfg.low_limit));

  ENC_CHECK(pcnt_unit_enable(unit));
  ENC_CHECK(pcnt_unit_clear_count(unit));
  ENC_CHECK(pcnt_unit_start(unit));

  unit_ = unit;  // sólo se guarda el handle si todo salió bien
  return true;
}

int32_t QuadratureEncoder::count() const {
  int value = 0;
  if (unit_) pcnt_unit_get_count(unit_, &value);  // thread-safe
  return (int32_t)value * dir_;
}
