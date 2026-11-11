#!/usr/bin/env bash
# Parchea libmicroros.a (micro_ros_arduino) para Arduino-ESP32 core 3.x.
#
# La librería precompilada usa time_t de 32 bits (ESP-IDF 4.4) y el core 3.x usa 64 bits
# (ESP-IDF 5). Su reloj interno queda con resolución de 1 s: timers rclc a 1 Hz y
# spin_some bloqueando ~1 s. Este script redirige su clock_gettime a uros_clock_gettime,
# que está en microros_time_fix.cpp (incluido en cada laboratorio). Es idempotente y deja un respaldo .orig.
#
# Uso: micro_ros_labs/tools/patch_microros_time.sh [ruta/a/micro_ros_arduino]   (re-ejecutar tras actualizar la librería)
set -euo pipefail

LIB_DIR="${1:-$HOME/Arduino/libraries/micro_ros_arduino}"
LIB="$LIB_DIR/src/esp32/libmicroros.a"
TOOLS="$HOME/.arduino15/packages/esp32/tools"

[[ -f "$LIB" ]] || { echo "No se encontró $LIB" >&2; exit 1; }

OBJCOPY=$(find "$TOOLS" -name 'xtensa-esp32-elf-objcopy' -type f 2>/dev/null | sort | tail -1)
NM=$(find "$TOOLS" -name 'xtensa-esp32-elf-nm' -type f 2>/dev/null | sort | tail -1)
[[ -x "$OBJCOPY" && -x "$NM" ]] || { echo "No se encontró el toolchain xtensa-esp32-elf en $TOOLS" >&2; exit 1; }

if "$NM" -A "$LIB" 2>/dev/null | grep -q ' U uros_clock_gettime'; then
  echo "Ya parcheada: $LIB"
  exit 0
fi

[[ -f "$LIB.orig" ]] || cp "$LIB" "$LIB.orig"
"$OBJCOPY" --redefine-sym clock_gettime=uros_clock_gettime "$LIB"

echo "Parcheada: $LIB (respaldo en $LIB.orig)"
"$NM" -A "$LIB" | grep 'uros_clock_gettime' | sed 's/.*libmicroros.a:/  /'
