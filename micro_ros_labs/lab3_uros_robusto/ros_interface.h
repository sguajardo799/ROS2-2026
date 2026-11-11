#pragma once
#include "motor.h"

// ---------------------------------------------------------------------
//  Interfaz micro-ROS del Lab 3 (WiFi + máquina de estados del agente).
// ---------------------------------------------------------------------
// Toda la API de micro-ROS se usa desde UNA sola tarea: loop().
//
//   rosBegin(): configura el transporte WiFi (bloquea hasta conectar a la red).
//   rosSpin():  llamar en cada loop(). Detecta el agente, crea/destruye las
//               entidades y ejecuta el executor cuando hay conexión.
//
// Librería nueva en el Lab 3.
void rosBegin(Motor &right, Motor &left);
void rosSpin();
bool rosConnected();
