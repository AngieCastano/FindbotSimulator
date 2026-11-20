#pragma once
// Avisos de texto por serial. Con micro-ROS (entorno esp32) el puerto USB lleva el protocolo
// binario XRCE-DDS: cualquier texto lo corrompería, así que los avisos se anulan.
#include <Arduino.h>

#ifndef FINDBOT_MICROROS
#define FINDBOT_MICROROS 0
#endif

#if FINDBOT_MICROROS
#define LOG_PRINTF(...) \
  do {                  \
  } while (0)
#else
#define LOG_PRINTF(...) Serial.printf(__VA_ARGS__)
#endif
