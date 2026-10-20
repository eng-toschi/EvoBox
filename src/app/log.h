// Log no console serial com carimbo de tempo (SPEC §9.2).
#pragma once

#include <Arduino.h>

#define EVO_LOG(tag, fmt, ...)                                                        \
  Serial.printf("[%7lu.%03lu] " tag ": " fmt "\n", (unsigned long)(millis() / 1000), \
                (unsigned long)(millis() % 1000), ##__VA_ARGS__)
