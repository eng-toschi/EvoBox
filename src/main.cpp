// EvoBox — boot e criação das tasks (SPEC §9.1).
#include <Arduino.h>

#include "config.h"

void setup() {
  Serial.begin(cfg::kConsoleBaud);
  Serial.printf("\nEvoBox %s boot\n", EVOBOX_FW_VERSION);
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
