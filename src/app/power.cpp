#include "app/power.h"

#include <Arduino.h>
#include <esp_sleep.h>

#include "app/builder.h"
#include "app/device.h"
#include "app/log.h"
#include "ble/ble_server.h"
#include "config.h"
#include "storage/storage.h"

namespace power {
namespace {

// Padrões do LED (SPEC §9.4), em ordem de prioridade.
bool led_on(uint32_t t) {
  const storage::Info si = storage::info();
  if (si.download_active || si.erase_active) return (t / 100) % 2 == 0;  // flash contínuo
  if (device::recording()) {                                             // duplo flash a cada 2 s
    const uint32_t ph = t % 2000;
    return ph < 80 || (ph >= 200 && ph < 280);
  }
  const ble::Status b = ble::status();
  if (b.connected) return true;                                         // aceso
  if (builder::stats().last_good_fix) return (t / 125) % 2 == 0;        // rápido: com fix
  return (t / 500) % 2 == 0;                                             // lento: sem fix
}

void shutdown() {
  EVO_LOG("power", "flushing log and entering deep sleep (wake: BOOT button / GPIO%d)", pins::kButton);
  storage::flush_now(3000);
  ble::disconnect();
  digitalWrite(pins::kLed, LOW);
  Serial.flush();
  esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(pins::kButton), 0);
  esp_deep_sleep_start();
}

void task(void*) {
  uint32_t last_tick = 0;
  for (;;) {
    const uint32_t now = millis();
    digitalWrite(pins::kLed, led_on(now) ? HIGH : LOW);
    if (now - last_tick >= 1000) {
      last_tick = now;
      device::tick();
    }
    if (device::shutdown_requested()) shutdown();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

}  // namespace

void start() {
  pinMode(pins::kLed, OUTPUT);
  pinMode(pins::kButton, INPUT_PULLUP);
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) EVO_LOG("power", "woke up from auto-shutdown (button)");
  xTaskCreatePinnedToCore(task, "power", 3072, nullptr, 1, nullptr, 0);
}

}  // namespace power
