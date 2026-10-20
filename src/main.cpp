// EvoBox — boot e criação das tasks (SPEC §9.1).
#include <Arduino.h>

#include "app/builder.h"
#include "app/console.h"
#include "app/console_cmds.h"
#include "app/log.h"
#include "app/settings.h"
#include "config.h"
#include "gnss/gnss.h"

namespace {
QueueHandle_t g_pvt_q = nullptr;
}

void setup() {
  Serial.begin(cfg::kConsoleBaud);
  EVO_LOG("boot", "EvoBox %s (sim=%d)", EVOBOX_FW_VERSION, GA_SIM);
  settings::begin();

  const rb::GnssConfig gcfg = settings::gnss_config();
  g_pvt_q = xQueueCreate(8, sizeof(gnss::Pvt));
  builder::set_gnss_config(gcfg);
  builder::start(g_pvt_q);
  gnss::start(g_pvt_q, GA_DEFAULT_LIVE_RATE, settings::max_rate_hz(), gcfg.dyn_model);

  console_cmds::register_base();
}

void loop() {
  console::poll();
  vTaskDelay(pdMS_TO_TICKS(10));
}
