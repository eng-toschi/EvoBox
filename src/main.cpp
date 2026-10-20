// EvoBox — boot e criação das tasks (SPEC §9.1).
#include <Arduino.h>

#include "app/builder.h"
#include "app/console.h"
#include "app/console_cmds.h"
#include "app/log.h"
#include "app/settings.h"
#include "ble/ble_server.h"
#include "ble/tx_queue.h"
#include "messages.h"
#include "config.h"
#include "gnss/gnss.h"

namespace {
QueueHandle_t g_pvt_q = nullptr;
bool g_hw_rev_set = false;

void live_to_ble(const uint8_t frame[builder::kFrameLen], const rb::LiveData&, uint32_t) {
  ble::on_live_frame(frame);
}

// F2: os comandos Mini S chegam na F4 — até lá todo id é desconhecido → NACK (D6).
void command_handler(const rb::Frame& f) {
  EVO_LOG("cmd", "0x%02X 0x%02X len=%u -> NACK (commands not implemented before F4)", f.cls, f.id,
          f.len);
  uint8_t b[16];
  const size_t n = rb::encode_ack(false, f.cls, f.id, b, sizeof b);
  txq::push_response(b, n);
}
}  // namespace

void setup() {
  Serial.begin(cfg::kConsoleBaud);
  EVO_LOG("boot", "EvoBox %s (sim=%d)", EVOBOX_FW_VERSION, GA_SIM);
  settings::begin();

  const rb::GnssConfig gcfg = settings::gnss_config();
  g_pvt_q = xQueueCreate(8, sizeof(gnss::Pvt));
  builder::set_gnss_config(gcfg);
  builder::add_sink(live_to_ble);
  builder::start(g_pvt_q);
  ble::start(command_handler);
  gnss::start(g_pvt_q, GA_DEFAULT_LIVE_RATE, settings::max_rate_hz(), gcfg.dyn_model);

  console_cmds::register_base();
}

void loop() {
  console::poll();
  if (!g_hw_rev_set) {
    const gnss::Status g = gnss::status();
    if (g.present) {
      // DIS Hardware Revision conforme o módulo detectado (SPEC §4.2).
      char hw[24];
      snprintf(hw, sizeof hw, "GA-ESP32-%s", g.info.module == ubx::Module::kM9    ? "M9N"
                                             : g.info.module == ubx::Module::kM10 ? "M10"
                                                                                  : "M8N");
      ble::set_hw_revision(hw);
      g_hw_rev_set = true;
    }
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}
