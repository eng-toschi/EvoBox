// EvoBox — boot e criação das tasks (SPEC §9.1).
#include <Arduino.h>

#include "app/builder.h"
#include "app/console.h"
#include "app/console_cmds.h"
#include "app/device.h"
#include "app/log.h"
#include "app/power.h"
#include "app/settings.h"
#include "app/sim_runtime.h"
#include "ble/ble_server.h"
#include "config.h"
#include "gnss/gnss.h"
#include "imu/imu.h"
#include "storage/storage.h"

namespace {
QueueHandle_t g_pvt_q = nullptr;
bool g_hw_rev_set = false;

// Motion source: simulação (IMU coerente com a pista) ou MPU6050 real.
bool motion(const uint8_t* pvt, uint32_t rx_ms, rb::Motion* out) {
  if (sim_rt::active()) return sim_rt::motion_for_epoch(pvt, rx_ms, out);
  if (imu::present()) return imu::motion_for_epoch(pvt, rx_ms, out);
  *out = rb::Motion{};
  out->gz_mg = 1000.0f;  // sem IMU: parado e nivelado
  return true;
}

void live_to_ble(const uint8_t frame[builder::kFrameLen], const rb::LiveData&, uint32_t) {
  ble::on_live_frame(frame);
}

}  // namespace

void setup() {
  Serial.begin(cfg::kConsoleBaud);
  EVO_LOG("boot", "EvoBox %s (sim=%d)", EVOBOX_FW_VERSION, GA_SIM);
  settings::begin();

  const rb::GnssConfig gcfg = settings::gnss_config();
  g_pvt_q = xQueueCreate(8, sizeof(gnss::Pvt));
  builder::set_gnss_config(gcfg);
  imu::start();
  builder::set_motion_source(motion);
  storage::start();
  builder::add_sink(live_to_ble);
  builder::add_sink(device::on_live);
  ble::start(device::on_command);
  device::start();  // retoma gravação salva (precisa de storage e GNSS de pé)
  builder::start(g_pvt_q);
  const uint8_t rate = device::recording() ? device::rec_config().rate : device::default_live_rate();
  gnss::start(g_pvt_q, rate, settings::max_rate_hz(), gcfg.dyn_model);
  sim_rt::start(g_pvt_q, GA_SIM != 0);

  power::start();
  console_cmds::register_base();
}

void loop() {
  console::poll();
  imu::service_persistence();
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
