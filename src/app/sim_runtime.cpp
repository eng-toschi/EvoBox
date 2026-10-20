#include "app/sim_runtime.h"

#include "app/log.h"
#include "frame.h"
#include "gnss/gnss.h"
#include "gnss/ublox_config.h"
#include "itow.h"
#include "messages.h"

namespace sim_rt {
namespace {

QueueHandle_t g_q = nullptr;
volatile bool g_on = false;
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
sim::TrackSim g_track;
uint32_t g_generated = 0, g_dropped = 0;

constexpr uint16_t kWeek = 2441;  // data sintética: semana GPS de 2026-10-18

void task(void*) {
  bool was_on = false;
  uint32_t itow = 0;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    if (!g_on) {
      was_on = false;
      vTaskDelay(pdMS_TO_TICKS(50));
      wake = xTaskGetTickCount();
      continue;
    }
    // Sem limite de módulo na simulação; o teto max_rate vale (SPEC §5.3).
    const gnss::Status gs = gnss::status();
    const ubx::RateDecision d = ubx::decide_rate(gs.requested_code, 25, gs.cap_hz);
    const uint16_t period = d.period_ms ? d.period_ms : 40;
    if (!was_on) {
      was_on = true;
      // Continua o iTOW do receptor para o fio seguir monotônico (R14).
      const uint32_t last = gnss::stats().last_itow;
      itow = ((last ? last : 100000) / period + 1) * period % rb::kMsPerWeek;
      portENTER_CRITICAL(&g_mux);
      g_track.reset(kWeek, itow);
      portEXIT_CRITICAL(&g_mux);
      EVO_LOG("sim", "simulation ON: %u Hz, circle r=%.0f m at %.1f m/s (lap %.2f s)", 1000 / period,
              g_track.config().radius_m, g_track.config().speed_mps, g_track.lap_seconds());
    }
    gnss::Pvt item;
    rb::Motion m;
    portENTER_CRITICAL(&g_mux);
    g_track.make(itow, item.raw, &m);
    portEXIT_CRITICAL(&g_mux);
    item.rx_ms = millis();
    if (xQueueSend(g_q, &item, 0) != pdTRUE) ++g_dropped;
    ++g_generated;
    itow = (itow + period) % rb::kMsPerWeek;
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(period));
  }
}

}  // namespace

void start(QueueHandle_t pvt_q, bool enabled) {
  g_q = pvt_q;
  set_enabled(enabled);
  xTaskCreatePinnedToCore(task, "sim", 3072, nullptr, 5, nullptr, 1);
}

void set_enabled(bool on) {
  g_on = on;
  gnss::set_suppress_pvt(on);  // receptor real fica configurado, mas fora da fila
  if (!on) EVO_LOG("sim", "simulation OFF");
}

bool active() { return g_on; }

void set_track(const sim::TrackConfig& c) {
  portENTER_CRITICAL(&g_mux);
  g_track.set_config(c);
  portEXIT_CRITICAL(&g_mux);
}

sim::TrackConfig track() {
  portENTER_CRITICAL(&g_mux);
  const sim::TrackConfig c = g_track.config();
  portEXIT_CRITICAL(&g_mux);
  return c;
}

uint32_t generated() { return g_generated; }
uint32_t dropped() { return g_dropped; }

bool motion_for_epoch(const uint8_t* pvt, uint32_t, rb::Motion* out) {
  uint8_t tmp[rb::kNavPvtLen];
  portENTER_CRITICAL(&g_mux);
  g_track.make(rb::get_u32(pvt), tmp, out);
  portEXIT_CRITICAL(&g_mux);
  return true;
}

}  // namespace sim_rt
