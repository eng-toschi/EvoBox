// Modo simulação em runtime (SPEC §9.3): task que injeta NAV-PVT sintético na
// pvt_q no lugar do receptor (`-DGA_SIM=1` liga no boot; console `sim on|off`).
#pragma once

#include <Arduino.h>
#include <freertos/queue.h>

#include "app/sim.h"
#include "payload.h"

namespace sim_rt {

void start(QueueHandle_t pvt_q, bool enabled);
void set_enabled(bool on);
bool active();
void set_track(const sim::TrackConfig& c);
sim::TrackConfig track();
uint32_t generated();
uint32_t dropped();
// Motion source do builder enquanto a simulação está ativa.
bool motion_for_epoch(const uint8_t* pvt, uint32_t rx_ms, rb::Motion* out);

}  // namespace sim_rt
