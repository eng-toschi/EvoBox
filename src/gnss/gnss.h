// Task GNSS (SPEC §9.1): UART2 → parser UBX → NAV-PVT → fila pvt_q.
// Detecta o módulo (UBX-MON-VER), configura (SPEC §5.4) e aplica a taxa
// (SPEC §5.3). Nunca espera BLE nem flash.
#pragma once

#include <Arduino.h>
#include <freertos/queue.h>

#include "gnss/ublox_config.h"
#include "payload.h"

namespace gnss {

// Item da pvt_q: NAV-PVT cru de uma época real do receptor.
struct Pvt {
  uint8_t raw[rb::kNavPvtLen];
  uint32_t rx_ms;  // millis() na chegada do último byte
};

struct Status {
  ubx::ModuleInfo info;
  ubx::GnssMask gnss;
  bool present = false;
  bool configured = false;
  uint32_t baud = 0;
  uint32_t bringup_ms = 0;     // boot → primeiro NAV-PVT configurado (D10)
  uint32_t first_pvt_ms = 0;   // millis() do primeiro NAV-PVT
  uint8_t requested_code = 0;
  uint8_t effective_code = 0;
  uint16_t period_ms = 0;
  bool limited = false;
  uint8_t cap_hz = 0;
  uint8_t dyn_model = 4;
  bool saved_to_module = false;  // CFG-CFG/BBR gravado neste boot
};

struct Stats {
  uint32_t pvt = 0;           // NAV-PVT recebidos
  uint32_t pvt_dropped = 0;   // pvt_q cheia (timeout 0)
  uint32_t bad_checksum = 0;  // do assembler
  uint32_t noise_bytes = 0;
  uint32_t dt_on_period = 0;  // Δ iTOW == período configurado
  uint32_t dt_off_period = 0; // Δ iTOW ≠ período (exclui o 1º após mudança de taxa)
  uint32_t gaps = 0;          // Δ iTOW > 1,5 × período
  uint32_t dt_min = 0;
  uint32_t dt_max = 0;
  uint32_t last_itow = 0;
  uint32_t rate_changes = 0;
  uint32_t rx_jitter_max_ms = 0;  // |Δ chegada − período|, máx.
};

void start(QueueHandle_t pvt_q, uint8_t rate_code, uint8_t cap_hz, uint8_t dyn_model);

// Pedidos assíncronos, aplicados pela task GNSS (dona da UART).
void request_rate(uint8_t code);
void request_cap(uint8_t cap_hz);
void request_dyn_model(uint8_t model);

Status status();
Stats stats();
void reset_stats();
void set_log_pvt(bool on);
// Simulação ativa: NAV-PVT real continua contado nas estatísticas, mas não
// vai para a pvt_q (nunca mistura dado real com sintético).
void set_suppress_pvt(bool on);

}  // namespace gnss
