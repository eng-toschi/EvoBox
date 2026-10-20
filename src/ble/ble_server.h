// Camada BLE (SPEC §4): identidade, advertising, GATT (UART + DIS), notify
// com fragmentação por MTU, RX com FrameAssembler.
#pragma once

#include <Arduino.h>

#include "frame.h"

namespace ble {

struct Status {
  bool connected = false;
  bool subscribed = false;  // CCCD do TX com notify
  uint16_t conn_handle = 0xFFFF;
  uint16_t mtu = 23;
  uint16_t conn_interval_x125 = 0;  // unidades de 1,25 ms
  bool advertising = false;
  bool adv_fast = false;
  char name[32] = {};
  char serial[11] = {};
  uint16_t mtu_limit = 0;  // teto local de MTU (console `ble mtu`), 0 = padrão
};

struct Stats {
  uint32_t connects = 0;
  uint32_t disconnects = 0;
  uint32_t notifies = 0;
  uint32_t notify_bytes = 0;
  uint32_t enomem_retries = 0;
  uint32_t rx_bytes = 0;
  uint32_t rx_frames = 0;
  uint32_t rx_bad = 0;
  uint32_t subscribe_ms = 0;        // millis() do último subscribe
  uint32_t first_live_after_sub_ms = 0;  // subscribe → 1º live notificado (A2)
};

// Handler de comando (task BLE). Responde via txq::push_response.
using CommandHandler = void (*)(const rb::Frame& f);

void start(CommandHandler handler);
void set_hw_revision(const char* hw);
// Sink do builder: entrega um frame live se houver cliente inscrito.
void on_live_frame(const uint8_t frame[88]);

Status status();
Stats stats();
void reset_stats();
// Teto de MTU local (para provar A5 com MTU 23). 0 = padrão (517).
void set_mtu_limit(uint16_t mtu);
void disconnect();

}  // namespace ble
