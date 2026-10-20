// Task builder (SPEC §9.1): consome pvt_q, fecha a janela IMU, monta o
// payload de 80 B e entrega aos sinks (BLE, gravação). Um frame por NAV-PVT.
#pragma once

#include <Arduino.h>
#include <freertos/queue.h>

#include "frame.h"
#include "messages.h"
#include "payload.h"

namespace builder {

constexpr size_t kFrameLen = rb::kLivePayloadLen + rb::kOverhead;  // 88

// Chamado na task builder para cada frame. Não pode bloquear.
using LiveSink = void (*)(const uint8_t frame[kFrameLen], const rb::LiveData& d, uint32_t rx_ms);
// Fecha a janela IMU da época e devolve a média no frame do fio. false =
// janela vazia (o builder repete o último valor e conta imu_stall).
using MotionSource = bool (*)(const uint8_t* pvt, uint32_t rx_ms, rb::Motion* out);

struct Stats {
  uint32_t frames = 0;
  uint32_t itow_time_resolved = 0;
  uint32_t itow_backsteps = 0;
  uint32_t imu_stall = 0;
  uint32_t build_us_max = 0;    // NAV-PVT na fila → sinks entregues
  uint32_t latency_ms_max = 0;  // chegada do NAV-PVT na UART → sinks entregues
  uint32_t last_itow_out = 0;
  bool last_good_fix = false;
};

void start(QueueHandle_t pvt_q);
void add_sink(LiveSink sink);
void set_motion_source(MotionSource src);
void set_gnss_config(const rb::GnssConfig& c);
Stats stats();
void reset_stats();
void set_log_frames(bool on);

}  // namespace builder
