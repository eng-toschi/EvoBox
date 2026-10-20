// Task IMU (SPEC §6, §9.1): FIFO do MPU a 1 kHz, acumuladores por janela;
// a janela é fechada pelo builder a cada NAV-PVT (média por época).
#pragma once

#include <Arduino.h>

#include "imu/calibration.h"
#include "payload.h"

namespace imu {

struct Stats {
  bool present = false;
  uint8_t who_am_i = 0;
  uint32_t samples = 0;
  uint32_t fifo_overflows = 0;
  uint32_t empty_windows = 0;
  uint32_t last_window_n = 0;
  uint32_t min_window_n = 0;
  uint32_t max_window_n = 0;
  float rate_hz = 0;  // amostras/s medidas
};

struct CalStatus {
  cal::Level level = cal::kNone;
  cal::Blob blob;
  cal::DynamicStats dyn;
  bool static_running = false;
  float static_seconds = 0;
  bool auto_static = false;  // sem calibração salva: estática automática ao ficar parado
};

void start();
bool present();

// Motion source do builder: fecha a janela da época, aplica R_dv e a convenção
// do fio, alimenta a calibração dinâmica. false = janela vazia.
bool motion_for_epoch(const uint8_t* pvt, uint32_t rx_ms, rb::Motion* out);

// Console.
void request_static_cal();
void request_cal_reset();
CalStatus cal_status();
Stats stats();
void reset_stats();
// Última média de janela no frame do device (g, °/s) — diagnóstico.
void last_device_window(cal::Vec3* acc_g, cal::Vec3* gyro_dps);

// Persistência pendente (NVS) — chamado de uma task de baixa prioridade.
void service_persistence();

}  // namespace imu
