// Modo simulação (SPEC §9.3) — gerador puro (C++ sem Arduino, test_sim):
// NAV-PVT sintético de uma pista circular com fix 3D e IMU coerente
// (aceleração centrípeta e yaw). Frames de simulação têm numSV = 0 e
// hAcc = 2,5 m (D21) para nunca serem confundidos com dado real.
#pragma once

#include <cstdint>

#include "payload.h"

namespace sim {

struct TrackConfig {
  double center_lat_deg = -23.703600;  // Interlagos (referência qualquer)
  double center_lon_deg = -46.699700;
  float radius_m = 100.0f;
  float speed_mps = 20.0f;
  bool clockwise = true;  // horário = curva à direita contínua
};

// Data/hora UTC de um instante GPS (semana + iTOW), com 18 s de leap.
struct Utc {
  uint16_t year;
  uint8_t month, day, hour, minute, second;
  int32_t nano;
};
Utc utc_from_gps(uint16_t week, uint32_t itow_ms);

class TrackSim {
 public:
  explicit TrackSim(const TrackConfig& c = TrackConfig{}) : cfg_(c) {}
  void set_config(const TrackConfig& c) { cfg_ = c; }
  const TrackConfig& config() const { return cfg_; }

  // Origem: o instante (semana GPS, iTOW) em que o carro está no ângulo 0.
  void reset(uint16_t gps_week, uint32_t itow0_ms) {
    week_ = gps_week;
    itow0_ = itow0_ms;
  }
  // NAV-PVT (92 B) e movimento no fio para o instante itow_ms.
  void make(uint32_t itow_ms, uint8_t pvt[rb::kNavPvtLen], rb::Motion* motion) const;
  float lap_seconds() const;

 private:
  TrackConfig cfg_;
  uint16_t week_ = 2441;  // 2026-10-18
  uint32_t itow0_ = 0;
};

}  // namespace sim
