// Mensagens de comando Mini S (SPEC §7). Layouts: vendor Rev 8 p.9–21.
// C++ puro (CLAUDE.md regra 4).
#pragma once

#include <cstddef>
#include <cstdint>

namespace rb {

// --- Data rate (vendor §6.2 / SPEC §5.3) -----------------------------------
namespace rate {
constexpr uint8_t k25Hz = 0;
constexpr uint8_t k10Hz = 1;
constexpr uint8_t k5Hz = 2;
constexpr uint8_t k1Hz = 3;
constexpr uint8_t k20Hz = 4;
}  // namespace rate

inline bool valid_rate(uint8_t code) { return code <= rate::k20Hz; }
// Período de medição (CFG-RATE.measRate) para um código de taxa. 0 se inválido.
uint16_t rate_period_ms(uint8_t code);
// Taxa em Hz para um código. 0 se inválido.
uint8_t rate_hz(uint8_t code);
// Código para uma taxa em Hz (25/20/10/5/1). 0xFF se não houver código.
uint8_t rate_code_for_hz(uint8_t hz);

// --- 0x25 Standalone Recording Configuration --------------------------------
namespace filter {
constexpr uint8_t kWaitForFix = 1u << 0;
constexpr uint8_t kStationary = 1u << 1;
constexpr uint8_t kNoFix = 1u << 2;
constexpr uint8_t kAutoShutdown = 1u << 3;
constexpr uint8_t kWaitForData = 1u << 4;
}  // namespace filter

struct RecordingConfig {
  uint8_t enable = 0;
  uint8_t rate = rate::k25Hz;
  uint8_t flags = 0;
  uint16_t stationary_speed_mmps = 0;
  uint16_t stationary_interval_s = 0;
  uint16_t nofix_interval_s = 0;
  uint16_t shutdown_interval_s = 0;

  // Mesma configuração de gravação (tudo menos `enable`) — base da
  // idempotência do 0x25 (SPEC §7.2, R11).
  bool same_settings(const RecordingConfig& o) const {
    return rate == o.rate && flags == o.flags &&
           stationary_speed_mmps == o.stationary_speed_mmps &&
           stationary_interval_s == o.stationary_interval_s &&
           nofix_interval_s == o.nofix_interval_s &&
           shutdown_interval_s == o.shutdown_interval_s;
  }
  bool operator==(const RecordingConfig& o) const {
    return enable == o.enable && same_settings(o);
  }
  bool operator!=(const RecordingConfig& o) const { return !(*this == o); }
};

constexpr size_t kRecConfigLen = 12;
constexpr size_t kRecConfigShortLen = 10;  // vendor: "10 bytes" na prosa (SPEC §13 D14)

void encode_rec_config(const RecordingConfig& c, uint8_t out[kRecConfigLen]);

// Decodifica uma escrita de 0x25 (10 ou 12 B). Com 10 B, o intervalo de
// auto-shutdown (offset 10) não veio e é mantido de `current` (SPEC §13 D14).
// Retorna false se o tamanho não for 10/12. Não valida a taxa.
bool decode_rec_config(const uint8_t* p, size_t len, const RecordingConfig& current,
                       RecordingConfig* out);

// --- 0x22 Standalone Recording Status ---------------------------------------
struct RecordingStatus {
  uint8_t recording = 0;
  uint8_t memory_pct = 0;
  uint8_t security = 0;  // b0 enabled, b1 unlocked
  uint32_t stored = 0;
  uint32_t capacity = 0;
  bool operator==(const RecordingStatus& o) const {
    return recording == o.recording && memory_pct == o.memory_pct &&
           security == o.security && stored == o.stored && capacity == o.capacity;
  }
};
constexpr size_t kStatusLen = 12;
void encode_status(const RecordingStatus& s, uint8_t out[kStatusLen]);
RecordingStatus decode_status(const uint8_t in[kStatusLen]);

// Nível de memória em % (0..100) — 0 só se vazio, 100 só se cheio.
uint8_t memory_percent(uint32_t stored, uint32_t capacity);

// --- 0x26 Standalone Recording State Change ---------------------------------
namespace rec_state {
constexpr uint8_t kStopped = 0;
constexpr uint8_t kRunning = 1;
constexpr uint8_t kPaused = 2;
}  // namespace rec_state

// Atenção: a ordem difere do 0x25 (vendor p.18): state, reserved, rate, flags.
struct StateChange {
  uint8_t state = 0;
  uint8_t rate = 0;
  uint8_t flags = 0;
  uint16_t stationary_speed_mmps = 0;
  uint16_t stationary_interval_s = 0;
  uint16_t nofix_interval_s = 0;
  uint16_t shutdown_interval_s = 0;
  bool operator==(const StateChange& o) const {
    return state == o.state && rate == o.rate && flags == o.flags &&
           stationary_speed_mmps == o.stationary_speed_mmps &&
           stationary_interval_s == o.stationary_interval_s &&
           nofix_interval_s == o.nofix_interval_s &&
           shutdown_interval_s == o.shutdown_interval_s;
  }
};
constexpr size_t kStateChangeLen = 12;
StateChange make_state_change(uint8_t state, const RecordingConfig& c);
void encode_state_change(const StateChange& s, uint8_t out[kStateChangeLen]);
StateChange decode_state_change(const uint8_t in[kStateChangeLen]);

// --- 0x27 GNSS Receiver Configuration ---------------------------------------
// min_hacc em décimos de metro: o exemplo do vendor codifica 2,0 m como 0x14
// (SPEC §13 D15). Default de fábrica: automotive, ground speed, 3,0 m.
struct GnssConfig {
  uint8_t dyn_model = 4;
  uint8_t speed_3d = 0;
  uint8_t min_hacc_dm = 30;
  bool operator==(const GnssConfig& o) const {
    return dyn_model == o.dyn_model && speed_3d == o.speed_3d &&
           min_hacc_dm == o.min_hacc_dm;
  }
};
constexpr size_t kGnssConfigLen = 3;
constexpr uint8_t kMaxDynModel = 8;
void encode_gnss_config(const GnssConfig& c, uint8_t out[kGnssConfigLen]);
GnssConfig decode_gnss_config(const uint8_t in[kGnssConfigLen]);
inline bool valid_gnss_config(const GnssConfig& c) { return c.dyn_model <= kMaxDynModel; }

// --- ACK / NACK ---------------------------------------------------------------
// Frame completo (10 B) de ACK/NACK referenciando class/id. Retorna o tamanho.
size_t encode_ack(bool positive, uint8_t acked_class, uint8_t acked_id, uint8_t* out,
                  size_t cap);

}  // namespace rb
