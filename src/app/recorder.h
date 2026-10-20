// Gravação standalone e filtros do vendor (SPEC §7.2, §7.4) — C++ puro com
// relógio injetado, testado em native (test_recorder).
//
// - start: grava 0x26 state=1 (sempre, vendor p.17) e começa (ou espera fix, b0).
// - b1 Stationary: com good fix, speed < limiar por `interval` s → pausa
//   (0x26 state=2 gravado); retoma no primeiro registro acima do limiar.
// - b2 No-Fix: sem good fix por `interval` s → pausa (state=2); retoma com fix.
// - Retomada após pausa NÃO grava marcador; só é reportada ao vivo (vendor p.18).
// - b3 Auto-shutdown: sem gravar nada por `interval` s e sem cliente BLE →
//   grava state=0 e pede desligamento. b4: só arma após ≥ 1 registro gravado.
// - Memória cheia durante a gravação: tenta gravar state=0, para; o live segue.
#pragma once

#include <cstdint>

#include "frame.h"
#include "messages.h"
#include "payload.h"

namespace rec {

class Sink {
 public:
  virtual ~Sink() = default;
  // Grava no log; false = memória cheia (ou erro).
  virtual bool store(uint8_t tag, const uint8_t* payload, uint8_t len) = 0;
  // 0x26 ao vivo para o cliente conectado.
  virtual void live_state(const rb::StateChange& s) = 0;
  // A gravação parou sozinha (memória cheia ou auto-shutdown).
  virtual void stopped_by_itself(bool memory_full) = 0;
  // Auto-shutdown: o dono faz flush e entra em deep sleep.
  virtual void shutdown() = 0;
};

enum class State : uint8_t { kStopped, kWaitingFix, kRunning, kPaused };
enum class PauseReason : uint8_t { kNone, kStationary, kNoFix };

struct Stats {
  uint32_t stored = 0;
  uint32_t markers = 0;
  uint32_t pauses = 0;
  uint32_t resumes = 0;
  uint32_t full_stops = 0;
};

class Recorder {
 public:
  explicit Recorder(Sink& sink) : sink_(sink) {}

  void start(const rb::RecordingConfig& c, uint32_t now_ms, bool client_connected);
  // Config diferente com a gravação ativa (o chamador garante que mudou).
  // Marcadores state=0 + state=1 só se a taxa mudou (SPEC §7.2).
  void reconfigure(const rb::RecordingConfig& c, uint32_t now_ms);
  void stop();
  void on_frame(const rb::LiveData& d, const uint8_t payload[rb::kLivePayloadLen], uint32_t now_ms,
                bool client_connected);
  // Avalia o auto-shutdown sem frame (ex.: GNSS mudo).
  void on_tick(uint32_t now_ms, bool client_connected);

  State state() const { return state_; }
  bool recording() const { return state_ != State::kStopped; }
  PauseReason pause_reason() const { return reason_; }
  const rb::RecordingConfig& config() const { return cfg_; }
  const Stats& stats() const { return stats_; }

 private:
  bool store_marker(uint8_t state);
  void live(uint8_t state);
  void pause(PauseReason r);
  void resume();
  bool store_record(const uint8_t* payload, uint32_t now_ms);
  void check_shutdown(uint32_t now_ms, bool client_connected);
  void stop_full();

  Sink& sink_;
  rb::RecordingConfig cfg_;
  State state_ = State::kStopped;
  PauseReason reason_ = PauseReason::kNone;
  Stats stats_;
  uint32_t nofix_since_ = 0;
  bool nofix_timing_ = false;
  uint32_t stat_since_ = 0;
  bool stat_timing_ = false;
  uint32_t idle_ref_ms_ = 0;  // último registro gravado ou fim da conexão BLE
  uint32_t records_since_enable_ = 0;
};

}  // namespace rec
