#include "app/recorder.h"

namespace rec {

using rb::filter::kAutoShutdown;
using rb::filter::kNoFix;
using rb::filter::kStationary;
using rb::filter::kWaitForData;
using rb::filter::kWaitForFix;

bool Recorder::store_marker(uint8_t state) {
  uint8_t p[rb::kStateChangeLen];
  rb::encode_state_change(rb::make_state_change(state, cfg_), p);
  const bool ok = sink_.store(rb::msg::kStateChange, p, sizeof p);
  if (ok) ++stats_.markers;
  return ok;
}

void Recorder::live(uint8_t state) { sink_.live_state(rb::make_state_change(state, cfg_)); }

void Recorder::start(const rb::RecordingConfig& c, uint32_t now_ms, bool client_connected) {
  cfg_ = c;
  cfg_.enable = 1;
  reason_ = PauseReason::kNone;
  nofix_timing_ = stat_timing_ = false;
  records_since_enable_ = 0;
  idle_ref_ms_ = now_ms;
  (void)client_connected;
  state_ = (cfg_.flags & kWaitForFix) ? State::kWaitingFix : State::kRunning;
  if (!store_marker(rb::rec_state::kRunning)) {
    stop_full();
    return;
  }
  live(rb::rec_state::kRunning);
}

void Recorder::reconfigure(const rb::RecordingConfig& c, uint32_t now_ms) {
  if (state_ == State::kStopped) return;
  const bool rate_changed = c.rate != cfg_.rate;
  if (rate_changed) {
    if (!store_marker(rb::rec_state::kStopped)) {
      stop_full();
      return;
    }
    live(rb::rec_state::kStopped);
  }
  cfg_ = c;
  cfg_.enable = 1;
  // Pausa por um filtro que deixou de existir: retoma.
  if (state_ == State::kPaused &&
      ((reason_ == PauseReason::kStationary && !(cfg_.flags & kStationary)) ||
       (reason_ == PauseReason::kNoFix && !(cfg_.flags & kNoFix)))) {
    state_ = State::kRunning;
    reason_ = PauseReason::kNone;
    if (!rate_changed) live(rb::rec_state::kRunning);
  }
  nofix_timing_ = stat_timing_ = false;
  idle_ref_ms_ = now_ms;
  if (rate_changed) {
    if (!store_marker(rb::rec_state::kRunning)) {
      stop_full();
      return;
    }
    live(rb::rec_state::kRunning);
  }
}

void Recorder::stop() {
  if (state_ == State::kStopped) return;
  store_marker(rb::rec_state::kStopped);  // se não couber, para assim mesmo
  live(rb::rec_state::kStopped);
  state_ = State::kStopped;
  reason_ = PauseReason::kNone;
  cfg_.enable = 0;
}

void Recorder::stop_full() {
  store_marker(rb::rec_state::kStopped);  // "se couber" (SPEC §7.2)
  live(rb::rec_state::kStopped);
  state_ = State::kStopped;
  reason_ = PauseReason::kNone;
  cfg_.enable = 0;
  ++stats_.full_stops;
  sink_.stopped_by_itself(true);
}

void Recorder::pause(PauseReason r) {
  store_marker(rb::rec_state::kPaused);
  live(rb::rec_state::kPaused);
  state_ = State::kPaused;
  reason_ = r;
  nofix_timing_ = stat_timing_ = false;
  ++stats_.pauses;
}

void Recorder::resume() {
  live(rb::rec_state::kRunning);  // só ao vivo: o próximo registro gravado indica a retomada
  state_ = State::kRunning;
  reason_ = PauseReason::kNone;
  ++stats_.resumes;
}

bool Recorder::store_record(const uint8_t* payload, uint32_t now_ms) {
  if (!sink_.store(rb::msg::kHistory, payload, rb::kLivePayloadLen)) {
    stop_full();
    return false;
  }
  ++stats_.stored;
  ++records_since_enable_;
  idle_ref_ms_ = now_ms;
  return true;
}

void Recorder::on_frame(const rb::LiveData& d, const uint8_t* payload, uint32_t now_ms,
                        bool client_connected) {
  if (state_ == State::kStopped) return;
  const bool good = rb::good_fix(d);
  const bool slow = d.speed_mmps < static_cast<int32_t>(cfg_.stationary_speed_mmps);

  if (state_ == State::kWaitingFix) {
    if (!good) {
      check_shutdown(now_ms, client_connected);
      return;
    }
    state_ = State::kRunning;
  }

  if (state_ == State::kPaused) {
    const bool resume_now = (reason_ == PauseReason::kStationary) ? (good && !slow) : good;
    if (!resume_now) {
      check_shutdown(now_ms, client_connected);
      return;
    }
    resume();
  }

  // kRunning: filtros de pausa.
  if (cfg_.flags & kNoFix) {
    if (!good) {
      if (!nofix_timing_) {
        nofix_timing_ = true;
        nofix_since_ = now_ms;
      }
      if (now_ms - nofix_since_ >= cfg_.nofix_interval_s * 1000u) {
        pause(PauseReason::kNoFix);
        check_shutdown(now_ms, client_connected);
        return;
      }
    } else {
      nofix_timing_ = false;
    }
  }
  if (cfg_.flags & kStationary) {
    if (good && slow) {  // o filtro só age com fix válido
      if (!stat_timing_) {
        stat_timing_ = true;
        stat_since_ = now_ms;
      }
      if (now_ms - stat_since_ >= cfg_.stationary_interval_s * 1000u) {
        pause(PauseReason::kStationary);
        check_shutdown(now_ms, client_connected);
        return;
      }
    } else {
      stat_timing_ = false;
    }
  }
  if (!store_record(payload, now_ms)) return;
  check_shutdown(now_ms, client_connected);
}

void Recorder::on_tick(uint32_t now_ms, bool client_connected) {
  if (state_ == State::kStopped) return;
  check_shutdown(now_ms, client_connected);
}

void Recorder::check_shutdown(uint32_t now_ms, bool client_connected) {
  if (!(cfg_.flags & kAutoShutdown) || state_ == State::kStopped) return;
  if (client_connected) {
    idle_ref_ms_ = now_ms;  // com cliente conectado o timer não corre
    return;
  }
  if ((cfg_.flags & kWaitForData) && records_since_enable_ == 0) return;
  if (now_ms - idle_ref_ms_ < cfg_.shutdown_interval_s * 1000u) return;
  store_marker(rb::rec_state::kStopped);  // vendor grava marcador no auto-shutdown
  state_ = State::kStopped;
  reason_ = PauseReason::kNone;
  sink_.shutdown();
}

}  // namespace rec
