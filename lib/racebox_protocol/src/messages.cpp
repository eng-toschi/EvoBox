#include "messages.h"

#include "frame.h"

namespace rb {

uint16_t rate_period_ms(uint8_t code) {
  switch (code) {
    case rate::k25Hz:
      return 40;
    case rate::k20Hz:
      return 50;
    case rate::k10Hz:
      return 100;
    case rate::k5Hz:
      return 200;
    case rate::k1Hz:
      return 1000;
    default:
      return 0;
  }
}

uint8_t rate_hz(uint8_t code) {
  const uint16_t p = rate_period_ms(code);
  return p == 0 ? 0 : static_cast<uint8_t>(1000 / p);
}

uint8_t rate_code_for_hz(uint8_t hz) {
  switch (hz) {
    case 25:
      return rate::k25Hz;
    case 20:
      return rate::k20Hz;
    case 10:
      return rate::k10Hz;
    case 5:
      return rate::k5Hz;
    case 1:
      return rate::k1Hz;
    default:
      return 0xFF;
  }
}

void encode_rec_config(const RecordingConfig& c, uint8_t* o) {
  o[0] = c.enable;
  o[1] = c.rate;
  o[2] = c.flags;
  o[3] = 0;  // reserved
  put_u16(o + 4, c.stationary_speed_mmps);
  put_u16(o + 6, c.stationary_interval_s);
  put_u16(o + 8, c.nofix_interval_s);
  put_u16(o + 10, c.shutdown_interval_s);
}

bool decode_rec_config(const uint8_t* p, size_t len, const RecordingConfig& current,
                       RecordingConfig* out) {
  if (len != kRecConfigLen && len != kRecConfigShortLen) return false;
  RecordingConfig c;
  c.enable = p[0];
  c.rate = p[1];
  c.flags = p[2];
  c.stationary_speed_mmps = get_u16(p + 4);
  c.stationary_interval_s = get_u16(p + 6);
  c.nofix_interval_s = get_u16(p + 8);
  c.shutdown_interval_s = len == kRecConfigLen ? get_u16(p + 10) : current.shutdown_interval_s;
  *out = c;
  return true;
}

void encode_status(const RecordingStatus& s, uint8_t* o) {
  o[0] = s.recording;
  o[1] = s.memory_pct;
  o[2] = s.security;
  o[3] = 0;
  put_u32(o + 4, s.stored);
  put_u32(o + 8, s.capacity);
}

RecordingStatus decode_status(const uint8_t* p) {
  RecordingStatus s;
  s.recording = p[0];
  s.memory_pct = p[1];
  s.security = p[2];
  s.stored = get_u32(p + 4);
  s.capacity = get_u32(p + 8);
  return s;
}

uint8_t memory_percent(uint32_t stored, uint32_t capacity) {
  if (capacity == 0 || stored >= capacity) return 100;
  if (stored == 0) return 0;
  // Piso, como o exemplo do vendor (67173/196608 → 34 %); 0 só vazio, 100 só cheio.
  const uint64_t pct = static_cast<uint64_t>(stored) * 100u / capacity;
  if (pct == 0) return 1;
  return static_cast<uint8_t>(pct >= 100 ? 99 : pct);
}

StateChange make_state_change(uint8_t state, const RecordingConfig& c) {
  StateChange s;
  s.state = state;
  s.rate = c.rate;
  s.flags = c.flags;
  s.stationary_speed_mmps = c.stationary_speed_mmps;
  s.stationary_interval_s = c.stationary_interval_s;
  s.nofix_interval_s = c.nofix_interval_s;
  s.shutdown_interval_s = c.shutdown_interval_s;
  return s;
}

void encode_state_change(const StateChange& s, uint8_t* o) {
  o[0] = s.state;
  o[1] = 0;  // reserved
  o[2] = s.rate;
  o[3] = s.flags;
  put_u16(o + 4, s.stationary_speed_mmps);
  put_u16(o + 6, s.stationary_interval_s);
  put_u16(o + 8, s.nofix_interval_s);
  put_u16(o + 10, s.shutdown_interval_s);
}

StateChange decode_state_change(const uint8_t* p) {
  StateChange s;
  s.state = p[0];
  s.rate = p[2];
  s.flags = p[3];
  s.stationary_speed_mmps = get_u16(p + 4);
  s.stationary_interval_s = get_u16(p + 6);
  s.nofix_interval_s = get_u16(p + 8);
  s.shutdown_interval_s = get_u16(p + 10);
  return s;
}

void encode_gnss_config(const GnssConfig& c, uint8_t* o) {
  o[0] = c.dyn_model;
  o[1] = c.speed_3d;
  o[2] = c.min_hacc_dm;
}

GnssConfig decode_gnss_config(const uint8_t* p) {
  GnssConfig c;
  c.dyn_model = p[0];
  c.speed_3d = p[1];
  c.min_hacc_dm = p[2];
  return c;
}

size_t encode_ack(bool positive, uint8_t acked_class, uint8_t acked_id, uint8_t* out,
                  size_t cap) {
  const uint8_t payload[2] = {acked_class, acked_id};
  return encode_frame(kClass, positive ? msg::kAck : msg::kNack, payload, 2, out, cap);
}

}  // namespace rb
