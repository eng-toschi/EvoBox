#include "frame.h"

#include <cstring>

namespace rb {

size_t encode_frame(uint8_t cls, uint8_t id, const uint8_t* payload, size_t len,
                    uint8_t* out, size_t cap) {
  if (len > kMaxPayload || cap < len + kOverhead) return 0;
  out[0] = kSync1;
  out[1] = kSync2;
  out[2] = cls;
  out[3] = id;
  put_u16(out + 4, static_cast<uint16_t>(len));
  if (len > 0) std::memcpy(out + 6, payload, len);
  const Checksum ck = fletcher8(out + 2, 4 + len);
  out[6 + len] = ck.a;
  out[7 + len] = ck.b;
  return len + kOverhead;
}

bool validate_frame(const uint8_t* frame, size_t len) {
  if (len < kOverhead || frame[0] != kSync1 || frame[1] != kSync2) return false;
  const size_t plen = get_u16(frame + 4);
  if (plen > kMaxPayload || plen + kOverhead != len) return false;
  const Checksum ck = fletcher8(frame + 2, 4 + plen);
  return ck.a == frame[6 + plen] && ck.b == frame[7 + plen];
}

}  // namespace rb
