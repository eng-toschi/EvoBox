// RaceBox BLE Protocol Rev 8 — framing (SPEC §5.1).
// C++ puro: sem Arduino, sem FreeRTOS, sem NimBLE (CLAUDE.md regra 4).
#pragma once

#include <cstddef>
#include <cstdint>

namespace rb {

constexpr uint8_t kSync1 = 0xB5;
constexpr uint8_t kSync2 = 0x62;
constexpr uint8_t kClass = 0xFF;  // toda a classe RaceBox

// Vendor: payload 0..504 B (buffer interno de 512 B por pacote).
constexpr size_t kMaxPayload = 504;
constexpr size_t kOverhead = 8;  // sync(2) + class/id(2) + len(2) + ck(2)
constexpr size_t kMaxFrame = kMaxPayload + kOverhead;

namespace msg {
constexpr uint8_t kLive = 0x01;
constexpr uint8_t kAck = 0x02;
constexpr uint8_t kNack = 0x03;
constexpr uint8_t kHistory = 0x21;
constexpr uint8_t kStatus = 0x22;
constexpr uint8_t kDownload = 0x23;
constexpr uint8_t kErase = 0x24;
constexpr uint8_t kRecConfig = 0x25;
constexpr uint8_t kStateChange = 0x26;
constexpr uint8_t kGnssConfig = 0x27;
constexpr uint8_t kUnlock = 0x30;
}  // namespace msg

struct Checksum {
  uint8_t a;
  uint8_t b;
};

// Fletcher-8 do UBX sobre `len` bytes (class..fim do payload).
inline Checksum fletcher8(const uint8_t* data, size_t len) {
  uint8_t a = 0, b = 0;
  for (size_t i = 0; i < len; ++i) {
    a = static_cast<uint8_t>(a + data[i]);
    b = static_cast<uint8_t>(b + a);
  }
  return {a, b};
}

// Monta um frame completo em `out`. Retorna o tamanho total (len + 8) ou 0 se
// não couber em `cap` ou se o payload exceder kMaxPayload.
size_t encode_frame(uint8_t cls, uint8_t id, const uint8_t* payload, size_t len,
                    uint8_t* out, size_t cap);

// Valida um frame completo (sync, tamanho declarado == tamanho real, checksum).
bool validate_frame(const uint8_t* frame, size_t len);

// Frame decodificado (aponta para o buffer de origem; válido até ele mudar).
struct Frame {
  uint8_t cls;
  uint8_t id;
  const uint8_t* payload;
  uint16_t len;
};

// Little-endian helpers.
inline void put_u16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}
inline void put_u32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v >> 16);
  p[3] = static_cast<uint8_t>(v >> 24);
}
inline void put_i16(uint8_t* p, int16_t v) { put_u16(p, static_cast<uint16_t>(v)); }
inline void put_i32(uint8_t* p, int32_t v) { put_u32(p, static_cast<uint32_t>(v)); }
inline uint16_t get_u16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}
inline uint32_t get_u32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
inline int16_t get_i16(const uint8_t* p) { return static_cast<int16_t>(get_u16(p)); }
inline int32_t get_i32(const uint8_t* p) { return static_cast<int32_t>(get_u32(p)); }

}  // namespace rb
