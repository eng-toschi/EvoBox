// Live Data / History Data payload (0xFF 0x01 / 0xFF 0x21), 80 B — SPEC §5.2.
// Layout: vendor Rev 8 p.4–5. C++ puro (CLAUDE.md regra 4).
#pragma once

#include <cstddef>
#include <cstdint>

namespace rb {

constexpr size_t kLivePayloadLen = 80;
constexpr size_t kNavPvtLen = 92;  // UBX-NAV-PVT payload

// Campos do payload exatamente como estão no fio (inteiros, unidades do vendor).
struct LiveData {
  uint32_t itow = 0;            // 0   ms da semana GPS
  uint16_t year = 0;            // 4
  uint8_t month = 0;            // 6
  uint8_t day = 0;              // 7
  uint8_t hour = 0;             // 8
  uint8_t minute = 0;           // 9
  uint8_t second = 0;           // 10
  uint8_t validity = 0;         // 11  b0 date, b1 time, b2 fully resolved
  uint32_t time_acc_ns = 0;     // 12
  int32_t nano = 0;             // 16
  uint8_t fix_status = 0;       // 20  0 / 2 / 3
  uint8_t fix_flags = 0;        // 21  b0 gnssFixOK
  uint8_t datetime_flags = 0;   // 22
  uint8_t num_sv = 0;           // 23
  int32_t lon = 0;              // 24  1e-7 °
  int32_t lat = 0;              // 28  1e-7 °
  int32_t wgs_alt_mm = 0;       // 32
  int32_t msl_alt_mm = 0;       // 36
  uint32_t h_acc_mm = 0;        // 40
  uint32_t v_acc_mm = 0;        // 44
  int32_t speed_mmps = 0;       // 48
  int32_t heading = 0;          // 52  1e-5 °
  uint32_t speed_acc_mmps = 0;  // 56
  uint32_t heading_acc = 0;     // 60  1e-5 °
  uint16_t pdop = 0;            // 64  ×100
  uint8_t latlon_flags = 0;     // 66  b0 invalidLlh, b1–4 idade da correção
  uint8_t battery = 0;          // 67  b7 carregando, b0–6 %
  int16_t gx = 0;               // 68  milli-g
  int16_t gy = 0;               // 70
  int16_t gz = 0;               // 72
  int16_t rx = 0;               // 74  centi-°/s
  int16_t ry = 0;               // 76
  int16_t rz = 0;               // 78
};

void encode_live(const LiveData& d, uint8_t out[kLivePayloadLen]);
LiveData decode_live(const uint8_t in[kLivePayloadLen]);

// fixType do NAV-PVT → Fix Status do RaceBox: 2→2, 3→3, 4 (GNSS+DR)→3, demais→0.
uint8_t map_fix_type(uint8_t navpvt_fix_type);

// Int16 do fio: clamp, nunca wrap (CLAUDE.md regra 9).
int16_t clamp_i16(int32_t v);
// Arredonda ao inteiro mais próximo e faz clamp. NaN → 0.
int16_t wire_i16(float v);

// Movimento já no frame do veículo e na convenção do fio (§6.4), em unidades do
// fio: milli-g e centi-°/s. Valores fora de ±32767 saturam.
struct Motion {
  float gx_mg = 0.0f;
  float gy_mg = 0.0f;
  float gz_mg = 0.0f;
  float rx_cdps = 0.0f;
  float ry_cdps = 0.0f;
  float rz_cdps = 0.0f;
};

// Byte de bateria (§5.5). Sem medição real → 0x64 (100 %, não carregando);
// nunca 0 sem medição (CLAUDE.md regra 10).
uint8_t battery_byte(bool measured, uint8_t percent, bool charging);

struct BuildOptions {
  bool speed_3d = false;   // §7.6: offset 48 = |vel| 3D em vez de gSpeed
  uint8_t battery = 0x64;  // já codificado por battery_byte()
};

// "Good fix" do vendor (p.5): Fix Status 3 e Fix Status Flags b0.
inline bool good_fix(const LiveData& d) { return d.fix_status == 3 && (d.fix_flags & 0x01) != 0; }

// 0x27 "minimum horizontal accuracy" (vendor p.10, SPEC §7.6): com hAcc pior
// que o mínimo o device reporta perda de fix zerando o b0 de Fix Status Flags.
// min_hacc_dm em décimos de metro (D15).
void apply_min_hacc(LiveData& d, uint8_t min_hacc_dm);

// Monta o payload RaceBox a partir do NAV-PVT cru (92 B, layout u-blox M8/M9).
// O iTOW sai cru; a correção de monotonicidade (§5.2) é feita por quem chama.
LiveData live_from_navpvt(const uint8_t pvt[kNavPvtLen], const Motion& motion,
                          const BuildOptions& opt);

}  // namespace rb
