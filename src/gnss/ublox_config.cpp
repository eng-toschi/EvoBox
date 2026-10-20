#include "ublox_config.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "frame.h"
#include "messages.h"

namespace ubx {

const char* module_name(Module m) {
  switch (m) {
    case Module::kM8:
      return "M8";
    case Module::kM9:
      return "M9";
    case Module::kM10:
      return "M10";
    default:
      return "unknown";
  }
}

static void copy_field(char* dst, size_t cap, const uint8_t* src, size_t n) {
  size_t k = 0;
  while (k < n && k + 1 < cap && src[k] != 0) {
    dst[k] = static_cast<char>(src[k]);
    ++k;
  }
  dst[k] = 0;
}

static bool starts_with(const char* s, const char* prefix) {
  return std::strncmp(s, prefix, std::strlen(prefix)) == 0;
}

bool parse_mon_ver(const uint8_t* p, size_t len, ModuleInfo* out) {
  if (len < 40) return false;
  ModuleInfo info;
  copy_field(info.sw, sizeof info.sw, p, 30);
  copy_field(info.hw, sizeof info.hw, p + 30, 10);
  for (size_t off = 40; off + 30 <= len; off += 30) {
    char ext[31];
    copy_field(ext, sizeof ext, p + off, 30);
    if (starts_with(ext, "MOD=")) {
      std::snprintf(info.mod, sizeof info.mod, "%s", ext + 4);
    } else if (starts_with(ext, "FWVER=")) {
      std::snprintf(info.fwver, sizeof info.fwver, "%s", ext + 6);
    } else if (starts_with(ext, "PROTVER=")) {
      // "18.00" / "32.01" → ×100 sem float
      const char* v = ext + 8;
      char* end = nullptr;
      const long major = std::strtol(v, &end, 10);
      long minor = 0;
      if (end && *end == '.') minor = std::strtol(end + 1, nullptr, 10);
      info.protver_x100 = static_cast<uint16_t>(major * 100 + (minor % 100));
    }
  }
  // Geração: pela string MOD quando existe; senão pela versão do protocolo.
  // hwVersion 000A0000 é comum a M9 e M10 (visto em bancada: M10 SPG 5.10
  // reporta 000A0000, sem MOD=) — por isso PROTVER decide: M8 ≤ 23.01,
  // M9 = 27..33 (SPG 4.x), M10 ≥ 34 (SPG 5.x).
  if (std::strstr(info.mod, "M10")) {
    info.module = Module::kM10;
  } else if (std::strstr(info.mod, "M9")) {
    info.module = Module::kM9;
  } else if (std::strstr(info.mod, "M8")) {
    info.module = Module::kM8;
  } else if (info.protver_x100 >= 3400 || starts_with(info.fwver, "SPG 5")) {
    info.module = Module::kM10;
  } else if (info.protver_x100 >= 2700 || starts_with(info.fwver, "SPG 4")) {
    info.module = Module::kM9;
  } else if (std::strcmp(info.hw, "00080000") == 0) {
    info.module = Module::kM8;
  }
  switch (info.module) {
    case Module::kM9:
      info.max_rate_hz = 25;
      info.uses_valset = true;
      break;
    case Module::kM10:
      info.max_rate_hz = 10;  // 4 GNSS (SPEC §2.2, ‹VALIDAR›)
      info.uses_valset = true;
      break;
    default:
      info.max_rate_hz = 10;  // M8 e desconhecido: config legada, 10 Hz
      info.uses_valset = info.protver_x100 >= 2700;
      break;
  }
  *out = info;
  return true;
}

bool parse_mon_gnss(const uint8_t* p, size_t len, GnssMask* out) {
  if (len < 8 || p[0] != 0) return false;
  out->supported = p[1];
  out->enabled = p[3];
  out->simultaneous = p[4];
  return true;
}

const char* gnss_mask_str(uint8_t mask, char* buf, size_t cap) {
  static const char* names[] = {"GPS", "GLO", "BDS", "GAL"};
  size_t n = 0;
  buf[0] = 0;
  for (int i = 0; i < 4; ++i) {
    if (!(mask & (1u << i))) continue;
    const int w = std::snprintf(buf + n, cap - n, "%s%s", n ? "+" : "", names[i]);
    if (w < 0 || static_cast<size_t>(w) >= cap - n) break;
    n += static_cast<size_t>(w);
  }
  return buf;
}

size_t poll(uint8_t cls, uint8_t id, uint8_t* out, size_t cap) {
  return rb::encode_frame(cls, id, nullptr, 0, out, cap);
}

size_t cfg_prt_uart1(uint32_t baud, uint8_t* out, size_t cap) {
  uint8_t p[20] = {};
  p[0] = 1;                                // UART1
  rb::put_u32(p + 4, 0x000008D0);          // 8N1
  rb::put_u32(p + 8, baud);
  rb::put_u16(p + 12, 0x0007);             // entrada: UBX + NMEA + RTCM2 (default)
  rb::put_u16(p + 14, 0x0001);             // saída: só UBX
  return rb::encode_frame(kClsCfg, kIdCfgPrt, p, sizeof p, out, cap);
}

size_t cfg_prt_poll_uart1(uint8_t* out, size_t cap) {
  const uint8_t p[1] = {1};
  return rb::encode_frame(kClsCfg, kIdCfgPrt, p, 1, out, cap);
}

size_t cfg_msg(uint8_t cls, uint8_t id, uint8_t rate, uint8_t* out, size_t cap) {
  const uint8_t p[3] = {cls, id, rate};
  return rb::encode_frame(kClsCfg, kIdCfgMsg, p, 3, out, cap);
}

size_t cfg_msg_poll(uint8_t cls, uint8_t id, uint8_t* out, size_t cap) {
  const uint8_t p[2] = {cls, id};
  return rb::encode_frame(kClsCfg, kIdCfgMsg, p, 2, out, cap);
}

size_t cfg_rate(uint16_t meas_ms, uint8_t* out, size_t cap) {
  uint8_t p[6];
  rb::put_u16(p, meas_ms);
  rb::put_u16(p + 2, 1);  // navRate: 1 solução por medição
  rb::put_u16(p + 4, 1);  // timeRef: GPS
  return rb::encode_frame(kClsCfg, kIdCfgRate, p, sizeof p, out, cap);
}

size_t cfg_nav5_dyn(uint8_t dyn_model, uint8_t* out, size_t cap) {
  uint8_t p[36] = {};
  rb::put_u16(p, 0x0001);  // mask: só dyn
  p[2] = dyn_model;
  return rb::encode_frame(kClsCfg, kIdCfgNav5, p, sizeof p, out, cap);
}

size_t cfg_save(uint8_t* out, size_t cap) {
  uint8_t p[13] = {};
  rb::put_u32(p + 0, 0);           // clearMask: nada
  rb::put_u32(p + 4, 0x0000FFFF);  // saveMask: toda a config atual
  rb::put_u32(p + 8, 0);           // loadMask: nada
  p[12] = 0x03;                    // devBBR | devFlash
  return rb::encode_frame(kClsCfg, kIdCfgCfg, p, sizeof p, out, cap);
}

static size_t key_value_size(uint32_t key) {
  switch ((key >> 28) & 0x07) {
    case 1:  // L (1 bit, ocupa 1 byte)
    case 2:  // 1 byte
      return 1;
    case 3:
      return 2;
    case 4:
      return 4;
    default:
      return 0;  // 8 B não usado aqui
  }
}

size_t cfg_valset(uint8_t layers, const KeyVal* kv, size_t n, uint8_t* out, size_t cap) {
  uint8_t p[kMaxCmd];
  size_t len = 4;
  p[0] = 0;  // version
  p[1] = layers;
  p[2] = p[3] = 0;
  for (size_t i = 0; i < n; ++i) {
    const size_t vs = key_value_size(kv[i].key);
    if (vs == 0 || len + 4 + vs > sizeof p) return 0;
    rb::put_u32(p + len, kv[i].key);
    len += 4;
    for (size_t b = 0; b < vs; ++b) p[len++] = static_cast<uint8_t>(kv[i].value >> (8 * b));
  }
  return rb::encode_frame(kClsCfg, kIdCfgValset, p, len, out, cap);
}

bool parse_cfg_prt(const uint8_t* p, size_t len, PrtUart* out) {
  if (len < 20 || p[0] != 1) return false;
  out->mode = rb::get_u32(p + 4);
  out->baud = rb::get_u32(p + 8);
  out->in_proto = rb::get_u16(p + 12);
  out->out_proto = rb::get_u16(p + 14);
  return true;
}

bool parse_cfg_rate(const uint8_t* p, size_t len, uint16_t* meas_ms, uint16_t* nav_rate) {
  if (len < 6) return false;
  *meas_ms = rb::get_u16(p);
  *nav_rate = rb::get_u16(p + 2);
  return true;
}

bool parse_cfg_nav5_dyn(const uint8_t* p, size_t len, uint8_t* dyn_model) {
  if (len < 36) return false;
  *dyn_model = p[2];
  return true;
}

bool parse_cfg_msg_uart1(const uint8_t* p, size_t len, uint8_t* rate) {
  if (len < 8) return false;
  *rate = p[2 + 1];
  return true;
}

RateDecision decide_rate(uint8_t requested_code, uint8_t module_max_hz, uint8_t cap_hz) {
  RateDecision d{requested_code, requested_code, rb::rate_period_ms(requested_code), false};
  if (!rb::valid_rate(requested_code)) return d;
  uint8_t limit = module_max_hz;
  if (cap_hz != 0 && cap_hz < limit) limit = cap_hz;
  if (rb::rate_hz(requested_code) <= limit) return d;
  static const uint8_t by_hz_desc[] = {rb::rate::k25Hz, rb::rate::k20Hz, rb::rate::k10Hz,
                                       rb::rate::k5Hz, rb::rate::k1Hz};
  for (uint8_t code : by_hz_desc) {
    if (rb::rate_hz(code) <= limit) {
      d.effective_code = code;
      d.period_ms = rb::rate_period_ms(code);
      d.limited = true;
      return d;
    }
  }
  d.effective_code = rb::rate::k1Hz;
  d.period_ms = rb::rate_period_ms(rb::rate::k1Hz);
  d.limited = true;
  return d;
}

}  // namespace ubx
