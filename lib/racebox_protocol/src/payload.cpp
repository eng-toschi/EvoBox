#include "payload.h"

#include <cmath>
#include <cstring>

#include "frame.h"

namespace rb {

void encode_live(const LiveData& d, uint8_t o[kLivePayloadLen]) {
  put_u32(o + 0, d.itow);
  put_u16(o + 4, d.year);
  o[6] = d.month;
  o[7] = d.day;
  o[8] = d.hour;
  o[9] = d.minute;
  o[10] = d.second;
  o[11] = d.validity;
  put_u32(o + 12, d.time_acc_ns);
  put_i32(o + 16, d.nano);
  o[20] = d.fix_status;
  o[21] = d.fix_flags;
  o[22] = d.datetime_flags;
  o[23] = d.num_sv;
  put_i32(o + 24, d.lon);
  put_i32(o + 28, d.lat);
  put_i32(o + 32, d.wgs_alt_mm);
  put_i32(o + 36, d.msl_alt_mm);
  put_u32(o + 40, d.h_acc_mm);
  put_u32(o + 44, d.v_acc_mm);
  put_i32(o + 48, d.speed_mmps);
  put_i32(o + 52, d.heading);
  put_u32(o + 56, d.speed_acc_mmps);
  put_u32(o + 60, d.heading_acc);
  put_u16(o + 64, d.pdop);
  o[66] = d.latlon_flags;
  o[67] = d.battery;
  put_i16(o + 68, d.gx);
  put_i16(o + 70, d.gy);
  put_i16(o + 72, d.gz);
  put_i16(o + 74, d.rx);
  put_i16(o + 76, d.ry);
  put_i16(o + 78, d.rz);
}

LiveData decode_live(const uint8_t* p) {
  LiveData d;
  d.itow = get_u32(p + 0);
  d.year = get_u16(p + 4);
  d.month = p[6];
  d.day = p[7];
  d.hour = p[8];
  d.minute = p[9];
  d.second = p[10];
  d.validity = p[11];
  d.time_acc_ns = get_u32(p + 12);
  d.nano = get_i32(p + 16);
  d.fix_status = p[20];
  d.fix_flags = p[21];
  d.datetime_flags = p[22];
  d.num_sv = p[23];
  d.lon = get_i32(p + 24);
  d.lat = get_i32(p + 28);
  d.wgs_alt_mm = get_i32(p + 32);
  d.msl_alt_mm = get_i32(p + 36);
  d.h_acc_mm = get_u32(p + 40);
  d.v_acc_mm = get_u32(p + 44);
  d.speed_mmps = get_i32(p + 48);
  d.heading = get_i32(p + 52);
  d.speed_acc_mmps = get_u32(p + 56);
  d.heading_acc = get_u32(p + 60);
  d.pdop = get_u16(p + 64);
  d.latlon_flags = p[66];
  d.battery = p[67];
  d.gx = get_i16(p + 68);
  d.gy = get_i16(p + 70);
  d.gz = get_i16(p + 72);
  d.rx = get_i16(p + 74);
  d.ry = get_i16(p + 76);
  d.rz = get_i16(p + 78);
  return d;
}

uint8_t map_fix_type(uint8_t t) {
  switch (t) {
    case 2:
      return 2;
    case 3:
    case 4:
      return 3;
    default:
      return 0;
  }
}

int16_t clamp_i16(int32_t v) {
  if (v > 32767) return 32767;
  if (v < -32767) return -32767;  // simétrico: −32768 não tem par positivo
  return static_cast<int16_t>(v);
}

int16_t wire_i16(float v) {
  if (std::isnan(v)) return 0;
  if (v >= 32767.0f) return 32767;
  if (v <= -32767.0f) return -32767;
  return clamp_i16(static_cast<int32_t>(std::lround(v)));
}

uint8_t battery_byte(bool measured, uint8_t percent, bool charging) {
  if (!measured) return 0x64;
  const uint8_t pct = percent > 100 ? 100 : percent;
  return static_cast<uint8_t>(pct | (charging ? 0x80 : 0x00));
}

LiveData live_from_navpvt(const uint8_t* pvt, const Motion& m, const BuildOptions& opt) {
  // NAV-PVT 0..47 é idêntico ao payload RaceBox 0..47 (SPEC §5.2).
  uint8_t raw[kLivePayloadLen];
  std::memset(raw, 0, sizeof raw);
  std::memcpy(raw, pvt, 48);
  LiveData d = decode_live(raw);
  d.fix_status = map_fix_type(pvt[20]);

  if (opt.speed_3d) {
    const double vn = get_i32(pvt + 48);
    const double ve = get_i32(pvt + 52);
    const double vd = get_i32(pvt + 56);
    const double s = std::sqrt(vn * vn + ve * ve + vd * vd);
    d.speed_mmps = s >= 2147483647.0 ? 2147483647 : static_cast<int32_t>(std::lround(s));
  } else {
    d.speed_mmps = get_i32(pvt + 60);  // gSpeed
  }
  d.heading = get_i32(pvt + 64);         // headMot
  d.speed_acc_mmps = get_u32(pvt + 68);  // sAcc
  d.heading_acc = get_u32(pvt + 72);     // headAcc
  d.pdop = get_u16(pvt + 76);            // pDOP
  d.latlon_flags = pvt[78] & 0x1F;       // flags3 (byte baixo): b0 invalidLlh, b1–4 idade
  d.battery = opt.battery;
  d.gx = wire_i16(m.gx_mg);
  d.gy = wire_i16(m.gy_mg);
  d.gz = wire_i16(m.gz_mg);
  d.rx = wire_i16(m.rx_cdps);
  d.ry = wire_i16(m.ry_cdps);
  d.rz = wire_i16(m.rz_cdps);
  return d;
}

}  // namespace rb
