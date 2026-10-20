#include "app/sim.h"

#include <cmath>
#include <cstring>

#include "frame.h"
#include "itow.h"

namespace sim {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthR = 6371000.0;
constexpr double kG = 9.80665;
constexpr int64_t kGpsEpochUnix = 315964800;  // 1980-01-06
constexpr int kLeapSeconds = 18;

// Dias desde 1970-01-01 → data civil (Howard Hinnant).
void civil_from_days(int64_t z, int* y, unsigned* m, unsigned* d) {
  z += 719468;
  const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  const unsigned doe = static_cast<unsigned>(z - era * 146097);
  const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const int64_t yy = static_cast<int64_t>(yoe) + era * 400;
  const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const unsigned mp = (5 * doy + 2) / 153;
  *d = doy - (153 * mp + 2) / 5 + 1;
  *m = mp < 10 ? mp + 3 : mp - 9;
  *y = static_cast<int>(yy + (*m <= 2));
}

}  // namespace

Utc utc_from_gps(uint16_t week, uint32_t itow_ms) {
  const int64_t ms = (kGpsEpochUnix + static_cast<int64_t>(week) * 604800 - kLeapSeconds) * 1000 +
                     itow_ms;
  const int64_t secs = ms / 1000;
  const int64_t days = secs / 86400;
  const int64_t sod = secs % 86400;
  int y;
  unsigned m, d;
  civil_from_days(days, &y, &m, &d);
  Utc u;
  u.year = static_cast<uint16_t>(y);
  u.month = static_cast<uint8_t>(m);
  u.day = static_cast<uint8_t>(d);
  u.hour = static_cast<uint8_t>(sod / 3600);
  u.minute = static_cast<uint8_t>((sod / 60) % 60);
  u.second = static_cast<uint8_t>(sod % 60);
  u.nano = static_cast<int32_t>((ms % 1000) * 1000000);
  return u;
}

float TrackSim::lap_seconds() const {
  return static_cast<float>(2.0 * kPi * cfg_.radius_m / cfg_.speed_mps);
}

void TrackSim::make(uint32_t itow_ms, uint8_t* p, rb::Motion* motion) const {
  std::memset(p, 0, rb::kNavPvtLen);
  const uint32_t dt_ms = (itow_ms + rb::kMsPerWeek - itow0_) % rb::kMsPerWeek;
  const double t = dt_ms / 1000.0;
  const double r = cfg_.radius_m, v = cfg_.speed_mps;
  const double w = v / r;                          // rad/s
  const double dir = cfg_.clockwise ? -1.0 : 1.0;  // sentido do ângulo (anti-horário = +)
  const double th = dir * w * t;
  const double east = r * std::cos(th), north = r * std::sin(th);
  const double ve = -r * std::sin(th) * dir * w;  // d(east)/dt  = −r·sinθ·θ'
  const double vn = r * std::cos(th) * dir * w;   // d(north)/dt =  r·cosθ·θ'
  const double lat0 = cfg_.center_lat_deg * kPi / 180.0;
  const double lat = cfg_.center_lat_deg + (north / kEarthR) * 180.0 / kPi;
  const double lon = cfg_.center_lon_deg + (east / (kEarthR * std::cos(lat0))) * 180.0 / kPi;
  double head = std::atan2(ve, vn) * 180.0 / kPi;  // de norte, horário
  if (head < 0) head += 360.0;

  const Utc u = utc_from_gps(week_, itow_ms);
  rb::put_u32(p + 0, itow_ms);
  rb::put_u16(p + 4, u.year);
  p[6] = u.month;
  p[7] = u.day;
  p[8] = u.hour;
  p[9] = u.minute;
  p[10] = u.second;
  p[11] = 0x07;                      // validDate | validTime | fullyResolved
  rb::put_u32(p + 12, 20);           // tAcc
  rb::put_i32(p + 16, u.nano);
  p[20] = 3;                         // 3D
  p[21] = 0x01;                      // gnssFixOK
  p[22] = 0xE0;                      // data/hora confirmadas
  p[23] = 0;                         // numSV = 0: marca de simulação
  rb::put_i32(p + 24, static_cast<int32_t>(std::lround(lon * 1e7)));
  rb::put_i32(p + 28, static_cast<int32_t>(std::lround(lat * 1e7)));
  rb::put_i32(p + 32, 760000);       // height (mm)
  rb::put_i32(p + 36, 755000);       // hMSL
  rb::put_u32(p + 40, 2500);         // hAcc 2,5 m (< 3,0 m default do 0x27)
  rb::put_u32(p + 44, 4000);         // vAcc
  rb::put_i32(p + 48, static_cast<int32_t>(std::lround(vn * 1000)));
  rb::put_i32(p + 52, static_cast<int32_t>(std::lround(ve * 1000)));
  rb::put_i32(p + 56, 0);            // velD
  rb::put_i32(p + 60, static_cast<int32_t>(std::lround(v * 1000)));
  rb::put_i32(p + 64, static_cast<int32_t>(std::lround(head * 1e5)));
  rb::put_u32(p + 68, 300);          // sAcc
  rb::put_u32(p + 72, 50000);        // headAcc 0,5°
  rb::put_u16(p + 76, 150);          // pDOP 1,5
  p[78] = 0;                         // flags3

  // IMU no fio: centrípeta para o centro da curva; yaw pela regra da mão
  // direita sobre (frente, direita, cima): horário = negativo (axis.h, D5).
  const double a_c = v * v / r / kG;  // g
  motion->gx_mg = 0.0f;
  motion->gy_mg = static_cast<float>((cfg_.clockwise ? 1.0 : -1.0) * a_c * 1000.0);
  motion->gz_mg = 1000.0f;
  motion->rx_cdps = 0.0f;
  motion->ry_cdps = 0.0f;
  motion->rz_cdps = static_cast<float>(dir * w * 180.0 / kPi * 100.0);
}

}  // namespace sim
