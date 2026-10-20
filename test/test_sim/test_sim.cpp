// Modo simulação: pista circular coerente (SPEC §9.3).
#include <unity.h>

#include <cmath>

#include "app/sim.h"
#include "frame.h"
#include "messages.h"
#include "payload.h"

void setUp() {}
void tearDown() {}

constexpr double kPi = 3.14159265358979323846;

static void pos_m(const sim::TrackConfig& c, const uint8_t* p, double* east, double* north) {
  const double lat = rb::get_i32(p + 28) / 1e7, lon = rb::get_i32(p + 24) / 1e7;
  *north = (lat - c.center_lat_deg) * kPi / 180.0 * 6371000.0;
  *east = (lon - c.center_lon_deg) * kPi / 180.0 * 6371000.0 * std::cos(c.center_lat_deg * kPi / 180.0);
}

void test_utc_from_gps() {
  // Semana 2441 começa em 2026-10-18 00:00:00 GPS = 2026-10-17 23:59:42 UTC.
  sim::Utc u = sim::utc_from_gps(2441, 0);
  TEST_ASSERT_EQUAL_UINT16(2026, u.year);
  TEST_ASSERT_EQUAL_UINT8(10, u.month);
  TEST_ASSERT_EQUAL_UINT8(17, u.day);
  TEST_ASSERT_EQUAL_UINT8(23, u.hour);
  TEST_ASSERT_EQUAL_UINT8(59, u.minute);
  TEST_ASSERT_EQUAL_UINT8(42, u.second);
  u = sim::utc_from_gps(2441, 2 * 86400000u + 18000u + 123);  // terça 00:00:00.123 UTC
  TEST_ASSERT_EQUAL_UINT8(20, u.day);
  TEST_ASSERT_EQUAL_UINT8(0, u.hour);
  TEST_ASSERT_EQUAL_UINT8(0, u.second);
  TEST_ASSERT_EQUAL_INT32(123000000, u.nano);
  // vetor do vendor: iTOW 118286240 → 2022-01-10 08:51:08 (semana 2192)
  u = sim::utc_from_gps(2192, 118286240);
  TEST_ASSERT_EQUAL_UINT16(2022, u.year);
  TEST_ASSERT_EQUAL_UINT8(1, u.month);
  TEST_ASSERT_EQUAL_UINT8(10, u.day);
  TEST_ASSERT_EQUAL_UINT8(8, u.hour);
  TEST_ASSERT_EQUAL_UINT8(51, u.minute);
  TEST_ASSERT_EQUAL_UINT8(8, u.second);
}

void test_circle_geometry_speed_heading() {
  sim::TrackSim s;
  const sim::TrackConfig& c = s.config();
  s.reset(2441, 1000000);
  uint8_t p[rb::kNavPvtLen];
  rb::Motion m;
  for (uint32_t k = 0; k < 2000; ++k) {
    s.make(1000000 + 40 * k, p, &m);
    double e, n;
    pos_m(c, p, &e, &n);
    TEST_ASSERT_FLOAT_WITHIN(static_cast<float>(0.05), static_cast<float>(c.radius_m), static_cast<float>(std::sqrt(e * e + n * n)));
    const double vn = rb::get_i32(p + 48) / 1000.0, ve = rb::get_i32(p + 52) / 1000.0;
    TEST_ASSERT_FLOAT_WITHIN(static_cast<float>(0.002), static_cast<float>(c.speed_mps), static_cast<float>(std::sqrt(vn * vn + ve * ve)));
    TEST_ASSERT_EQUAL_INT32(static_cast<int32_t>(c.speed_mps * 1000), rb::get_i32(p + 60));
    // velocidade tangente ao círculo
    TEST_ASSERT_FLOAT_WITHIN(static_cast<float>(0.01), static_cast<float>(0.0), static_cast<float>((e * ve + n * vn) / c.radius_m));
    // horário: velocidade = (n, −e)·w
    TEST_ASSERT_TRUE(n * ve - e * vn > 0);
  }
  // no ponto leste, indo para o sul: heading 180°
  s.make(1000000, p, &m);
  TEST_ASSERT_INT32_WITHIN(10, 18000000, rb::get_i32(p + 64));
}

void test_lap_time_and_itow() {
  sim::TrackSim s;
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 31.416f, s.lap_seconds());
  s.reset(2441, 5000);
  uint8_t a[rb::kNavPvtLen], b[rb::kNavPvtLen];
  rb::Motion m;
  s.make(5000, a, &m);
  s.make(5000 + static_cast<uint32_t>(std::lround(s.lap_seconds() * 1000)), b, &m);
  TEST_ASSERT_INT32_WITHIN(25, rb::get_i32(a + 24), rb::get_i32(b + 24));
  TEST_ASSERT_INT32_WITHIN(25, rb::get_i32(a + 28), rb::get_i32(b + 28));
  TEST_ASSERT_EQUAL_UINT32(5000 + 31416, rb::get_u32(b));
}

void test_sim_frames_are_good_fix_and_marked() {
  sim::TrackSim s;
  s.reset(2441, 0);
  uint8_t p[rb::kNavPvtLen];
  rb::Motion m;
  s.make(123400, p, &m);
  rb::LiveData d = rb::live_from_navpvt(p, m, rb::BuildOptions{});
  rb::apply_min_hacc(d, rb::GnssConfig{}.min_hacc_dm);  // default 3,0 m
  TEST_ASSERT_TRUE(rb::good_fix(d));
  TEST_ASSERT_EQUAL_UINT8(0, d.num_sv);  // marca de simulação
  TEST_ASSERT_EQUAL_UINT8(0x07, d.validity & 0x07);
  // IMU coerente: curva à direita → gY +; v²/r = 400/100 = 4 m/s² = 408 mg
  TEST_ASSERT_INT16_WITHIN(1, 408, d.gy);
  TEST_ASSERT_EQUAL_INT16(1000, d.gz);
  TEST_ASSERT_EQUAL_INT16(0, d.gx);
  TEST_ASSERT_INT16_WITHIN(1, -1146, d.rz);  // −0,2 rad/s horário
}

void test_counter_clockwise_mirrors() {
  sim::TrackConfig c;
  c.clockwise = false;
  sim::TrackSim s(c);
  s.reset(2441, 0);
  uint8_t p[rb::kNavPvtLen];
  rb::Motion m;
  s.make(1000, p, &m);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, -408.0f, m.gy_mg);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 1146.0f, m.rz_cdps);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_utc_from_gps);
  RUN_TEST(test_circle_geometry_speed_heading);
  RUN_TEST(test_lap_time_and_itow);
  RUN_TEST(test_sim_frames_are_good_fix_and_marked);
  RUN_TEST(test_counter_clockwise_mirrors);
  return UNITY_END();
}
