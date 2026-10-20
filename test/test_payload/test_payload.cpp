// Mapeamento NAV-PVT → RaceBox (SPEC §5.2), clamps, fixType, bateria, iTOW.
#include <unity.h>

#include <cstring>
#include <string>
#include <vector>

#include "../common/mini_json.h"
#include "frame.h"
#include "itow.h"
#include "payload.h"

using Bytes = std::vector<uint8_t>;

void setUp() {}
void tearDown() {}

static Bytes golden_frame() {
  mj::Value fx;
  TEST_ASSERT_TRUE(mj::load_file(std::string(FIXTURE_DIR) + "/racebox_test_vectors.json", &fx));
  return mj::hex(fx["liveData"]["hex"].str());
}

// NAV-PVT (92 B) que um receptor teria mandado para gerar o frame golden.
static void navpvt_for_golden(const Bytes& golden, uint8_t pvt[rb::kNavPvtLen]) {
  const uint8_t* g = golden.data() + 6;
  std::memset(pvt, 0, rb::kNavPvtLen);
  std::memcpy(pvt, g, 48);
  rb::put_i32(pvt + 48, 30);              // velN
  rb::put_i32(pvt + 52, -18);             // velE
  rb::put_i32(pvt + 56, 7);               // velD
  rb::put_i32(pvt + 60, rb::get_i32(g + 48));  // gSpeed
  rb::put_i32(pvt + 64, rb::get_i32(g + 52));  // headMot
  rb::put_u32(pvt + 68, rb::get_u32(g + 56));  // sAcc
  rb::put_u32(pvt + 72, rb::get_u32(g + 60));  // headAcc
  rb::put_u16(pvt + 76, rb::get_u16(g + 64));  // pDOP
  pvt[78] = g[66];                              // flags3
  rb::put_i32(pvt + 84, 12345);                 // headVeh (não usado)
}

static rb::Motion motion_of(const uint8_t* g) {
  rb::Motion m;
  m.gx_mg = rb::get_i16(g + 68);
  m.gy_mg = rb::get_i16(g + 70);
  m.gz_mg = rb::get_i16(g + 72);
  m.rx_cdps = rb::get_i16(g + 74);
  m.ry_cdps = rb::get_i16(g + 76);
  m.rz_cdps = rb::get_i16(g + 78);
  return m;
}

void test_navpvt_path_reproduces_golden_frame() {
  const Bytes golden = golden_frame();
  uint8_t pvt[rb::kNavPvtLen];
  navpvt_for_golden(golden, pvt);
  rb::BuildOptions opt;
  opt.battery = golden[6 + 67];
  const rb::LiveData d = rb::live_from_navpvt(pvt, motion_of(golden.data() + 6), opt);
  uint8_t p[rb::kLivePayloadLen];
  rb::encode_live(d, p);
  uint8_t f[rb::kLivePayloadLen + rb::kOverhead];
  TEST_ASSERT_EQUAL_size_t(88, rb::encode_frame(rb::kClass, rb::msg::kLive, p, sizeof p, f, sizeof f));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(golden.data(), f, 88);
}

void test_first_48_bytes_are_direct_copy() {
  uint8_t pvt[rb::kNavPvtLen];
  for (size_t i = 0; i < sizeof pvt; ++i) pvt[i] = static_cast<uint8_t>(i * 7 + 3);
  pvt[20] = 3;  // fixType 3 → 3, para a cópia ser literal
  const rb::LiveData d = rb::live_from_navpvt(pvt, rb::Motion{}, rb::BuildOptions{});
  uint8_t p[rb::kLivePayloadLen];
  rb::encode_live(d, p);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(pvt, p, 48);
}

void test_remapped_offsets() {
  uint8_t pvt[rb::kNavPvtLen] = {};
  rb::put_i32(pvt + 60, 27777);       // gSpeed
  rb::put_i32(pvt + 64, 18000000);    // headMot
  rb::put_u32(pvt + 68, 321);         // sAcc
  rb::put_u32(pvt + 72, 4567890);     // headAcc
  rb::put_u16(pvt + 76, 155);         // pDOP
  pvt[78] = 0x03;                     // invalidLlh + idade 1
  rb::put_i32(pvt + 84, -1);          // headVeh não vaza
  const rb::LiveData d = rb::live_from_navpvt(pvt, rb::Motion{}, rb::BuildOptions{});
  TEST_ASSERT_EQUAL_INT32(27777, d.speed_mmps);
  TEST_ASSERT_EQUAL_INT32(18000000, d.heading);
  TEST_ASSERT_EQUAL_UINT32(321, d.speed_acc_mmps);
  TEST_ASSERT_EQUAL_UINT32(4567890, d.heading_acc);
  TEST_ASSERT_EQUAL_UINT16(155, d.pdop);
  TEST_ASSERT_EQUAL_HEX8(0x03, d.latlon_flags);
}

void test_flags3_high_bits_masked() {
  uint8_t pvt[rb::kNavPvtLen] = {};
  pvt[78] = 0xFF;
  pvt[79] = 0xFF;  // M9: flags3 é X2; byte alto nunca vaza para o offset 67
  const rb::LiveData d = rb::live_from_navpvt(pvt, rb::Motion{}, rb::BuildOptions{});
  TEST_ASSERT_EQUAL_HEX8(0x1F, d.latlon_flags);
  TEST_ASSERT_EQUAL_HEX8(0x64, d.battery);
}

void test_fix_type_mapping_all_values() {
  for (int t = 0; t < 256; ++t) {
    const uint8_t expect = t == 2 ? 2 : (t == 3 || t == 4) ? 3 : 0;
    TEST_ASSERT_EQUAL_UINT8(expect, rb::map_fix_type(static_cast<uint8_t>(t)));
    uint8_t pvt[rb::kNavPvtLen] = {};
    pvt[20] = static_cast<uint8_t>(t);
    TEST_ASSERT_EQUAL_UINT8(expect,
                            rb::live_from_navpvt(pvt, rb::Motion{}, rb::BuildOptions{}).fix_status);
  }
}

void test_no_fix_frame_keeps_flags_from_receiver() {
  uint8_t pvt[rb::kNavPvtLen] = {};
  pvt[20] = 0;     // no fix
  pvt[21] = 0x00;  // gnssFixOK = 0
  pvt[23] = 0;
  const rb::LiveData d = rb::live_from_navpvt(pvt, rb::Motion{}, rb::BuildOptions{});
  TEST_ASSERT_EQUAL_UINT8(0, d.fix_status);
  TEST_ASSERT_EQUAL_UINT8(0, d.fix_flags & 0x01);
}

void test_speed_3d() {
  uint8_t pvt[rb::kNavPvtLen] = {};
  rb::put_i32(pvt + 48, 3000);
  rb::put_i32(pvt + 52, -4000);
  rb::put_i32(pvt + 56, 12000);
  rb::put_i32(pvt + 60, 5000);
  rb::BuildOptions opt;
  TEST_ASSERT_EQUAL_INT32(5000, rb::live_from_navpvt(pvt, rb::Motion{}, opt).speed_mmps);
  opt.speed_3d = true;
  TEST_ASSERT_EQUAL_INT32(13000, rb::live_from_navpvt(pvt, rb::Motion{}, opt).speed_mmps);
}

void test_motion_clamp_never_wraps() {
  uint8_t pvt[rb::kNavPvtLen] = {};
  rb::Motion m;
  m.gx_mg = 40000.0f;     // 40 g
  m.gy_mg = -40000.0f;
  m.gz_mg = 1000.4f;
  m.rx_cdps = 40000.0f;   // 400 °/s
  m.ry_cdps = -50000.0f;  // −500 °/s
  m.rz_cdps = -0.6f;
  const rb::LiveData d = rb::live_from_navpvt(pvt, m, rb::BuildOptions{});
  TEST_ASSERT_EQUAL_INT16(32767, d.gx);
  TEST_ASSERT_EQUAL_INT16(-32767, d.gy);
  TEST_ASSERT_EQUAL_INT16(1000, d.gz);
  TEST_ASSERT_EQUAL_INT16(32767, d.rx);
  TEST_ASSERT_EQUAL_INT16(-32767, d.ry);
  TEST_ASSERT_EQUAL_INT16(-1, d.rz);
}

void test_battery_byte() {
  TEST_ASSERT_EQUAL_HEX8(0x64, rb::battery_byte(false, 0, false));
  TEST_ASSERT_EQUAL_HEX8(0x64, rb::battery_byte(false, 37, true));
  TEST_ASSERT_EQUAL_HEX8(0x00, rb::battery_byte(true, 0, false));
  TEST_ASSERT_EQUAL_HEX8(0x59, rb::battery_byte(true, 89, false));
  TEST_ASSERT_EQUAL_HEX8(0xB2, rb::battery_byte(true, 50, true));
  TEST_ASSERT_EQUAL_HEX8(0x64, rb::battery_byte(true, 150, false));
  TEST_ASSERT_TRUE(rb::BuildOptions{}.battery != 0);
}

void test_min_hacc_clears_fix_ok_only_when_worse() {
  rb::LiveData d;
  d.fix_status = 3;
  d.fix_flags = 0x01 | 0x20;
  d.h_acc_mm = 3000;
  rb::apply_min_hacc(d, 30);  // 3,0 m: igual ao limite → mantém
  TEST_ASSERT_TRUE(rb::good_fix(d));
  d.h_acc_mm = 3001;
  rb::apply_min_hacc(d, 30);
  TEST_ASSERT_FALSE(rb::good_fix(d));
  TEST_ASSERT_EQUAL_HEX8(0x20, d.fix_flags);  // só o b0 muda
  TEST_ASSERT_EQUAL_UINT8(3, d.fix_status);
  d.fix_flags = 0x01;
  d.h_acc_mm = 924;
  rb::apply_min_hacc(d, 20);
  TEST_ASSERT_TRUE(rb::good_fix(d));
}

// --- iTOW monotônico (SPEC §5.2, R14) -----------------------------------------

void test_itow_hot_start_is_raw() {
  rb::ItowMonotonic m;
  TEST_ASSERT_EQUAL_UINT32(1000, m.process(1000, true, 50));
  TEST_ASSERT_EQUAL_UINT32(1100, m.process(1100, true, 151));
  TEST_ASSERT_EQUAL_UINT32(1200, m.process(1200, true, 249));
  TEST_ASSERT_TRUE(m.mode() == rb::ItowMonotonic::Mode::kRaw);
  TEST_ASSERT_EQUAL_UINT32(0, m.backstep_fixes());
}

void test_itow_cold_start_counter_then_resolution_jump_back() {
  rb::ItowMonotonic m;
  // Receptor sem hora: iTOW interno alto; depois resolve num valor bem menor.
  uint32_t last = m.process(500000000, false, 10000);
  TEST_ASSERT_EQUAL_UINT32(500000000, last);
  for (uint32_t k = 1; k <= 50; ++k) {
    const uint32_t out = m.process(500000000 + 100 * k, false, 10000 + 100 * k);
    TEST_ASSERT_EQUAL_UINT32(last + 100, out);
    last = out;
  }
  // Hora resolve: iTOW real = 123456789 (salto para trás de ~4 dias).
  uint32_t out = m.process(123456789, true, 10000 + 100 * 51);
  TEST_ASSERT_EQUAL_UINT32(last + 100, out);
  TEST_ASSERT_EQUAL_UINT32(1, m.time_resolved_events());
  last = out;
  for (uint32_t k = 1; k <= 50; ++k) {
    out = m.process(123456789 + 100 * k, true, 0 /* relógio local não importa mais */);
    TEST_ASSERT_EQUAL_UINT32(last + 100, out);  // cadência real do receptor
    last = out;
  }
  TEST_ASSERT_TRUE(m.mode() == rb::ItowMonotonic::Mode::kOffset);
}

void test_itow_week_rollover_passes_through() {
  rb::ItowMonotonic m;
  m.process(rb::kMsPerWeek - 200, true, 0);
  TEST_ASSERT_EQUAL_UINT32(rb::kMsPerWeek - 100, m.process(rb::kMsPerWeek - 100, true, 0));
  TEST_ASSERT_EQUAL_UINT32(0, m.process(0, true, 0));
  TEST_ASSERT_EQUAL_UINT32(100, m.process(100, true, 0));
  TEST_ASSERT_EQUAL_UINT32(0, m.backstep_fixes());  // virada de semana não é passo para trás
}

void test_itow_backstep_in_raw_mode_is_absorbed() {
  rb::ItowMonotonic m;
  m.process(10000, true, 0);
  m.process(10100, true, 0);
  const uint32_t a = m.process(9000, true, 0);  // passo para trás espúrio
  TEST_ASSERT_EQUAL_UINT32(10101, a);
  TEST_ASSERT_EQUAL_UINT32(10201, m.process(9100, true, 0));
  TEST_ASSERT_EQUAL_UINT32(1, m.backstep_fixes());
  const uint32_t b = m.process(9100, true, 0);  // repetido
  TEST_ASSERT_EQUAL_UINT32(10202, b);
}

void test_itow_counter_never_repeats_with_coarse_clock() {
  rb::ItowMonotonic m;
  uint32_t last = m.process(42, false, 5);
  for (int k = 0; k < 10; ++k) {
    const uint32_t out = m.process(42, false, 5);  // relógio parado
    TEST_ASSERT_TRUE(out > last);
    last = out;
  }
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_navpvt_path_reproduces_golden_frame);
  RUN_TEST(test_first_48_bytes_are_direct_copy);
  RUN_TEST(test_remapped_offsets);
  RUN_TEST(test_flags3_high_bits_masked);
  RUN_TEST(test_fix_type_mapping_all_values);
  RUN_TEST(test_no_fix_frame_keeps_flags_from_receiver);
  RUN_TEST(test_speed_3d);
  RUN_TEST(test_motion_clamp_never_wraps);
  RUN_TEST(test_battery_byte);
  RUN_TEST(test_min_hacc_clears_fix_ok_only_when_worse);
  RUN_TEST(test_itow_hot_start_is_raw);
  RUN_TEST(test_itow_cold_start_counter_then_resolution_jump_back);
  RUN_TEST(test_itow_week_rollover_passes_through);
  RUN_TEST(test_itow_backstep_in_raw_mode_is_absorbed);
  RUN_TEST(test_itow_counter_never_repeats_with_coarse_clock);
  return UNITY_END();
}
