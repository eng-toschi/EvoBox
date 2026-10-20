// Comandos UBX para o receptor, parse de MON-VER e política de taxa (SPEC §2.2, §5.3–§5.4).
#include <unity.h>

#include <cstring>
#include <vector>

#include "frame.h"
#include "gnss/ublox_config.h"
#include "messages.h"

using Bytes = std::vector<uint8_t>;

void setUp() {}
void tearDown() {}

static void assert_frame(const Bytes& expect, const uint8_t* got, size_t n) {
  TEST_ASSERT_EQUAL_size_t(expect.size(), n);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expect.data(), got, n);
}

// Comandos u-blox amplamente publicados (Interface Description / app notes).
void test_known_commands() {
  uint8_t b[ubx::kMaxCmd + 8];
  assert_frame({0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34}, b,
               ubx::poll(ubx::kClsMon, ubx::kIdMonVer, b, sizeof b));
  assert_frame({0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12},
               b, ubx::cfg_rate(100, b, sizeof b));
}

void test_cfg_prt_uart1_ubx_only() {
  uint8_t b[64];
  const size_t n = ubx::cfg_prt_uart1(115200, b, sizeof b);
  TEST_ASSERT_EQUAL_size_t(28, n);
  TEST_ASSERT_TRUE(rb::validate_frame(b, n));
  ubx::PrtUart prt;
  TEST_ASSERT_TRUE(ubx::parse_cfg_prt(b + 6, 20, &prt));
  TEST_ASSERT_EQUAL_UINT32(115200, prt.baud);
  TEST_ASSERT_EQUAL_HEX32(0x08D0, prt.mode);
  TEST_ASSERT_EQUAL_HEX16(0x0001, prt.out_proto);  // nenhuma NMEA na saída
  TEST_ASSERT_EQUAL_HEX16(0x0007, prt.in_proto);
}

void test_cfg_msg_nav_pvt_and_nav5_and_save() {
  uint8_t b[64];
  size_t n = ubx::cfg_msg(ubx::kClsNav, ubx::kIdNavPvt, 1, b, sizeof b);
  assert_frame({0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51}, b, n);

  n = ubx::cfg_nav5_dyn(4, b, sizeof b);
  TEST_ASSERT_EQUAL_size_t(44, n);
  TEST_ASSERT_TRUE(rb::validate_frame(b, n));
  uint8_t dyn = 0;
  TEST_ASSERT_TRUE(ubx::parse_cfg_nav5_dyn(b + 6, 36, &dyn));
  TEST_ASSERT_EQUAL_UINT8(4, dyn);
  TEST_ASSERT_EQUAL_HEX8(0x01, b[6]);  // mask só dyn
  TEST_ASSERT_EQUAL_HEX8(0x00, b[7]);

  n = ubx::cfg_save(b, sizeof b);
  TEST_ASSERT_EQUAL_size_t(21, n);
  TEST_ASSERT_TRUE(rb::validate_frame(b, n));
  // Nunca clear/load (nada de reset de fábrica).
  TEST_ASSERT_EQUAL_UINT32(0, rb::get_u32(b + 6));
  TEST_ASSERT_EQUAL_UINT32(0, rb::get_u32(b + 14));
  TEST_ASSERT_EQUAL_HEX8(0x03, b[18]);
}

void test_valset_encoding() {
  const ubx::KeyVal kv[] = {{ubx::key::kRateMeas, 40},
                            {ubx::key::kMsgNavPvtUart1, 1},
                            {ubx::key::kUart1Baud, 230400},
                            {ubx::key::kUart1OutNmea, 0}};
  uint8_t b[ubx::kMaxCmd + 8];
  const size_t n = ubx::cfg_valset(ubx::kLayerRam | ubx::kLayerBbr, kv, 4, b, sizeof b);
  // 4 (hdr) + (4+2) + (4+1) + (4+4) + (4+1) = 28
  TEST_ASSERT_EQUAL_size_t(28 + 8, n);
  TEST_ASSERT_TRUE(rb::validate_frame(b, n));
  const uint8_t* p = b + 6;
  TEST_ASSERT_EQUAL_HEX8(0x00, p[0]);
  TEST_ASSERT_EQUAL_HEX8(0x03, p[1]);
  TEST_ASSERT_EQUAL_HEX32(ubx::key::kRateMeas, rb::get_u32(p + 4));
  TEST_ASSERT_EQUAL_UINT16(40, rb::get_u16(p + 8));
  TEST_ASSERT_EQUAL_HEX32(ubx::key::kMsgNavPvtUart1, rb::get_u32(p + 10));
  TEST_ASSERT_EQUAL_UINT8(1, p[14]);
  TEST_ASSERT_EQUAL_HEX32(ubx::key::kUart1Baud, rb::get_u32(p + 15));
  TEST_ASSERT_EQUAL_UINT32(230400, rb::get_u32(p + 19));
  TEST_ASSERT_EQUAL_HEX32(ubx::key::kUart1OutNmea, rb::get_u32(p + 23));
  TEST_ASSERT_EQUAL_UINT8(0, p[27]);
}

static Bytes mon_ver(const char* sw, const char* hw, std::vector<const char*> ext) {
  Bytes p(40 + 30 * ext.size(), 0);
  std::memcpy(p.data(), sw, std::strlen(sw));
  std::memcpy(p.data() + 30, hw, std::strlen(hw));
  for (size_t i = 0; i < ext.size(); ++i)
    std::memcpy(p.data() + 40 + 30 * i, ext[i], std::strlen(ext[i]));
  return p;
}

void test_mon_ver_m8n() {
  const Bytes p = mon_ver("ROM CORE 3.01 (107888)", "00080000",
                          {"ROM BASE 0x118B2060", "FWVER=SPG 3.01", "PROTVER=18.00",
                           "GPS;GLO;GAL;BDS", "SBAS;IMES;QZSS"});
  ubx::ModuleInfo m;
  TEST_ASSERT_TRUE(ubx::parse_mon_ver(p.data(), p.size(), &m));
  TEST_ASSERT_TRUE(m.module == ubx::Module::kM8);
  TEST_ASSERT_EQUAL_UINT8(10, m.max_rate_hz);
  TEST_ASSERT_FALSE(m.uses_valset);
  TEST_ASSERT_EQUAL_UINT16(1800, m.protver_x100);
  TEST_ASSERT_EQUAL_STRING("SPG 3.01", m.fwver);
  TEST_ASSERT_EQUAL_STRING("00080000", m.hw);
}

void test_mon_ver_m9n() {
  const Bytes p = mon_ver("ROM SPG 4.04 (75bb9b)", "000A0000",
                          {"ROM BASE 0x118B2060", "FWVER=SPG 4.04", "PROTVER=32.01",
                           "MOD=NEO-M9N", "GPS;GLO;GAL;BDS", "SBAS;QZSS"});
  ubx::ModuleInfo m;
  TEST_ASSERT_TRUE(ubx::parse_mon_ver(p.data(), p.size(), &m));
  TEST_ASSERT_TRUE(m.module == ubx::Module::kM9);
  TEST_ASSERT_EQUAL_UINT8(25, m.max_rate_hz);
  TEST_ASSERT_TRUE(m.uses_valset);
  TEST_ASSERT_EQUAL_UINT16(3201, m.protver_x100);
  TEST_ASSERT_EQUAL_STRING("NEO-M9N", m.mod);
}

// Strings reais lidas da placa EvoBox em 2026-10-20 (vendida como NEO-M8N).
void test_mon_ver_m10_as_seen_on_bench() {
  const Bytes p = mon_ver("ROM SPG 5.10 (7b202e)", "000A0000",
                          {"FWVER=SPG 5.10", "PROTVER=34.10", "GPS;GLO;GAL;BDS", "SBAS;QZSS"});
  ubx::ModuleInfo m;
  TEST_ASSERT_TRUE(ubx::parse_mon_ver(p.data(), p.size(), &m));
  TEST_ASSERT_TRUE(m.module == ubx::Module::kM10);
  TEST_ASSERT_EQUAL_UINT8(10, m.max_rate_hz);  // SPEC §2.2 / D16 ‹VALIDAR›
  TEST_ASSERT_TRUE(m.uses_valset);
  TEST_ASSERT_EQUAL_UINT16(3410, m.protver_x100);
}

void test_mon_ver_short_rejected() {
  uint8_t p[39] = {};
  ubx::ModuleInfo m;
  TEST_ASSERT_FALSE(ubx::parse_mon_ver(p, sizeof p, &m));
}

void test_mon_gnss_mask() {
  const uint8_t p[8] = {0, 0x0F, 0x03, 0x03, 0x02, 0, 0, 0};
  ubx::GnssMask g;
  TEST_ASSERT_TRUE(ubx::parse_mon_gnss(p, 8, &g));
  TEST_ASSERT_EQUAL_HEX8(0x03, g.enabled);
  char buf[32];
  TEST_ASSERT_EQUAL_STRING("GPS+GLO", ubx::gnss_mask_str(g.enabled, buf, sizeof buf));
  TEST_ASSERT_EQUAL_STRING("GPS+GLO+BDS+GAL", ubx::gnss_mask_str(0x0F, buf, sizeof buf));
}

void test_rate_policy_m8n_limits_25_and_20_to_10() {
  ubx::RateDecision d = ubx::decide_rate(rb::rate::k25Hz, 10, 0);
  TEST_ASSERT_TRUE(d.limited);
  TEST_ASSERT_EQUAL_UINT8(rb::rate::k10Hz, d.effective_code);
  TEST_ASSERT_EQUAL_UINT16(100, d.period_ms);
  d = ubx::decide_rate(rb::rate::k20Hz, 10, 0);
  TEST_ASSERT_TRUE(d.limited);
  TEST_ASSERT_EQUAL_UINT16(100, d.period_ms);
  for (uint8_t code : {rb::rate::k10Hz, rb::rate::k5Hz, rb::rate::k1Hz}) {
    d = ubx::decide_rate(code, 10, 0);
    TEST_ASSERT_FALSE(d.limited);
    TEST_ASSERT_EQUAL_UINT8(code, d.effective_code);
  }
}

void test_rate_policy_m9n_and_cap() {
  ubx::RateDecision d = ubx::decide_rate(rb::rate::k25Hz, 25, 0);
  TEST_ASSERT_FALSE(d.limited);
  TEST_ASSERT_EQUAL_UINT16(40, d.period_ms);
  d = ubx::decide_rate(rb::rate::k25Hz, 25, 10);  // teto max_rate = 10 (D12)
  TEST_ASSERT_TRUE(d.limited);
  TEST_ASSERT_EQUAL_UINT16(100, d.period_ms);
  d = ubx::decide_rate(rb::rate::k25Hz, 25, 20);
  TEST_ASSERT_TRUE(d.limited);
  TEST_ASSERT_EQUAL_UINT16(50, d.period_ms);
  d = ubx::decide_rate(rb::rate::k20Hz, 25, 25);
  TEST_ASSERT_FALSE(d.limited);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_known_commands);
  RUN_TEST(test_cfg_prt_uart1_ubx_only);
  RUN_TEST(test_cfg_msg_nav_pvt_and_nav5_and_save);
  RUN_TEST(test_valset_encoding);
  RUN_TEST(test_mon_ver_m8n);
  RUN_TEST(test_mon_ver_m9n);
  RUN_TEST(test_mon_ver_m10_as_seen_on_bench);
  RUN_TEST(test_mon_ver_short_rejected);
  RUN_TEST(test_mon_gnss_mask);
  RUN_TEST(test_rate_policy_m8n_limits_25_and_20_to_10);
  RUN_TEST(test_rate_policy_m9n_and_cap);
  return UNITY_END();
}
