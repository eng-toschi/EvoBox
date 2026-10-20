// Suíte de conformidade (SPEC §10.1): golden vectors do Racevo, byte a byte.
// test/fixtures/racebox_test_vectors.json é cópia verbatim — nunca editar.
#include <unity.h>

#include <cmath>
#include <string>
#include <vector>

#include "../common/mini_json.h"
#include "frame.h"
#include "messages.h"
#include "payload.h"

using Bytes = std::vector<uint8_t>;

static mj::Value g_fx;

static long long scaled(const mj::Value& v, double k) {
  return std::llround(v.num() * k);
}

static Bytes frame_of(uint8_t id, const uint8_t* payload, size_t len) {
  Bytes out(len + rb::kOverhead);
  const size_t n = rb::encode_frame(rb::kClass, id, payload, len, out.data(), out.size());
  out.resize(n);
  return out;
}

static void assert_bytes(const Bytes& expected, const Bytes& actual) {
  TEST_ASSERT_EQUAL_size_t(expected.size(), actual.size());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected.data(), actual.data(), expected.size());
}

// Payload de um frame já validado.
static const uint8_t* payload_of(const Bytes& f) { return f.data() + 6; }
static size_t payload_len(const Bytes& f) { return rb::get_u16(f.data() + 4); }

void setUp() {}
void tearDown() {}

// --- Golden live frame -------------------------------------------------------

static rb::LiveData live_from_expected(const mj::Value& e) {
  rb::LiveData d;
  d.itow = static_cast<uint32_t>(e["iTOW"].i64());
  d.year = static_cast<uint16_t>(e["year"].i64());
  d.month = static_cast<uint8_t>(e["month"].i64());
  d.day = static_cast<uint8_t>(e["day"].i64());
  d.hour = static_cast<uint8_t>(e["hour"].i64());
  d.minute = static_cast<uint8_t>(e["minute"].i64());
  d.second = static_cast<uint8_t>(e["second"].i64());
  d.validity = static_cast<uint8_t>(e["validityFlags"].i64());
  d.time_acc_ns = static_cast<uint32_t>(e["timeAccuracyNs"].i64());
  d.nano = static_cast<int32_t>(e["nanoseconds"].i64());
  d.fix_status = static_cast<uint8_t>(e["fixStatus"].i64());
  d.fix_flags = static_cast<uint8_t>(e["fixStatusFlags"].i64());
  d.datetime_flags = static_cast<uint8_t>(e["dateTimeFlags"].i64());
  d.num_sv = static_cast<uint8_t>(e["numSV"].i64());
  d.lon = static_cast<int32_t>(scaled(e["longitudeDeg"], 1e7));
  d.lat = static_cast<int32_t>(scaled(e["latitudeDeg"], 1e7));
  d.wgs_alt_mm = static_cast<int32_t>(scaled(e["wgsAltitudeM"], 1e3));
  d.msl_alt_mm = static_cast<int32_t>(scaled(e["mslAltitudeM"], 1e3));
  d.h_acc_mm = static_cast<uint32_t>(scaled(e["horizontalAccuracyM"], 1e3));
  d.v_acc_mm = static_cast<uint32_t>(scaled(e["verticalAccuracyM"], 1e3));
  d.speed_mmps = static_cast<int32_t>(scaled(e["speedMps"], 1e3));
  d.heading = static_cast<int32_t>(scaled(e["headingDeg"], 1e5));
  d.speed_acc_mmps = static_cast<uint32_t>(scaled(e["speedAccuracyMps"], 1e3));
  d.heading_acc = static_cast<uint32_t>(scaled(e["headingAccuracyDeg"], 1e5));
  d.pdop = static_cast<uint16_t>(scaled(e["pdop"], 1e2));
  d.latlon_flags = static_cast<uint8_t>(e["latLonFlags"].i64());
  d.battery = rb::battery_byte(true, static_cast<uint8_t>(e["batteryPercent"].i64()),
                               e["charging"].b);
  d.gx = rb::wire_i16(static_cast<float>(e["gxG"].num() * 1e3));
  d.gy = rb::wire_i16(static_cast<float>(e["gyG"].num() * 1e3));
  d.gz = rb::wire_i16(static_cast<float>(e["gzG"].num() * 1e3));
  d.rx = rb::wire_i16(static_cast<float>(e["rotXDps"].num() * 1e2));
  d.ry = rb::wire_i16(static_cast<float>(e["rotYDps"].num() * 1e2));
  d.rz = rb::wire_i16(static_cast<float>(e["rotZDps"].num() * 1e2));
  return d;
}

void test_golden_live_frame_built_from_decoded_fields_is_byte_identical() {
  const mj::Value& live = g_fx["liveData"];
  const Bytes golden = mj::hex(live["hex"].str());
  TEST_ASSERT_EQUAL_size_t(88, golden.size());

  const rb::LiveData d = live_from_expected(live["expected"]);
  TEST_ASSERT_EQUAL_UINT8(live["expected"]["batteryRaw"].i64(), d.battery);
  uint8_t payload[rb::kLivePayloadLen];
  rb::encode_live(d, payload);
  assert_bytes(golden, frame_of(rb::msg::kLive, payload, sizeof payload));
}

void test_golden_live_frame_decodes_to_expected_fields() {
  const mj::Value& live = g_fx["liveData"];
  const mj::Value& e = live["expected"];
  const Bytes golden = mj::hex(live["hex"].str());
  TEST_ASSERT_TRUE(rb::validate_frame(golden.data(), golden.size()));
  TEST_ASSERT_EQUAL_HEX8(rb::kClass, golden[2]);
  TEST_ASSERT_EQUAL_HEX8(rb::msg::kLive, golden[3]);
  TEST_ASSERT_EQUAL_size_t(rb::kLivePayloadLen, payload_len(golden));

  const rb::LiveData d = rb::decode_live(payload_of(golden));
  TEST_ASSERT_EQUAL_UINT32(e["iTOW"].i64(), d.itow);
  TEST_ASSERT_EQUAL_UINT16(e["year"].i64(), d.year);
  TEST_ASSERT_EQUAL_UINT8(e["month"].i64(), d.month);
  TEST_ASSERT_EQUAL_UINT8(e["day"].i64(), d.day);
  TEST_ASSERT_EQUAL_UINT8(e["hour"].i64(), d.hour);
  TEST_ASSERT_EQUAL_UINT8(e["minute"].i64(), d.minute);
  TEST_ASSERT_EQUAL_UINT8(e["second"].i64(), d.second);
  TEST_ASSERT_EQUAL_UINT8(e["validityFlags"].i64(), d.validity);
  TEST_ASSERT_EQUAL_UINT32(e["timeAccuracyNs"].i64(), d.time_acc_ns);
  TEST_ASSERT_EQUAL_INT32(e["nanoseconds"].i64(), d.nano);
  TEST_ASSERT_EQUAL_UINT8(e["fixStatus"].i64(), d.fix_status);
  TEST_ASSERT_EQUAL_UINT8(e["fixStatusFlags"].i64(), d.fix_flags);
  TEST_ASSERT_EQUAL_UINT8(e["dateTimeFlags"].i64(), d.datetime_flags);
  TEST_ASSERT_EQUAL_UINT8(e["numSV"].i64(), d.num_sv);
  TEST_ASSERT_EQUAL_INT32(scaled(e["longitudeDeg"], 1e7), d.lon);
  TEST_ASSERT_EQUAL_INT32(scaled(e["latitudeDeg"], 1e7), d.lat);
  TEST_ASSERT_EQUAL_INT32(scaled(e["wgsAltitudeM"], 1e3), d.wgs_alt_mm);
  TEST_ASSERT_EQUAL_INT32(scaled(e["mslAltitudeM"], 1e3), d.msl_alt_mm);
  TEST_ASSERT_EQUAL_UINT32(scaled(e["horizontalAccuracyM"], 1e3), d.h_acc_mm);
  TEST_ASSERT_EQUAL_UINT32(scaled(e["verticalAccuracyM"], 1e3), d.v_acc_mm);
  TEST_ASSERT_EQUAL_INT32(scaled(e["speedMps"], 1e3), d.speed_mmps);
  TEST_ASSERT_EQUAL_INT32(scaled(e["headingDeg"], 1e5), d.heading);
  TEST_ASSERT_EQUAL_UINT32(scaled(e["speedAccuracyMps"], 1e3), d.speed_acc_mmps);
  TEST_ASSERT_EQUAL_UINT32(scaled(e["headingAccuracyDeg"], 1e5), d.heading_acc);
  TEST_ASSERT_EQUAL_UINT16(scaled(e["pdop"], 1e2), d.pdop);
  TEST_ASSERT_EQUAL_UINT8(e["latLonFlags"].i64(), d.latlon_flags);
  TEST_ASSERT_EQUAL_UINT8(e["batteryRaw"].i64(), d.battery);
  TEST_ASSERT_EQUAL_UINT8(e["batteryPercent"].i64(), d.battery & 0x7F);
  TEST_ASSERT_EQUAL(e["charging"].b, (d.battery & 0x80) != 0);
  TEST_ASSERT_EQUAL_INT16(scaled(e["gxG"], 1e3), d.gx);
  TEST_ASSERT_EQUAL_INT16(scaled(e["gyG"], 1e3), d.gy);
  TEST_ASSERT_EQUAL_INT16(scaled(e["gzG"], 1e3), d.gz);
  TEST_ASSERT_EQUAL_INT16(scaled(e["rotXDps"], 1e2), d.rx);
  TEST_ASSERT_EQUAL_INT16(scaled(e["rotYDps"], 1e2), d.ry);
  TEST_ASSERT_EQUAL_INT16(scaled(e["rotZDps"], 1e2), d.rz);
  const bool good_fix = d.fix_status == 3 && (d.fix_flags & 0x01) != 0;
  TEST_ASSERT_EQUAL(e["goodFix"].b, good_fix);
}

void test_golden_live_payload_reencodes_byte_identical() {
  const Bytes golden = mj::hex(g_fx["liveData"]["hex"].str());
  uint8_t payload[rb::kLivePayloadLen];
  rb::encode_live(rb::decode_live(payload_of(golden)), payload);
  assert_bytes(golden, frame_of(rb::msg::kLive, payload, sizeof payload));
}

// --- Checksum vectors ---------------------------------------------------------

void test_checksum_vectors_all_validate() {
  const mj::Value& v = g_fx["checksumVectors"];
  TEST_ASSERT_TRUE(v.arr.size() >= 10);
  for (const mj::Value& c : v.arr) {
    const Bytes f = mj::hex(c["hex"].str());
    TEST_ASSERT_TRUE_MESSAGE(rb::validate_frame(f.data(), f.size()), c["name"].str().c_str());
    // Re-encode a partir de class/id/payload → bytes idênticos.
    assert_bytes(f, frame_of(f[3], payload_of(f), payload_len(f)));
    TEST_ASSERT_EQUAL_HEX8(rb::kClass, f[2]);
  }
}

void test_checksum_vectors_tampered_are_rejected() {
  for (const mj::Value& c : g_fx["checksumVectors"].arr) {
    const Bytes f = mj::hex(c["hex"].str());
    // Cada byte coberto pelo checksum, e cada byte do checksum, adulterado.
    for (size_t i = 2; i < f.size(); ++i) {
      Bytes t = f;
      t[i] ^= 0x01;
      TEST_ASSERT_FALSE_MESSAGE(rb::validate_frame(t.data(), t.size()), c["name"].str().c_str());
    }
  }
}

void test_live_frame_tampered_is_rejected() {
  const Bytes f = mj::hex(g_fx["liveData"]["hex"].str());
  for (size_t i = 2; i < f.size(); ++i) {
    Bytes t = f;
    t[i] ^= 0x80;
    TEST_ASSERT_FALSE(rb::validate_frame(t.data(), t.size()));
  }
}

// --- Respostas de comando ---------------------------------------------------

void test_download_start_reply_780() {
  const mj::Value& v = g_fx["downloadStartReply"];
  const Bytes f = mj::hex(v["hex"].str());
  TEST_ASSERT_TRUE(rb::validate_frame(f.data(), f.size()));
  TEST_ASSERT_EQUAL_HEX8(rb::msg::kDownload, f[3]);
  TEST_ASSERT_EQUAL_size_t(4, payload_len(f));
  const uint32_t n = rb::get_u32(payload_of(f));
  TEST_ASSERT_EQUAL_UINT32(v["expectedMaxMessages"].i64(), n);
  uint8_t p[4];
  rb::put_u32(p, n);
  assert_bytes(f, frame_of(rb::msg::kDownload, p, 4));
}

void test_erase_progress_59() {
  const mj::Value& v = g_fx["eraseProgress"];
  const Bytes f = mj::hex(v["hex"].str());
  TEST_ASSERT_TRUE(rb::validate_frame(f.data(), f.size()));
  TEST_ASSERT_EQUAL_HEX8(rb::msg::kErase, f[3]);
  TEST_ASSERT_EQUAL_size_t(1, payload_len(f));
  TEST_ASSERT_EQUAL_UINT8(v["percent"].i64(), f[6]);
  const uint8_t p = static_cast<uint8_t>(v["percent"].i64());
  assert_bytes(f, frame_of(rb::msg::kErase, &p, 1));
}

void test_ack_of_unlock() {
  const mj::Value& v = g_fx["ack"];
  const Bytes f = mj::hex(v["hex"].str());
  TEST_ASSERT_TRUE(rb::validate_frame(f.data(), f.size()));
  TEST_ASSERT_EQUAL_HEX8(v["positive"].b ? rb::msg::kAck : rb::msg::kNack, f[3]);
  TEST_ASSERT_EQUAL_UINT8(v["ackedClass"].i64(), f[6]);
  TEST_ASSERT_EQUAL_UINT8(v["ackedId"].i64(), f[7]);
  Bytes out(16);
  out.resize(rb::encode_ack(v["positive"].b, static_cast<uint8_t>(v["ackedClass"].i64()),
                            static_cast<uint8_t>(v["ackedId"].i64()), out.data(), out.size()));
  assert_bytes(f, out);
}

void test_unlock_123456() {
  const mj::Value& v = g_fx["unlock"];
  const Bytes f = mj::hex(v["hex"].str());
  TEST_ASSERT_TRUE(rb::validate_frame(f.data(), f.size()));
  TEST_ASSERT_EQUAL_HEX8(rb::msg::kUnlock, f[3]);
  TEST_ASSERT_EQUAL_UINT32(v["securityCode"].i64(), rb::get_u32(payload_of(f)));
  uint8_t p[4];
  rb::put_u32(p, static_cast<uint32_t>(v["securityCode"].i64()));
  assert_bytes(f, frame_of(rb::msg::kUnlock, p, 4));
}

void test_recording_config_preset_a() {
  const mj::Value& v = g_fx["recordingConfigPresetA"];
  const mj::Value& e = v["expected"];
  const Bytes f = mj::hex(v["hex"].str());
  TEST_ASSERT_TRUE(rb::validate_frame(f.data(), f.size()));
  TEST_ASSERT_EQUAL_HEX8(rb::msg::kRecConfig, f[3]);
  rb::RecordingConfig c;
  TEST_ASSERT_TRUE(rb::decode_rec_config(payload_of(f), payload_len(f), rb::RecordingConfig{}, &c));
  TEST_ASSERT_EQUAL_UINT8(e["enable"].b ? 1 : 0, c.enable);
  TEST_ASSERT_EQUAL_UINT8(e["dataRate"].i64(), c.rate);
  TEST_ASSERT_EQUAL_UINT8(e["flags"].i64(), c.flags);
  TEST_ASSERT_EQUAL_UINT16(e["stationarySpeedMmps"].i64(), c.stationary_speed_mmps);
  TEST_ASSERT_EQUAL_UINT16(e["stationaryIntervalS"].i64(), c.stationary_interval_s);
  TEST_ASSERT_EQUAL_UINT16(e["noFixIntervalS"].i64(), c.nofix_interval_s);
  TEST_ASSERT_EQUAL_UINT16(e["autoShutdownIntervalS"].i64(), c.shutdown_interval_s);
  uint8_t p[rb::kRecConfigLen];
  rb::encode_rec_config(c, p);
  assert_bytes(f, frame_of(rb::msg::kRecConfig, p, sizeof p));
}

// 0x22 e 0x26 não estão no fixture: round-trip (CLAUDE.md regra 3).
void test_status_round_trip() {
  rb::RecordingStatus s;
  s.recording = 1;
  s.memory_pct = 34;
  s.security = 0;
  s.stored = 67173;
  s.capacity = 24966;
  uint8_t p[rb::kStatusLen];
  rb::encode_status(s, p);
  TEST_ASSERT_EQUAL_HEX8(0x00, p[3]);  // reserved
  TEST_ASSERT_TRUE(rb::decode_status(p) == s);
  uint8_t p2[rb::kStatusLen];
  rb::encode_status(rb::decode_status(p), p2);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(p, p2, sizeof p);
}

void test_state_change_round_trip_and_field_order() {
  rb::RecordingConfig c;
  c.enable = 1;
  c.rate = rb::rate::k10Hz;
  c.flags = 0x1D;
  c.stationary_speed_mmps = 0;
  c.stationary_interval_s = 0;
  c.nofix_interval_s = 30;
  c.shutdown_interval_s = 600;
  const rb::StateChange s = rb::make_state_change(rb::rec_state::kRunning, c);
  uint8_t p[rb::kStateChangeLen];
  rb::encode_state_change(s, p);
  // Ordem do vendor (p.18): state, reserved, rate, flags.
  TEST_ASSERT_EQUAL_HEX8(0x01, p[0]);
  TEST_ASSERT_EQUAL_HEX8(0x00, p[1]);
  TEST_ASSERT_EQUAL_HEX8(0x01, p[2]);
  TEST_ASSERT_EQUAL_HEX8(0x1D, p[3]);
  TEST_ASSERT_TRUE(rb::decode_state_change(p) == s);
  uint8_t p2[rb::kStateChangeLen];
  rb::encode_state_change(rb::decode_state_change(p), p2);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(p, p2, sizeof p);
}

// Exemplos impressos no PDF do vendor (Rev 8 p.10, p.12, p.18) que o fixture
// não traz. Transcritos do PDF; o próprio checksum de cada um prova que a
// transcrição está certa (o vendor tem um typo conhecido só no ACK, p.21).
void test_vendor_pdf_examples_status_state_gnss() {
  const Bytes st = mj::hex("B5 62 FF 22 0C 00 00 22 01 00 65 06 01 00 00 00 03 00 BF 74");
  TEST_ASSERT_TRUE(rb::validate_frame(st.data(), st.size()));
  const rb::RecordingStatus s = rb::decode_status(payload_of(st));
  TEST_ASSERT_EQUAL_UINT8(0, s.recording);
  TEST_ASSERT_EQUAL_UINT8(34, s.memory_pct);
  TEST_ASSERT_EQUAL_UINT8(0x01, s.security);
  TEST_ASSERT_EQUAL_UINT32(67173, s.stored);
  TEST_ASSERT_EQUAL_UINT32(196608, s.capacity);
  TEST_ASSERT_EQUAL_UINT8(34, rb::memory_percent(s.stored, s.capacity));
  uint8_t p[rb::kStatusLen];
  rb::encode_status(s, p);
  assert_bytes(st, frame_of(rb::msg::kStatus, p, sizeof p));

  const Bytes sc = mj::hex("B5 62 FF 26 0C 00 01 00 00 1F 6D 05 0A 00 0A 00 0A 00 E1 F8");
  TEST_ASSERT_TRUE(rb::validate_frame(sc.data(), sc.size()));
  const rb::StateChange c = rb::decode_state_change(payload_of(sc));
  TEST_ASSERT_EQUAL_UINT8(rb::rec_state::kRunning, c.state);
  TEST_ASSERT_EQUAL_UINT8(rb::rate::k25Hz, c.rate);
  TEST_ASSERT_EQUAL_HEX8(0x1F, c.flags);
  TEST_ASSERT_EQUAL_UINT16(1389, c.stationary_speed_mmps);
  TEST_ASSERT_EQUAL_UINT16(10, c.stationary_interval_s);
  TEST_ASSERT_EQUAL_UINT16(10, c.nofix_interval_s);
  TEST_ASSERT_EQUAL_UINT16(10, c.shutdown_interval_s);
  uint8_t q[rb::kStateChangeLen];
  rb::encode_state_change(c, q);
  assert_bytes(sc, frame_of(rb::msg::kStateChange, q, sizeof q));

  const Bytes gc = mj::hex("B5 62 FF 27 03 00 08 01 14 46 20");
  TEST_ASSERT_TRUE(rb::validate_frame(gc.data(), gc.size()));
  const rb::GnssConfig g = rb::decode_gnss_config(payload_of(gc));
  TEST_ASSERT_EQUAL_UINT8(8, g.dyn_model);
  TEST_ASSERT_EQUAL_UINT8(1, g.speed_3d);
  TEST_ASSERT_EQUAL_UINT8(20, g.min_hacc_dm);  // "2.0 m" no PDF → décimos de metro
  uint8_t r[rb::kGnssConfigLen];
  rb::encode_gnss_config(g, r);
  assert_bytes(gc, frame_of(rb::msg::kGnssConfig, r, sizeof r));
}

// --- Clamp --------------------------------------------------------------------

void test_clamp_gyro_400dps_is_max_positive_never_negative() {
  TEST_ASSERT_EQUAL_INT16(32767, rb::wire_i16(400.0f * 100.0f));
  TEST_ASSERT_EQUAL_INT16(-32767, rb::wire_i16(-400.0f * 100.0f));
  TEST_ASSERT_EQUAL_INT16(32767, rb::clamp_i16(40000));
  TEST_ASSERT_EQUAL_INT16(-32767, rb::clamp_i16(-40000));
  TEST_ASSERT_EQUAL_INT16(32767, rb::wire_i16(1e30f));
  TEST_ASSERT_EQUAL_INT16(0, rb::wire_i16(std::nanf("")));
  // No fio: os bytes têm de ser 0x7FFF, nunca o wrap 0x9C40 (−25536).
  rb::LiveData d;
  d.rx = rb::wire_i16(40000.0f);
  uint8_t p[rb::kLivePayloadLen];
  rb::encode_live(d, p);
  TEST_ASSERT_EQUAL_HEX8(0xFF, p[74]);
  TEST_ASSERT_EQUAL_HEX8(0x7F, p[75]);
}

int main(int, char**) {
  if (!mj::load_file(std::string(FIXTURE_DIR) + "/racebox_test_vectors.json", &g_fx)) {
    UNITY_BEGIN();
    TEST_FAIL_MESSAGE("could not load fixture");
    return UNITY_END();
  }
  UNITY_BEGIN();
  RUN_TEST(test_golden_live_frame_built_from_decoded_fields_is_byte_identical);
  RUN_TEST(test_golden_live_frame_decodes_to_expected_fields);
  RUN_TEST(test_golden_live_payload_reencodes_byte_identical);
  RUN_TEST(test_checksum_vectors_all_validate);
  RUN_TEST(test_checksum_vectors_tampered_are_rejected);
  RUN_TEST(test_live_frame_tampered_is_rejected);
  RUN_TEST(test_download_start_reply_780);
  RUN_TEST(test_erase_progress_59);
  RUN_TEST(test_ack_of_unlock);
  RUN_TEST(test_unlock_123456);
  RUN_TEST(test_recording_config_preset_a);
  RUN_TEST(test_status_round_trip);
  RUN_TEST(test_state_change_round_trip_and_field_order);
  RUN_TEST(test_vendor_pdf_examples_status_state_gnss);
  RUN_TEST(test_clamp_gyro_400dps_is_max_positive_never_negative);
  return UNITY_END();
}
