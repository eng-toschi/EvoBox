// Comandos Mini S (SPEC §7): ACK/NACK, idempotência do 0x25 (R11), download
// em ordem, erase, 0x27, unlock — entradas dos golden vectors quando existem.
#include <unity.h>

#include <string>
#include <vector>

#include "../common/mini_json.h"
#include "app/commands.h"
#include "assembler.h"

using Bytes = std::vector<uint8_t>;

static mj::Value g_fx;

void setUp() {}
void tearDown() {}

struct FakeDevice : cmd::Device {
  bool rec = false, dl = false, er = false, can_start = true, prepare_ok = true, erase_ok = true;
  rb::RecordingConfig cfg;
  rb::GnssConfig gcfg;
  rb::RecordingStatus st;
  uint32_t expected = 780;
  int starts = 0, reconfigs = 0, stops = 0, prepares = 0, begins = 0, dl_cancels = 0, erases = 0,
      er_cancels = 0, gnss_sets = 0;
  std::vector<std::string>* log = nullptr;

  bool recording() const override { return rec; }
  rb::RecordingConfig rec_config() const override { return cfg; }
  rb::RecordingStatus status() const override { return st; }
  bool can_start_recording() const override { return can_start; }
  bool download_active() const override { return dl; }
  bool erase_active() const override { return er; }
  void start_recording(const rb::RecordingConfig& c) override {
    ++starts;
    cfg = c;
    rec = true;
  }
  void reconfigure_recording(const rb::RecordingConfig& c) override {
    ++reconfigs;
    cfg = c;
  }
  void stop_recording() override {
    ++stops;
    rec = false;
  }
  bool prepare_download(uint32_t* e) override {
    ++prepares;
    *e = expected;
    return prepare_ok;
  }
  void begin_download() override {
    ++begins;
    dl = true;
    if (log) log->push_back("begin");
  }
  void cancel_download() override { ++dl_cancels; }
  bool start_erase() override {
    ++erases;
    if (erase_ok) er = true;
    return erase_ok;
  }
  void cancel_erase() override { ++er_cancels; }
  rb::GnssConfig gnss_config() const override { return gcfg; }
  void set_gnss_config(const rb::GnssConfig& c) override {
    ++gnss_sets;
    gcfg = c;
  }
};

struct FakeOut : cmd::Output {
  std::vector<Bytes> resp, bulks;
  std::vector<std::string>* log = nullptr;
  void response(const uint8_t* f, size_t n) override { resp.emplace_back(f, f + n); }
  void bulk(const uint8_t* f, size_t n) override {
    bulks.emplace_back(f, f + n);
    if (log) log->push_back("bulk");
  }
};

// Alimenta bytes de um frame (como chegariam no RX) no processador.
static void send(cmd::Processor& p, const Bytes& wire) {
  rb::FrameAssembler a;
  a.feed(wire.data(), wire.size(), [&](const rb::Frame& f) { p.handle(f); });
}

static Bytes frame(uint8_t id, const Bytes& payload) {
  Bytes b(payload.size() + 8);
  b.resize(rb::encode_frame(0xFF, id, payload.data(), payload.size(), b.data(), b.size()));
  return b;
}

static Bytes ack_bytes(bool positive, uint8_t id) {
  Bytes b(16);
  b.resize(rb::encode_ack(positive, 0xFF, id, b.data(), b.size()));
  return b;
}

static Bytes fx(const char* key) { return mj::hex(g_fx[key]["hex"].str()); }
static Bytes fx_checksum(const char* name) {
  for (const mj::Value& v : g_fx["checksumVectors"].arr)
    if (v["name"].str().rfind(name, 0) == 0) return mj::hex(v["hex"].str());
  TEST_FAIL_MESSAGE(name);
  return {};
}

void test_unlock_always_acks_with_golden_bytes() {
  FakeDevice d;
  FakeOut o;
  cmd::Processor p(d, o);
  send(p, fx("unlock"));
  TEST_ASSERT_EQUAL_size_t(1, o.resp.size());
  const Bytes golden = fx("ack");
  TEST_ASSERT_EQUAL_size_t(golden.size(), o.resp[0].size());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(golden.data(), o.resp[0].data(), golden.size());
  send(p, frame(0x30, {0, 0, 0, 0}));  // qualquer código
  TEST_ASSERT_TRUE(o.resp[1] == golden);
}

void test_status_reply() {
  FakeDevice d;
  d.st.recording = 1;
  d.st.memory_pct = 7;
  d.st.stored = 1234;
  d.st.capacity = 25872;
  FakeOut o;
  cmd::Processor p(d, o);
  send(p, fx_checksum("recordingStatusQuery"));
  TEST_ASSERT_EQUAL_size_t(1, o.resp.size());
  const Bytes& r = o.resp[0];
  TEST_ASSERT_TRUE(rb::validate_frame(r.data(), r.size()));
  TEST_ASSERT_EQUAL_HEX8(0x22, r[3]);
  TEST_ASSERT_TRUE(rb::decode_status(r.data() + 6) == d.st);
  TEST_ASSERT_EQUAL_HEX8(0x00, r[6 + 2]);  // security = 0
  send(p, frame(0x22, {1}));
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(false, 0x22));
}

void test_preset_a_starts_once_then_idempotent_on_every_reconnect() {
  FakeDevice d;
  FakeOut o;
  cmd::Processor p(d, o);
  const Bytes preset = fx("recordingConfigPresetA");
  send(p, preset);
  TEST_ASSERT_EQUAL_INT(1, d.starts);
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(true, 0x25));
  TEST_ASSERT_EQUAL_UINT16(1389, d.cfg.stationary_speed_mmps);
  for (int i = 0; i < 10; ++i) send(p, preset);  // R11: reescrito a cada reconexão
  TEST_ASSERT_EQUAL_INT(1, d.starts);
  TEST_ASSERT_EQUAL_INT(0, d.reconfigs);
  TEST_ASSERT_EQUAL_INT(0, d.stops);
  TEST_ASSERT_EQUAL_UINT32(10, p.stats().idempotent_25);
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(true, 0x25));
}

void test_preset_b_while_recording_reconfigures() {
  FakeDevice d;
  FakeOut o;
  cmd::Processor p(d, o);
  send(p, fx("recordingConfigPresetA"));
  rb::RecordingConfig b;
  b.enable = 1;
  b.rate = 0;
  b.flags = 0x1D;
  b.nofix_interval_s = 30;
  b.shutdown_interval_s = 600;
  uint8_t pl[12];
  rb::encode_rec_config(b, pl);
  send(p, frame(0x25, Bytes(pl, pl + 12)));
  TEST_ASSERT_EQUAL_INT(1, d.reconfigs);
  TEST_ASSERT_EQUAL_HEX8(0x1D, d.cfg.flags);
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(true, 0x25));
}

void test_disable_stops_and_disable_again_is_noop() {
  FakeDevice d;
  FakeOut o;
  cmd::Processor p(d, o);
  send(p, fx("recordingConfigPresetA"));
  send(p, frame(0x25, Bytes(12, 0)));
  TEST_ASSERT_EQUAL_INT(1, d.stops);
  send(p, frame(0x25, Bytes(12, 0)));
  TEST_ASSERT_EQUAL_INT(1, d.stops);
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(true, 0x25));
}

void test_invalid_rate_full_memory_and_bad_length_nack() {
  FakeDevice d;
  FakeOut o;
  cmd::Processor p(d, o);
  Bytes pl = {1, 5, 0x1F, 0, 0x6D, 0x05, 30, 0, 30, 0, 0x2C, 0x01};
  send(p, frame(0x25, pl));
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(false, 0x25));
  d.can_start = false;
  pl[1] = 0;
  send(p, frame(0x25, pl));
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(false, 0x25));
  TEST_ASSERT_EQUAL_INT(0, d.starts);
  d.can_start = true;
  send(p, frame(0x25, Bytes(pl.begin(), pl.begin() + 11)));  // 11 B
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(false, 0x25));
}

void test_ten_byte_write_keeps_shutdown_interval() {
  FakeDevice d;
  d.cfg.shutdown_interval_s = 777;
  FakeOut o;
  cmd::Processor p(d, o);
  send(p, frame(0x25, {1, 1, 0x05, 0, 0, 0, 0, 0, 20, 0}));
  TEST_ASSERT_EQUAL_INT(1, d.starts);
  TEST_ASSERT_EQUAL_UINT16(777, d.cfg.shutdown_interval_s);
  TEST_ASSERT_EQUAL_UINT8(1, d.cfg.rate);
}

void test_rec_config_read_reflects_recording() {
  FakeDevice d;
  FakeOut o;
  cmd::Processor p(d, o);
  send(p, frame(0x25, {}));
  TEST_ASSERT_EQUAL_size_t(20, o.resp.back().size());
  TEST_ASSERT_EQUAL_UINT8(0, o.resp.back()[6]);
  send(p, fx("recordingConfigPresetA"));
  send(p, frame(0x25, {}));
  const Bytes golden = fx("recordingConfigPresetA");  // mesma config, enable=1
  TEST_ASSERT_TRUE(o.resp.back() == golden);
}

void test_download_reply_goes_first_in_stream() {
  FakeDevice d;
  FakeOut o;
  std::vector<std::string> order;
  d.log = &order;
  o.log = &order;
  cmd::Processor p(d, o);
  send(p, fx_checksum("downloadStart (§5.3)"));
  TEST_ASSERT_EQUAL_INT(1, d.prepares);
  TEST_ASSERT_EQUAL_INT(1, d.begins);
  TEST_ASSERT_EQUAL_size_t(2, order.size());
  TEST_ASSERT_EQUAL_STRING("bulk", order[0].c_str());
  TEST_ASSERT_EQUAL_STRING("begin", order[1].c_str());
  const Bytes golden = fx("downloadStartReply");  // 780 registros
  TEST_ASSERT_TRUE(o.bulks[0] == golden);
  TEST_ASSERT_TRUE(o.resp.empty());
  // Cancel com download ativo: quem manda o ACK é o dono, depois de drenar.
  send(p, fx_checksum("downloadCancel"));
  TEST_ASSERT_EQUAL_INT(1, d.dl_cancels);
  TEST_ASSERT_TRUE(o.resp.empty());
}

void test_download_nacks_and_idle_cancel() {
  FakeDevice d;
  FakeOut o;
  cmd::Processor p(d, o);
  d.er = true;
  send(p, fx_checksum("downloadStart (§5.3)"));
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(false, 0x23));
  d.er = false;
  d.prepare_ok = false;
  send(p, fx_checksum("downloadStart (§5.3)"));
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(false, 0x23));
  send(p, fx_checksum("downloadCancel"));  // nada ativo → ACK
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(true, 0x23));
}

void test_download_works_while_recording() {
  FakeDevice d;
  FakeOut o;
  cmd::Processor p(d, o);
  send(p, fx("recordingConfigPresetA"));
  send(p, fx_checksum("downloadStart (§5.3)"));
  TEST_ASSERT_EQUAL_INT(1, d.begins);  // R12
  TEST_ASSERT_TRUE(d.rec);
  // Racevo reenvia o preset (reconexão) durante o download: idempotente → ACK.
  send(p, fx("recordingConfigPresetA"));
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(true, 0x25));
  // Config diferente durante o download → NACK.
  send(p, frame(0x25, {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(false, 0x25));
  TEST_ASSERT_EQUAL_INT(0, d.reconfigs);
}

void test_erase_rules() {
  FakeDevice d;
  FakeOut o;
  cmd::Processor p(d, o);
  d.rec = true;
  send(p, fx_checksum("eraseStart"));
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(false, 0x24));
  d.rec = false;
  send(p, fx_checksum("eraseStart"));
  TEST_ASSERT_EQUAL_INT(1, d.erases);
  const size_t n = o.resp.size();
  send(p, fx_checksum("eraseCancel"));
  TEST_ASSERT_EQUAL_INT(1, d.er_cancels);
  TEST_ASSERT_EQUAL_size_t(n, o.resp.size());  // ACK vem do dono ao parar
  send(p, fx_checksum("eraseStart"));          // já em andamento
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(false, 0x24));
  d.er = false;
  send(p, fx_checksum("eraseCancel"));
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(true, 0x24));
}

void test_gnss_config() {
  FakeDevice d;
  FakeOut o;
  cmd::Processor p(d, o);
  send(p, frame(0x27, {}));
  const Bytes expect_default = frame(0x27, {4, 0, 30});
  TEST_ASSERT_TRUE(o.resp.back() == expect_default);
  send(p, frame(0x27, {8, 1, 0x14}));  // exemplo do vendor (p.10)
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(true, 0x27));
  TEST_ASSERT_EQUAL_UINT8(8, d.gcfg.dyn_model);
  TEST_ASSERT_EQUAL_UINT8(20, d.gcfg.min_hacc_dm);
  send(p, frame(0x27, {9, 0, 30}));
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(false, 0x27));
  send(p, frame(0x27, {4, 0}));
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(false, 0x27));
  TEST_ASSERT_EQUAL_INT(1, d.gnss_sets);
}

void test_unknown_id_nack_and_foreign_class_ignored() {
  FakeDevice d;
  FakeOut o;
  cmd::Processor p(d, o);
  send(p, frame(0x99, {1, 2}));
  TEST_ASSERT_TRUE(o.resp.back() == ack_bytes(false, 0x99));
  // UBX cru (CFG-RATE) nunca é repassado nem respondido
  const Bytes raw = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
  const size_t n = o.resp.size();
  send(p, raw);
  TEST_ASSERT_EQUAL_size_t(n, o.resp.size());
  TEST_ASSERT_EQUAL_UINT32(1, p.stats().ignored_foreign_class);
}

int main(int, char**) {
  if (!mj::load_file(std::string(FIXTURE_DIR) + "/racebox_test_vectors.json", &g_fx)) return 1;
  UNITY_BEGIN();
  RUN_TEST(test_unlock_always_acks_with_golden_bytes);
  RUN_TEST(test_status_reply);
  RUN_TEST(test_preset_a_starts_once_then_idempotent_on_every_reconnect);
  RUN_TEST(test_preset_b_while_recording_reconfigures);
  RUN_TEST(test_disable_stops_and_disable_again_is_noop);
  RUN_TEST(test_invalid_rate_full_memory_and_bad_length_nack);
  RUN_TEST(test_ten_byte_write_keeps_shutdown_interval);
  RUN_TEST(test_rec_config_read_reflects_recording);
  RUN_TEST(test_download_reply_goes_first_in_stream);
  RUN_TEST(test_download_nacks_and_idle_cancel);
  RUN_TEST(test_download_works_while_recording);
  RUN_TEST(test_erase_rules);
  RUN_TEST(test_gnss_config);
  RUN_TEST(test_unknown_id_nack_and_foreign_class_ignored);
  return UNITY_END();
}
