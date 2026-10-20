// Gravação standalone e filtros (SPEC §7.2, §7.4) com relógio injetado.
#include <unity.h>

#include <vector>

#include "app/recorder.h"

void setUp() {}
void tearDown() {}

struct FakeSink : rec::Sink {
  struct Item {
    uint8_t tag;
    uint8_t state;  // para 0x26
    uint32_t itow;  // para 0x21
  };
  std::vector<Item> stored;
  std::vector<uint8_t> live;
  size_t capacity = 1000000;
  int shutdowns = 0;
  int self_stops = 0;
  bool store(uint8_t tag, const uint8_t* p, uint8_t len) override {
    if (stored.size() >= capacity) return false;
    Item it{tag, 0, 0};
    if (tag == 0x26 && len == 12) it.state = p[0];
    if (tag == 0x21 && len == 80) it.itow = rb::get_u32(p);
    stored.push_back(it);
    return true;
  }
  void live_state(const rb::StateChange& s) override { live.push_back(s.state); }
  void stopped_by_itself(bool) override { ++self_stops; }
  void shutdown() override { ++shutdowns; }
  size_t count(uint8_t tag) const {
    size_t n = 0;
    for (const Item& i : stored) n += i.tag == tag;
    return n;
  }
  std::vector<uint8_t> markers() const {
    std::vector<uint8_t> m;
    for (const Item& i : stored)
      if (i.tag == 0x26) m.push_back(i.state);
    return m;
  }
};

static rb::RecordingConfig preset_a() {  // Track Day (R9)
  rb::RecordingConfig c;
  c.enable = 1;
  c.rate = rb::rate::k25Hz;
  c.flags = 0x1F;
  c.stationary_speed_mmps = 1389;
  c.stationary_interval_s = 30;
  c.nofix_interval_s = 30;
  c.shutdown_interval_s = 300;
  return c;
}
static rb::RecordingConfig preset_b() {  // Standing Start (R9)
  rb::RecordingConfig c;
  c.enable = 1;
  c.rate = rb::rate::k25Hz;
  c.flags = 0x1D;
  c.nofix_interval_s = 30;
  c.shutdown_interval_s = 600;
  return c;
}

struct Drive {
  rec::Recorder& r;
  uint32_t t = 0;
  uint32_t itow = 1000;
  bool connected = true;
  void frames(int n, bool fix, int32_t speed_mmps, uint32_t step_ms = 100) {
    for (int i = 0; i < n; ++i) {
      rb::LiveData d;
      d.itow = itow;
      d.fix_status = fix ? 3 : 0;
      d.fix_flags = fix ? 1 : 0;
      d.speed_mmps = speed_mmps;
      uint8_t p[80];
      rb::encode_live(d, p);
      r.on_frame(d, p, t, connected);
      t += step_ms;
      itow += step_ms;
    }
  }
};

void test_start_stores_marker_even_waiting_for_fix() {
  FakeSink s;
  rec::Recorder r(s);
  r.start(preset_a(), 0, true);
  TEST_ASSERT_TRUE(r.state() == rec::State::kWaitingFix);
  TEST_ASSERT_EQUAL_size_t(1, s.stored.size());
  TEST_ASSERT_EQUAL_HEX8(0x26, s.stored[0].tag);
  TEST_ASSERT_EQUAL_UINT8(1, s.stored[0].state);
  TEST_ASSERT_EQUAL_size_t(1, s.live.size());
  Drive d{r};
  d.frames(50, false, 0);  // sem fix: nada gravado
  TEST_ASSERT_EQUAL_size_t(0, s.count(0x21));
  d.frames(10, true, 20000);
  TEST_ASSERT_TRUE(r.state() == rec::State::kRunning);
  TEST_ASSERT_EQUAL_size_t(10, s.count(0x21));
}

void test_no_wait_for_fix_records_everything() {
  FakeSink s;
  rec::Recorder r(s);
  rb::RecordingConfig c;
  c.rate = rb::rate::k10Hz;
  c.flags = 0;
  r.start(c, 0, true);
  Drive d{r};
  d.frames(30, false, 0);
  TEST_ASSERT_EQUAL_size_t(30, s.count(0x21));
}

void test_stationary_pause_and_resume_without_stored_marker() {
  FakeSink s;
  rec::Recorder r(s);
  r.start(preset_a(), 0, true);
  Drive d{r};
  d.frames(100, true, 20000);  // andando
  d.frames(299, true, 500);    // parado 29,9 s: ainda grava
  TEST_ASSERT_TRUE(r.state() == rec::State::kRunning);
  d.frames(2, true, 500);      // 30 s → pausa
  TEST_ASSERT_TRUE(r.state() == rec::State::kPaused);
  TEST_ASSERT_TRUE(r.pause_reason() == rec::PauseReason::kStationary);
  const size_t before = s.stored.size();
  d.frames(200, true, 500);  // parado: nada gravado
  TEST_ASSERT_EQUAL_size_t(before, s.stored.size());
  d.frames(1, true, 1389);  // um registro acima do limiar retoma
  TEST_ASSERT_TRUE(r.state() == rec::State::kRunning);
  TEST_ASSERT_EQUAL_HEX8(0x21, s.stored.back().tag);  // sem marcador gravado na retomada
  const std::vector<uint8_t> m = s.markers();
  TEST_ASSERT_EQUAL_size_t(2, m.size());  // start + pause
  TEST_ASSERT_EQUAL_UINT8(2, m[1]);
  TEST_ASSERT_EQUAL_UINT8(1, s.live.back());  // retomada reportada ao vivo
}

void test_stationary_needs_valid_fix() {
  FakeSink s;
  rec::Recorder r(s);
  rb::RecordingConfig c = preset_a();
  c.flags = rb::filter::kStationary;  // só stationary
  r.start(c, 0, true);
  Drive d{r};
  d.frames(600, false, 0);  // 60 s parado SEM fix: filtro não age
  TEST_ASSERT_TRUE(r.state() == rec::State::kRunning);
  TEST_ASSERT_EQUAL_size_t(600, s.count(0x21));
}

void test_nofix_pause_and_resume() {
  FakeSink s;
  rec::Recorder r(s);
  r.start(preset_b(), 0, true);
  Drive d{r};
  d.frames(50, true, 0);  // preset B: parado com fix continua gravando (sem stationary)
  TEST_ASSERT_EQUAL_size_t(50, s.count(0x21));
  d.frames(300, false, 0);  // 30 s sem fix → pausa no 301º
  TEST_ASSERT_TRUE(r.state() == rec::State::kRunning);
  d.frames(1, false, 0);
  TEST_ASSERT_TRUE(r.state() == rec::State::kPaused);
  TEST_ASSERT_TRUE(r.pause_reason() == rec::PauseReason::kNoFix);
  TEST_ASSERT_EQUAL_size_t(350, s.count(0x21));  // registros durante o intervalo ficam
  d.frames(10, false, 0);
  TEST_ASSERT_EQUAL_size_t(350, s.count(0x21));
  d.frames(1, true, 0);
  TEST_ASSERT_TRUE(r.state() == rec::State::kRunning);
  TEST_ASSERT_EQUAL_size_t(351, s.count(0x21));
}

void test_auto_shutdown_waits_for_data_and_no_client() {
  FakeSink s;
  rec::Recorder r(s);
  r.start(preset_a(), 0, true);
  Drive d{r};
  d.connected = false;
  d.frames(4000, false, 0);  // 400 s sem fix e sem dado: b4 segura o desligamento
  TEST_ASSERT_EQUAL_INT(0, s.shutdowns);
  d.frames(10, true, 20000);  // grava algo
  d.frames(3000, false, 0);   // 30 s até a pausa no-fix, depois 270 s parado
  TEST_ASSERT_EQUAL_INT(0, s.shutdowns);
  d.frames(2000, false, 0);  // passa de 300 s sem gravar
  TEST_ASSERT_EQUAL_INT(1, s.shutdowns);
  TEST_ASSERT_EQUAL_UINT8(0, s.markers().back());  // marcador state=0 no auto-shutdown
  TEST_ASSERT_TRUE(r.state() == rec::State::kStopped);
}

void test_auto_shutdown_never_with_client_connected() {
  FakeSink s;
  rec::Recorder r(s);
  rb::RecordingConfig c = preset_a();
  c.flags = rb::filter::kWaitForFix | rb::filter::kAutoShutdown;  // sem b4
  r.start(c, 0, true);
  Drive d{r};
  d.connected = true;
  d.frames(5000, false, 0);
  TEST_ASSERT_EQUAL_INT(0, s.shutdowns);
  d.connected = false;  // timer só começa ao desconectar
  d.frames(2999, false, 0);
  TEST_ASSERT_EQUAL_INT(0, s.shutdowns);
  d.frames(2, false, 0);
  TEST_ASSERT_EQUAL_INT(1, s.shutdowns);
}

void test_memory_full_stops_with_marker_if_it_fits() {
  FakeSink s;
  s.capacity = 21;  // start + 19 registros + 1 marcador
  rec::Recorder r(s);
  rb::RecordingConfig c;
  c.flags = 0;
  r.start(c, 0, true);
  Drive d{r};
  d.frames(19, true, 10000);
  TEST_ASSERT_TRUE(r.recording());
  d.frames(1, true, 10000);  // 21º item = cheio? ainda cabe (20 registros+start=21)
  d.frames(1, true, 10000);  // não cabe: para
  TEST_ASSERT_FALSE(r.recording());
  TEST_ASSERT_EQUAL_INT(1, s.self_stops);
  TEST_ASSERT_EQUAL_UINT8(0, s.live.back());
  d.frames(10, true, 10000);  // live segue, nada mais gravado
  TEST_ASSERT_EQUAL_size_t(21, s.stored.size());
}

void test_reconfigure_markers_only_when_rate_changes() {
  FakeSink s;
  rec::Recorder r(s);
  r.start(preset_a(), 0, true);
  rb::RecordingConfig c = preset_a();
  c.stationary_interval_s = 45;  // muda filtro, mesma taxa
  r.reconfigure(c, 1000);
  TEST_ASSERT_EQUAL_size_t(1, s.markers().size());
  c.rate = rb::rate::k10Hz;
  r.reconfigure(c, 2000);
  const std::vector<uint8_t> m = s.markers();
  TEST_ASSERT_EQUAL_size_t(3, m.size());
  TEST_ASSERT_EQUAL_UINT8(0, m[1]);
  TEST_ASSERT_EQUAL_UINT8(1, m[2]);
  TEST_ASSERT_EQUAL_UINT8(rb::rate::k10Hz, r.config().rate);
}

void test_stop_stores_marker_zero() {
  FakeSink s;
  rec::Recorder r(s);
  r.start(preset_b(), 0, true);
  r.stop();
  TEST_ASSERT_FALSE(r.recording());
  const std::vector<uint8_t> m = s.markers();
  TEST_ASSERT_EQUAL_size_t(2, m.size());
  TEST_ASSERT_EQUAL_UINT8(0, m[1]);
  r.stop();  // idempotente
  TEST_ASSERT_EQUAL_size_t(2, s.markers().size());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_start_stores_marker_even_waiting_for_fix);
  RUN_TEST(test_no_wait_for_fix_records_everything);
  RUN_TEST(test_stationary_pause_and_resume_without_stored_marker);
  RUN_TEST(test_stationary_needs_valid_fix);
  RUN_TEST(test_nofix_pause_and_resume);
  RUN_TEST(test_auto_shutdown_waits_for_data_and_no_client);
  RUN_TEST(test_auto_shutdown_never_with_client_connected);
  RUN_TEST(test_memory_full_stops_with_marker_if_it_fits);
  RUN_TEST(test_reconfigure_markers_only_when_rate_changes);
  RUN_TEST(test_stop_stores_marker_zero);
  return UNITY_END();
}
