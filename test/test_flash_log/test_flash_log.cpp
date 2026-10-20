// Log circular append-only contra uma flash NOR simulada, com queda de energia
// e escrita rasgada (SPEC §8.2, A8).
#include <unity.h>

#include <cstring>
#include <initializer_list>
#include <vector>

#include "storage/flash_log.h"

using storage::FlashIo;
using storage::FlashLog;

void setUp() {}
void tearDown() {}

// Flash NOR: escrita só leva bits 1→0; erase por setor leva tudo a 0xFF.
// Detecta reprogramação de byte já escrito (o log nunca deve fazer isso) e
// simula queda de energia no meio de uma escrita.
class NorSim : public FlashIo {
 public:
  explicit NorSim(size_t sectors) : mem_(sectors * kSector, 0xFF) {}
  size_t size() const override { return mem_.size(); }
  bool read(size_t off, void* buf, size_t len) override {
    if (off + len > mem_.size()) return false;
    std::memcpy(buf, mem_.data() + off, len);
    return true;
  }
  bool write(size_t off, const void* buf, size_t len) override {
    if (dead_ || off + len > mem_.size()) return false;
    const uint8_t* p = static_cast<const uint8_t*>(buf);
    for (size_t i = 0; i < len; ++i) {
      if (budget_ == 0) {  // energia acabou no meio da escrita
        dead_ = true;
        return false;
      }
      if (budget_ > 0) --budget_;
      if (mem_[off + i] != 0xFF) ++reprograms_;
      mem_[off + i] &= p[i];
    }
    ++writes_;
    return true;
  }
  bool erase_sector(size_t s) override {
    if (dead_ || (s + 1) * kSector > mem_.size()) return false;
    std::memset(mem_.data() + s * kSector, 0xFF, kSector);
    ++erases_;
    return true;
  }
  void cut_power_after(long bytes) { budget_ = bytes; }
  void power_on() {
    dead_ = false;
    budget_ = -1;
  }
  size_t reprograms_ = 0, writes_ = 0, erases_ = 0;
  std::vector<uint8_t> mem_;

 private:
  long budget_ = -1;
  bool dead_ = false;
};

static void hist_payload(uint32_t i, uint8_t* p) {
  for (int k = 0; k < 80; ++k) p[k] = static_cast<uint8_t>(i * 31 + k);
}
static void state_payload(uint32_t i, uint8_t* p) {
  std::memset(p, 0, 12);
  p[0] = static_cast<uint8_t>(i % 3);
  p[2] = 1;
  p[3] = 0x1F;
}

// Sequência determinística: a cada 25 registros, um marcador.
static bool is_marker(uint32_t i) { return i % 25 == 0; }

static void append_seq(FlashLog& log, uint32_t from, uint32_t n) {
  uint8_t p[80];
  for (uint32_t i = from; i < from + n; ++i) {
    if (is_marker(i)) {
      state_payload(i, p);
      TEST_ASSERT_TRUE(log.append(FlashLog::kTagState, p, 12) == FlashLog::Append::kOk);
    } else {
      hist_payload(i, p);
      TEST_ASSERT_TRUE(log.append(FlashLog::kTagHistory, p, 80) == FlashLog::Append::kOk);
    }
  }
}

// Lê tudo e confere que é exatamente a sequência [0, n).
static void verify_seq(FlashLog& log, uint32_t n) {
  size_t cur = 0;
  uint8_t tag, len, p[80], e[80];
  for (uint32_t i = 0; i < n; ++i) {
    TEST_ASSERT_TRUE_MESSAGE(log.read_next(&cur, log.flushed_end(), &tag, p, &len), "record missing");
    if (is_marker(i)) {
      state_payload(i, e);
      TEST_ASSERT_EQUAL_HEX8(0x26, tag);
      TEST_ASSERT_EQUAL_UINT8(12, len);
      TEST_ASSERT_EQUAL_HEX8_ARRAY(e, p, 12);
    } else {
      hist_payload(i, e);
      TEST_ASSERT_EQUAL_HEX8(0x21, tag);
      TEST_ASSERT_EQUAL_UINT8(80, len);
      TEST_ASSERT_EQUAL_HEX8_ARRAY(e, p, 80);
    }
  }
  TEST_ASSERT_FALSE(log.read_next(&cur, log.flushed_end(), &tag, p, &len));
}

static uint32_t markers_in(uint32_t n) {
  uint32_t m = 0;
  for (uint32_t i = 0; i < n; ++i) m += is_marker(i);
  return m;
}

void test_empty_mount() {
  NorSim f(16);
  FlashLog log(f);
  TEST_ASSERT_TRUE(log.mount());
  TEST_ASSERT_EQUAL_UINT32(0, log.history_count());
  TEST_ASSERT_EQUAL_UINT32(0, log.state_count());
  TEST_ASSERT_EQUAL_UINT8(0, log.memory_pct());
  TEST_ASSERT_EQUAL_UINT32(16 * 49, log.capacity_history());
  TEST_ASSERT_FALSE(log.full());
  TEST_ASSERT_TRUE(log.can_start());
}

void test_append_flush_remount_read_back() {
  NorSim f(32);
  {
    FlashLog log(f);
    TEST_ASSERT_TRUE(log.mount());
    append_seq(log, 0, 700);  // atravessa vários setores
    TEST_ASSERT_TRUE(log.flush());
    verify_seq(log, 700);
  }
  FlashLog again(f);
  TEST_ASSERT_TRUE(again.mount());
  TEST_ASSERT_EQUAL_UINT32(700 - markers_in(700), again.history_count());
  TEST_ASSERT_EQUAL_UINT32(markers_in(700), again.state_count());
  verify_seq(again, 700);
  append_seq(again, 700, 300);  // continua de onde parou
  TEST_ASSERT_TRUE(again.flush());
  verify_seq(again, 1000);
  TEST_ASSERT_EQUAL_size_t(0, f.reprograms_);
}

void test_records_never_straddle_sectors() {
  NorSim f(4);
  FlashLog log(f);
  TEST_ASSERT_TRUE(log.mount());
  uint8_t p[80] = {};
  for (int i = 0; i < 49; ++i) log.append(FlashLog::kTagHistory, p, 80);  // 4067 B
  TEST_ASSERT_EQUAL_size_t(49 * 83, log.used_bytes());
  log.append(FlashLog::kTagHistory, p, 80);  // não cabe nos 29 B restantes
  TEST_ASSERT_EQUAL_size_t(4096 + 83, log.used_bytes());
  log.flush();
  for (size_t i = 49 * 83; i < 4096; ++i) TEST_ASSERT_EQUAL_HEX8(0xFF, f.mem_[i]);
  FlashLog again(f);
  again.mount();
  TEST_ASSERT_EQUAL_UINT32(50, again.history_count());
}

void test_sector_exactly_full_then_continue() {
  NorSim f(4);
  FlashLog log(f);
  TEST_ASSERT_TRUE(log.mount());
  uint8_t p[80] = {};
  for (int i = 0; i < 47; ++i) log.append(FlashLog::kTagHistory, p, 80);  // 3901
  for (int i = 0; i < 13; ++i) log.append(FlashLog::kTagState, p, 12);    // +195 = 4096
  TEST_ASSERT_EQUAL_size_t(4096, log.used_bytes());
  // sem flush no meio: o próximo append tem de programar o fim do setor anterior
  log.append(FlashLog::kTagHistory, p, 80);
  log.flush();
  FlashLog again(f);
  again.mount();
  TEST_ASSERT_EQUAL_UINT32(48, again.history_count());
  TEST_ASSERT_EQUAL_UINT32(13, again.state_count());
  TEST_ASSERT_EQUAL_size_t(0, f.reprograms_);
}

void test_full_stops_never_overwrites() {
  NorSim f(3);
  FlashLog log(f);
  TEST_ASSERT_TRUE(log.mount());
  uint8_t p[80] = {};
  uint32_t ok = 0;
  while (log.append(FlashLog::kTagHistory, p, 80) == FlashLog::Append::kOk) ++ok;
  TEST_ASSERT_EQUAL_UINT32(3 * 49, ok);
  TEST_ASSERT_TRUE(log.full());
  TEST_ASSERT_EQUAL_UINT8(100, log.memory_pct());
  TEST_ASSERT_FALSE(log.can_start());
  // ainda cabe um marcador nos 29 B de sobra do último setor (state=0 ao encher)
  TEST_ASSERT_TRUE(log.append(FlashLog::kTagState, p, 12) == FlashLog::Append::kOk);
  log.flush();
  TEST_ASSERT_EQUAL_size_t(0, f.reprograms_);
  FlashLog again(f);
  again.mount();
  TEST_ASSERT_EQUAL_UINT32(3 * 49, again.history_count());
  TEST_ASSERT_TRUE(again.full());
}

void test_power_loss_loses_only_unflushed() {
  NorSim f(16);
  {
    FlashLog log(f);
    log.mount();
    append_seq(log, 0, 120);
    log.flush();
    append_seq(log, 120, 30);  // ainda no buffer de RAM
    // queda de energia: o objeto some sem flush
  }
  FlashLog again(f);
  again.mount();
  TEST_ASSERT_EQUAL_UINT32(120 - markers_in(120), again.history_count());
  verify_seq(again, 120);
}

void test_torn_write_closes_sector_and_log_continues() {
  for (long cut : {0L, 1L, 2L, 40L, 82L}) {
    NorSim f(16);
    {
      FlashLog log(f);
      log.mount();
      append_seq(log, 0, 60);
      log.flush();
      uint8_t p[80];
      hist_payload(9999, p);
      log.append(FlashLog::kTagHistory, p, 80);
      f.cut_power_after(cut);  // energia cai no meio deste registro
      TEST_ASSERT_FALSE(log.flush());
    }
    f.power_on();
    FlashLog again(f);
    TEST_ASSERT_TRUE(again.mount());
    TEST_ASSERT_EQUAL_UINT32(60 - markers_in(60), again.history_count());
    verify_seq(again, 60);
    // o log segue gravando sem reprogramar bytes rasgados
    append_seq(again, 60, 100);
    again.flush();
    TEST_ASSERT_EQUAL_size_t(0, f.reprograms_);
    FlashLog third(f);
    third.mount();
    TEST_ASSERT_EQUAL_UINT32(160 - markers_in(160), third.history_count());
    verify_seq(third, 160);
  }
}

void test_read_to_snapshot_excludes_later_records() {
  NorSim f(16);
  FlashLog log(f);
  log.mount();
  append_seq(log, 0, 80);
  log.flush();
  const size_t snap = log.flushed_end();
  append_seq(log, 80, 50);
  log.flush();
  size_t cur = 0;
  uint8_t tag, len, p[80];
  uint32_t n = 0;
  while (log.read_next(&cur, snap, &tag, p, &len)) ++n;
  TEST_ASSERT_EQUAL_UINT32(80, n);
}

void test_erase_top_down_cancel_keeps_bottom() {
  NorSim f(64);  // 4 blocos de 64 KB
  FlashLog log(f);
  log.mount();
  uint8_t p[80] = {};
  for (int i = 0; i < 40 * 49; ++i) log.append(FlashLog::kTagHistory, p, 80);  // 40 setores
  log.flush();
  TEST_ASSERT_EQUAL_size_t(4, log.erase_blocks());
  TEST_ASSERT_TRUE(log.erase_block_from_top(0));  // setores 48..63: já vazios
  TEST_ASSERT_EQUAL_size_t(0, f.erases_);
  TEST_ASSERT_TRUE(log.erase_block_from_top(1));  // 32..47: apaga 32..39
  TEST_ASSERT_EQUAL_size_t(8, f.erases_);
  // cancel aqui
  TEST_ASSERT_TRUE(log.mount());
  TEST_ASSERT_EQUAL_UINT32(32 * 49, log.history_count());
  TEST_ASSERT_EQUAL_size_t(32 * 4096, log.used_bytes());
  for (int i = 0; i < 10; ++i) log.append(FlashLog::kTagHistory, p, 80);
  log.flush();
  TEST_ASSERT_EQUAL_size_t(0, f.reprograms_);
  // erase completo
  for (size_t b = 0; b < log.erase_blocks(); ++b) TEST_ASSERT_TRUE(log.erase_block_from_top(b));
  TEST_ASSERT_TRUE(log.mount());
  TEST_ASSERT_EQUAL_UINT32(0, log.history_count());
  TEST_ASSERT_EQUAL_UINT8(0, log.memory_pct());
}

void test_bad_records_rejected() {
  NorSim f(4);
  FlashLog log(f);
  log.mount();
  uint8_t p[80] = {};
  TEST_ASSERT_TRUE(log.append(0x22, p, 12) == FlashLog::Append::kBadRecord);
  TEST_ASSERT_TRUE(log.append(FlashLog::kTagHistory, p, 12) == FlashLog::Append::kBadRecord);
  TEST_ASSERT_TRUE(log.append(0xFF, p, 80) == FlashLog::Append::kBadRecord);
  TEST_ASSERT_EQUAL_size_t(0, log.used_bytes());
}

void test_next_wp_matches_append() {
  NorSim f(8);
  FlashLog log(f);
  log.mount();
  size_t shadow = 0;
  uint8_t p[80] = {};
  for (int i = 0; i < 600; ++i) {
    const bool marker = i % 7 == 0;
    const size_t rec = marker ? FlashLog::kStateRecord : FlashLog::kHistoryRecord;
    size_t next;
    const bool fits = FlashLog::next_wp(shadow, rec, f.size(), &next);
    const FlashLog::Append r = log.append(marker ? 0x26 : 0x21, p, marker ? 12 : 80);
    TEST_ASSERT_EQUAL(fits, r == FlashLog::Append::kOk);
    if (fits) shadow = next;
    TEST_ASSERT_EQUAL_size_t(log.used_bytes(), shadow);
  }
}

void test_crc8_known_value() {
  // CRC-8/SMBUS (poly 0x07, init 0): "123456789" → 0xF4
  const uint8_t s[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL_HEX8(0xF4, storage::crc8(s, sizeof s));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_empty_mount);
  RUN_TEST(test_append_flush_remount_read_back);
  RUN_TEST(test_records_never_straddle_sectors);
  RUN_TEST(test_sector_exactly_full_then_continue);
  RUN_TEST(test_full_stops_never_overwrites);
  RUN_TEST(test_power_loss_loses_only_unflushed);
  RUN_TEST(test_torn_write_closes_sector_and_log_continues);
  RUN_TEST(test_read_to_snapshot_excludes_later_records);
  RUN_TEST(test_erase_top_down_cancel_keeps_bottom);
  RUN_TEST(test_bad_records_rejected);
  RUN_TEST(test_next_wp_matches_append);
  RUN_TEST(test_crc8_known_value);
  return UNITY_END();
}
