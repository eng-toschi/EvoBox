// FrameAssembler: re-chunking, concatenação, gotejamento, ruído, B5 62 dentro
// do payload, corrupção, overflow — espelha Racevo spec 04 §9.3 (SPEC §10.1).
#include <unity.h>

#include <cstring>
#include <vector>

#include "assembler.h"
#include "frame.h"

using Bytes = std::vector<uint8_t>;

void setUp() {}
void tearDown() {}

struct Rng {  // xorshift32, seed fixa → determinístico
  uint32_t s;
  explicit Rng(uint32_t seed) : s(seed) {}
  uint32_t next() {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
  }
  uint32_t below(uint32_t n) { return next() % n; }
};

static Bytes make_frame(uint8_t id, const Bytes& payload) {
  Bytes f(payload.size() + rb::kOverhead);
  f.resize(rb::encode_frame(rb::kClass, id, payload.data(), payload.size(), f.data(), f.size()));
  return f;
}

struct Got {
  uint8_t id;
  Bytes payload;
};

struct Collector {
  std::vector<Got> got;
  void operator()(const rb::Frame& f) {
    got.push_back({f.id, Bytes(f.payload, f.payload + f.len)});
  }
};

// N frames variados: tamanhos 0, 1, 2, 12, 80, 504, conteúdo que inclui B5 62.
static std::vector<Bytes> sample_frames(size_t n, Rng& rng, std::vector<Got>* expect) {
  static const size_t sizes[] = {0, 1, 2, 3, 4, 12, 80, 255, 504};
  std::vector<Bytes> frames;
  for (size_t i = 0; i < n; ++i) {
    const size_t len = sizes[rng.below(sizeof sizes / sizeof sizes[0])];
    Bytes p(len);
    for (size_t k = 0; k < len; ++k) p[k] = static_cast<uint8_t>(rng.next());
    if (len >= 4) {  // sync falso dentro do payload
      p[len / 2] = rb::kSync1;
      p[len / 2 + 1] = rb::kSync2;
    }
    const uint8_t id = static_cast<uint8_t>(0x01 + i % 0x30);
    frames.push_back(make_frame(id, p));
    expect->push_back({id, p});
  }
  return frames;
}

static void assert_same(const std::vector<Got>& expect, const std::vector<Got>& got) {
  TEST_ASSERT_EQUAL_size_t(expect.size(), got.size());
  for (size_t i = 0; i < expect.size(); ++i) {
    TEST_ASSERT_EQUAL_HEX8(expect[i].id, got[i].id);
    TEST_ASSERT_EQUAL_size_t(expect[i].payload.size(), got[i].payload.size());
    if (!expect[i].payload.empty())
      TEST_ASSERT_EQUAL_HEX8_ARRAY(expect[i].payload.data(), got[i].payload.data(),
                                   expect[i].payload.size());
  }
}

static Bytes concat(const std::vector<Bytes>& frames) {
  Bytes s;
  for (const Bytes& f : frames) s.insert(s.end(), f.begin(), f.end());
  return s;
}

void test_single_frame_whole() {
  Rng rng(1);
  std::vector<Got> expect;
  const Bytes s = concat(sample_frames(1, rng, &expect));
  rb::FrameAssembler a;
  Collector c;
  a.feed(s.data(), s.size(), c);
  assert_same(expect, c.got);
  TEST_ASSERT_EQUAL_size_t(0, a.buffered());
}

// Todos os pontos de corte (um e dois cortes) num stream pequeno.
void test_every_split_point_small_stream() {
  const Bytes p1 = {0x01, 0xB5, 0x62, 0x04};
  const Bytes p2 = {};
  const Bytes p3 = {0xB5, 0x62, 0xFF, 0x01, 0x02, 0x00};
  const std::vector<Bytes> frames = {make_frame(0x22, p1), make_frame(0x23, p2),
                                     make_frame(0x24, p3)};
  const std::vector<Got> expect = {{0x22, p1}, {0x23, p2}, {0x24, p3}};
  const Bytes s = concat(frames);
  for (size_t i = 0; i <= s.size(); ++i) {
    for (size_t j = i; j <= s.size(); ++j) {
      rb::FrameAssembler a;
      Collector c;
      a.feed(s.data(), i, c);
      a.feed(s.data() + i, j - i, c);
      a.feed(s.data() + j, s.size() - j, c);
      assert_same(expect, c.got);
      TEST_ASSERT_EQUAL_size_t(0, a.buffered());
    }
  }
}

void test_random_rechunking_fixed_seed() {
  Rng rng(0xC0FFEE);
  for (int round = 0; round < 200; ++round) {
    std::vector<Got> expect;
    const Bytes s = concat(sample_frames(1 + rng.below(20), rng, &expect));
    rb::FrameAssembler a;
    Collector c;
    size_t off = 0;
    while (off < s.size()) {
      const size_t n = 1 + rng.below(300);
      const size_t take = n < s.size() - off ? n : s.size() - off;
      a.feed(s.data() + off, take, c);
      off += take;
    }
    assert_same(expect, c.got);
    TEST_ASSERT_EQUAL_UINT32(0, a.stats().bad_checksum);
    TEST_ASSERT_EQUAL_UINT32(0, a.stats().noise_bytes);
  }
}

void test_concatenation_many_frames_one_feed() {
  Rng rng(7);
  std::vector<Got> expect;
  const Bytes s = concat(sample_frames(60, rng, &expect));
  rb::FrameAssembler a;
  Collector c;
  a.feed(s.data(), s.size(), c);  // maior que o buffer de 1 KB: feed fatia internamente
  assert_same(expect, c.got);
}

void test_drip_one_byte_at_a_time() {
  Rng rng(9);
  std::vector<Got> expect;
  const Bytes s = concat(sample_frames(15, rng, &expect));
  rb::FrameAssembler a;
  Collector c;
  for (uint8_t b : s) a.feed(&b, 1, c);
  assert_same(expect, c.got);
}

void test_noise_before_between_after() {
  Rng rng(11);
  std::vector<Got> expect;
  const std::vector<Bytes> frames = sample_frames(10, rng, &expect);
  Bytes s;
  for (const Bytes& f : frames) {
    for (int k = 0; k < 17; ++k) {
      uint8_t junk = static_cast<uint8_t>(rng.next());
      if (junk == rb::kSync1) junk = 0x00;  // ruído sem meio-sync (testado à parte)
      s.push_back(junk);
    }
    s.insert(s.end(), f.begin(), f.end());
  }
  for (int k = 0; k < 9; ++k) s.push_back(0x55);
  rb::FrameAssembler a;
  Collector c;
  a.feed(s.data(), s.size(), c);
  assert_same(expect, c.got);
  TEST_ASSERT_TRUE(a.stats().noise_bytes >= 17u * 10u);
}

// Ruído adversarial: sync falsos com len plausível e checksum errado no meio.
void test_false_syncs_in_noise_do_not_eat_next_frame() {
  const Bytes p = {0x10, 0x20, 0x30};
  const Bytes good = make_frame(0x26, p);
  Bytes s = {0xB5, 0x62, 0xFF, 0x01, 0x05, 0x00, 0xB5};  // header falso, len=5
  s.insert(s.end(), good.begin(), good.end());
  rb::FrameAssembler a;
  Collector c;
  a.feed(s.data(), s.size(), c);
  // O header falso pede 5+8 B; até completar, o frame bom fica preso dentro dele.
  // O checksum falha, o assembler avança 1 byte e reencontra o frame bom.
  const std::vector<Got> expect = {{0x26, p}};
  assert_same(expect, c.got);
  TEST_ASSERT_TRUE(a.stats().bad_checksum >= 1);
}

void test_sync_inside_payload_does_not_desync() {
  // Payload que contém um frame UBX completo e válido inteiro.
  const Bytes inner = make_frame(0x02, {0xFF, 0x30});
  const Bytes outer = make_frame(0x21, inner);
  rb::FrameAssembler a;
  Collector c;
  a.feed(outer.data(), outer.size(), c);
  const std::vector<Got> expect = {{0x21, inner}};
  assert_same(expect, c.got);
}

void test_corruption_drops_frame_and_next_parses() {
  const Bytes p1(80, 0x11), p2(12, 0x22);
  Bytes f1 = make_frame(0x01, p1);
  const Bytes f2 = make_frame(0x26, p2);
  for (size_t pos = 2; pos < f1.size(); ++pos) {
    Bytes bad = f1;
    bad[pos] ^= 0x5A;
    Bytes s = bad;
    s.insert(s.end(), f2.begin(), f2.end());
    rb::FrameAssembler a;
    Collector c;
    a.feed(s.data(), s.size(), c);
    // Se o len foi corrompido para algo grande, o assembler espera bytes que
    // nunca virão — o frame bom só sai quando o stream continua. Empurra mais
    // frames bons para provar que nunca trava.
    for (int k = 0; k < 8 && c.got.empty(); ++k) a.feed(f2.data(), f2.size(), c);
    TEST_ASSERT_FALSE(c.got.empty());
    for (const Got& g : c.got) {
      TEST_ASSERT_EQUAL_HEX8(0x26, g.id);
      TEST_ASSERT_EQUAL_HEX8_ARRAY(p2.data(), g.payload.data(), p2.size());
    }
  }
}

void test_length_over_max_resyncs() {
  Bytes s = {0xB5, 0x62, 0xFF, 0x01, 0xF9, 0x01};  // len = 505 > 504
  const Bytes p = {0x42};
  const Bytes good = make_frame(0x22, p);
  s.insert(s.end(), good.begin(), good.end());
  rb::FrameAssembler a;
  Collector c;
  a.feed(s.data(), s.size(), c);
  const std::vector<Got> expect = {{0x22, p}};
  assert_same(expect, c.got);
  TEST_ASSERT_EQUAL_UINT32(1, a.stats().bad_length);
}

void test_max_payload_504_accepted() {
  const Bytes p(504, 0xA5);
  const Bytes f = make_frame(0x21, p);
  TEST_ASSERT_EQUAL_size_t(rb::kMaxFrame, f.size());
  rb::FrameAssembler a;
  Collector c;
  a.feed(f.data(), f.size(), c);
  TEST_ASSERT_EQUAL_size_t(1, c.got.size());
}

void test_custom_max_payload_for_gnss() {
  // O assembler também serve para o UBX do receptor (NAV-PVT = 92 B).
  rb::FrameAssembler a(100);
  const Bytes big(101, 0), ok(92, 0);
  Bytes s = make_frame(0x07, big);
  const Bytes f = make_frame(0x07, ok);
  s.insert(s.end(), f.begin(), f.end());
  Collector c;
  a.feed(s.data(), s.size(), c);
  TEST_ASSERT_EQUAL_size_t(1, c.got.size());
  TEST_ASSERT_EQUAL_size_t(92, c.got[0].payload.size());
}

// Parcial que nunca completa + lixo infinito: buffer limitado, nunca trava,
// e o stream volta a fluir.
void test_never_completing_partial_is_bounded() {
  rb::FrameAssembler a;
  Collector c;
  const Bytes hdr = {0xB5, 0x62, 0xFF, 0x21, 0xF8, 0x01};  // len 504, nunca completa
  a.feed(hdr.data(), hdr.size(), c);
  Rng rng(3);
  for (int i = 0; i < 20000; ++i) {
    uint8_t b = static_cast<uint8_t>(rng.next());
    a.feed(&b, 1, c);
    TEST_ASSERT_TRUE(a.buffered() <= rb::FrameAssembler::kCapacity);
  }
  const Bytes p = {1, 2, 3};
  const Bytes good = make_frame(0x22, p);
  for (int k = 0; k < 4; ++k) a.feed(good.data(), good.size(), c);
  TEST_ASSERT_FALSE(c.got.empty());
  TEST_ASSERT_EQUAL_HEX8(0x22, c.got.back().id);
}

// Lixo adversarial puro (só B5/62 e lens grandes): nada sai, nada trava.
void test_adversarial_garbage_never_hangs() {
  rb::FrameAssembler a;
  Collector c;
  Rng rng(0xBADF00D);
  const uint8_t alphabet[] = {0xB5, 0x62, 0xFF, 0x01, 0xF8, 0x01, 0x00};
  for (int i = 0; i < 100000; ++i) {
    uint8_t chunk[37];
    const size_t n = 1 + rng.below(sizeof chunk);
    for (size_t k = 0; k < n; ++k) chunk[k] = alphabet[rng.below(sizeof alphabet)];
    a.feed(chunk, n, c);
    TEST_ASSERT_TRUE(a.buffered() <= rb::FrameAssembler::kCapacity);
  }
  for (const Got& g : c.got) {  // qualquer frame que saia tem checksum válido
    Bytes f = make_frame(g.id, g.payload);
    TEST_ASSERT_TRUE(rb::validate_frame(f.data(), f.size()));
  }
}

void test_half_sync_at_end_is_kept() {
  const Bytes p = {9};
  const Bytes f = make_frame(0x22, p);
  rb::FrameAssembler a;
  Collector c;
  const uint8_t junk[] = {0x00, 0x11, 0xB5};
  a.feed(junk, 2, c);
  a.feed(f.data(), 1, c);  // só o 0xB5
  TEST_ASSERT_EQUAL_size_t(1, a.buffered());
  a.feed(f.data() + 1, f.size() - 1, c);
  TEST_ASSERT_EQUAL_size_t(1, c.got.size());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_single_frame_whole);
  RUN_TEST(test_every_split_point_small_stream);
  RUN_TEST(test_random_rechunking_fixed_seed);
  RUN_TEST(test_concatenation_many_frames_one_feed);
  RUN_TEST(test_drip_one_byte_at_a_time);
  RUN_TEST(test_noise_before_between_after);
  RUN_TEST(test_false_syncs_in_noise_do_not_eat_next_frame);
  RUN_TEST(test_sync_inside_payload_does_not_desync);
  RUN_TEST(test_corruption_drops_frame_and_next_parses);
  RUN_TEST(test_length_over_max_resyncs);
  RUN_TEST(test_max_payload_504_accepted);
  RUN_TEST(test_custom_max_payload_for_gnss);
  RUN_TEST(test_never_completing_partial_is_bounded);
  RUN_TEST(test_adversarial_garbage_never_hangs);
  RUN_TEST(test_half_sync_at_end_is_kept);
  return UNITY_END();
}
