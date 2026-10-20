// FrameAssembler — mesmo contrato do Racevo (spec 04 §3.3, SPEC §4.4).
// FIFO de bytes de tamanho fixo (1 KB), resync em B5 62, espera por `len`,
// valida Fletcher-8. Frame inválido é descartado (contado). Nunca trava,
// nunca aloca, nunca lança. Serve também para o UBX que vem do receptor GNSS.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "frame.h"

namespace rb {

struct AssemblerStats {
  uint32_t frames = 0;
  uint32_t noise_bytes = 0;    // bytes sem sync descartados
  uint32_t bad_length = 0;     // sync com len > max → resync 1 byte
  uint32_t bad_checksum = 0;   // frame completo com checksum errado → resync 1 byte
  uint32_t overflow_bytes = 0; // descarte por buffer cheio (não deveria ocorrer)
};

class FrameAssembler {
 public:
  static constexpr size_t kCapacity = 1024;

  explicit FrameAssembler(size_t max_payload = kMaxPayload)
      : max_payload_(max_payload < kCapacity - kOverhead ? max_payload
                                                         : kCapacity - kOverhead) {}

  // Alimenta bytes arbitrariamente fragmentados/concatenados. Para cada frame
  // válido chama on_frame(const Frame&). O Frame aponta para o buffer interno
  // e só vale durante a chamada.
  template <typename F>
  void feed(const uint8_t* data, size_t len, F&& on_frame) {
    while (len > 0) {
      compact();
      size_t space = kCapacity - end_;
      if (space == 0) {
        // Impossível com frame máx. 512 B em 1 KB após drain(); defensivo.
        const size_t drop = size() / 2;
        stats_.overflow_bytes += static_cast<uint32_t>(drop);
        start_ += drop;
        compact();
        space = kCapacity - end_;
      }
      const size_t n = len < space ? len : space;
      std::memcpy(buf_ + end_, data, n);
      end_ += n;
      data += n;
      len -= n;
      drain(on_frame);
    }
  }

  void reset() { start_ = end_ = 0; }
  size_t buffered() const { return size(); }
  const AssemblerStats& stats() const { return stats_; }

 private:
  size_t size() const { return end_ - start_; }

  void compact() {
    if (start_ == 0) return;
    const size_t n = size();
    if (n > 0) std::memmove(buf_, buf_ + start_, n);
    start_ = 0;
    end_ = n;
  }

  template <typename F>
  void drain(F& on_frame) {
    for (;;) {
      if (size() < 2) return;
      // Procura o primeiro par de sync.
      size_t i = start_;
      while (i + 1 < end_ && !(buf_[i] == kSync1 && buf_[i + 1] == kSync2)) ++i;
      if (i + 1 >= end_) {
        // Sem sync: descarta tudo, menos um 0xB5 final (meio par).
        const bool keep = buf_[end_ - 1] == kSync1;
        const size_t discard = size() - (keep ? 1 : 0);
        stats_.noise_bytes += static_cast<uint32_t>(discard);
        start_ += discard;
        return;
      }
      if (i > start_) {
        stats_.noise_bytes += static_cast<uint32_t>(i - start_);
        start_ = i;
      }
      if (size() < 6) return;
      const size_t plen = get_u16(buf_ + start_ + 4);
      if (plen > max_payload_) {
        ++stats_.bad_length;
        ++start_;  // falso sync: avança 1 byte e reprocura (não pular 6+N)
        continue;
      }
      const size_t flen = plen + kOverhead;
      if (size() < flen) return;  // espera o resto
      const uint8_t* f = buf_ + start_;
      const Checksum ck = fletcher8(f + 2, 4 + plen);
      if (ck.a != f[6 + plen] || ck.b != f[7 + plen]) {
        ++stats_.bad_checksum;
        ++start_;
        continue;
      }
      ++stats_.frames;
      const Frame frame{f[2], f[3], f + 6, static_cast<uint16_t>(plen)};
      start_ += flen;
      on_frame(frame);
    }
  }

  uint8_t buf_[kCapacity];
  size_t start_ = 0;
  size_t end_ = 0;
  size_t max_payload_;
  AssemblerStats stats_;
};

}  // namespace rb
