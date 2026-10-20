// iTOW monotônico desde o primeiro frame emitido (SPEC §5.2, R14; CLAUDE.md regra 5).
//
// - Primeiro NAV-PVT com validTime = 1 (hot start, caso normal): iTOW sai cru.
// - Primeiro NAV-PVT com validTime = 0: o iTOW do receptor é substituído por um
//   contador local ancorado no relógio local (ms). Quando a hora resolve, o
//   deslocamento `contador − iTOW real` é fixado e aplicado até o próximo boot:
//   o fio continua monotônico e o salto vira só um evento contado.
// - Qualquer passo para trás que não seja virada de semana (salto > meia semana)
//   é absorvido re-ancorando o deslocamento (contado em `backstep_fixes`).
// Toda a aritmética é módulo uma semana GPS (o Racevo trata salto para trás
// > meia semana como virada de semana).
#pragma once

#include <cstdint>

namespace rb {

constexpr uint32_t kMsPerWeek = 604800000u;

class ItowMonotonic {
 public:
  enum class Mode : uint8_t { kUnset, kRaw, kLocalCounter, kOffset };

  // pvt_itow: iTOW do receptor; valid_time: NAV-PVT valid.b1; now_ms: relógio
  // local monotônico (ms) no instante de chegada.
  uint32_t process(uint32_t pvt_itow, bool valid_time, uint32_t now_ms) {
    uint32_t out;
    switch (mode_) {
      case Mode::kUnset:
        if (valid_time) {
          mode_ = Mode::kRaw;
          offset_ = 0;
          out = wrap(pvt_itow);
        } else {
          mode_ = Mode::kLocalCounter;
          anchor_itow_ = wrap(pvt_itow);
          anchor_ms_ = now_ms;
          out = anchor_itow_;
        }
        last_ = out;
        return out;
      case Mode::kLocalCounter:
        out = add(anchor_itow_, now_ms - anchor_ms_);
        if (!ahead(out, last_)) out = add(last_, 1);
        if (valid_time) {
          // Hora resolvida: fixa o deslocamento para que o fio siga deste ponto.
          offset_ = sub(out, wrap(pvt_itow));
          mode_ = Mode::kOffset;
          ++time_resolved_events_;
        }
        last_ = out;
        return out;
      case Mode::kRaw:
      case Mode::kOffset:
      default:
        out = add(wrap(pvt_itow), offset_);
        if (!ahead(out, last_)) {
          // Passo para trás (ou repetido) que não é virada de semana: re-ancora.
          const uint32_t fixed = add(last_, 1);
          offset_ = sub(fixed, wrap(pvt_itow));
          out = fixed;
          mode_ = Mode::kOffset;
          ++backstep_fixes_;
        }
        last_ = out;
        return out;
    }
  }

  Mode mode() const { return mode_; }
  uint32_t offset() const { return offset_; }
  uint32_t time_resolved_events() const { return time_resolved_events_; }
  uint32_t backstep_fixes() const { return backstep_fixes_; }

 private:
  static uint32_t wrap(uint32_t v) { return v % kMsPerWeek; }
  static uint32_t add(uint32_t a, uint32_t b) {
    return static_cast<uint32_t>((static_cast<uint64_t>(a) + b) % kMsPerWeek);
  }
  static uint32_t sub(uint32_t a, uint32_t b) {
    return static_cast<uint32_t>((static_cast<uint64_t>(a) + kMsPerWeek - b) % kMsPerWeek);
  }
  // `a` está estritamente à frente de `b` (módulo semana, janela de meia semana).
  static bool ahead(uint32_t a, uint32_t b) {
    const uint32_t d = sub(a, b);
    return d != 0 && d < kMsPerWeek / 2;
  }

  Mode mode_ = Mode::kUnset;
  uint32_t anchor_itow_ = 0;
  uint32_t anchor_ms_ = 0;
  uint32_t offset_ = 0;
  uint32_t last_ = 0;
  uint32_t time_resolved_events_ = 0;
  uint32_t backstep_fixes_ = 0;
};

}  // namespace rb
