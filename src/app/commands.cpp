#include "app/commands.h"

namespace cmd {

void Processor::ack(uint8_t id, bool positive) {
  uint8_t b[16];
  const size_t n = rb::encode_ack(positive, rb::kClass, id, b, sizeof b);
  out_.response(b, n);
  if (positive) ++stats_.acks; else ++stats_.nacks;
}

void Processor::reply(uint8_t id, const uint8_t* payload, size_t len) {
  uint8_t b[32];
  const size_t n = rb::encode_frame(rb::kClass, id, payload, len, b, sizeof b);
  out_.response(b, n);
}

void Processor::handle(const rb::Frame& f) {
  if (f.cls != rb::kClass) {
    ++stats_.ignored_foreign_class;
    return;
  }
  ++stats_.handled;
  switch (f.id) {
    case rb::msg::kStatus:
      on_status(f);
      break;
    case rb::msg::kRecConfig:
      on_rec_config(f);
      break;
    case rb::msg::kDownload:
      on_download(f);
      break;
    case rb::msg::kErase:
      on_erase(f);
      break;
    case rb::msg::kGnssConfig:
      on_gnss_config(f);
      break;
    case rb::msg::kUnlock:
      ack(rb::msg::kUnlock, true);  // segurança desabilitada: sempre ACK (SPEC §7.7)
      break;
    default:
      ack(f.id, false);  // id desconhecido → NACK (D6)
      break;
  }
}

void Processor::on_status(const rb::Frame& f) {
  if (f.len != 0) {
    ack(f.id, false);
    return;
  }
  uint8_t p[rb::kStatusLen];
  rb::encode_status(dev_.status(), p);
  reply(rb::msg::kStatus, p, sizeof p);
}

void Processor::on_rec_config(const rb::Frame& f) {
  const rb::RecordingConfig cur = dev_.rec_config();
  if (f.len == 0) {
    rb::RecordingConfig c = cur;
    c.enable = dev_.recording() ? 1 : 0;
    uint8_t p[rb::kRecConfigLen];
    rb::encode_rec_config(c, p);
    reply(rb::msg::kRecConfig, p, sizeof p);
    return;
  }
  rb::RecordingConfig c;
  if (!rb::decode_rec_config(f.payload, f.len, cur, &c)) {
    ack(f.id, false);
    return;
  }
  const bool rec = dev_.recording();
  if (c.enable == 0) {
    // Vendor: com Enable=0 o resto é ignorado.
    if (!rec) {
      ack(f.id, true);
      return;
    }
    if (dev_.download_active() || dev_.erase_active()) {
      ack(f.id, false);
      return;
    }
    dev_.stop_recording();
    ack(f.id, true);
    return;
  }
  c.enable = 1;
  if (!rb::valid_rate(c.rate)) {
    ack(f.id, false);
    return;
  }
  // Idempotência (R11/R13): já gravando com a mesma config → ACK e nada mais,
  // inclusive durante um download (não muda nada).
  if (rec && c.same_settings(cur)) {
    ++stats_.idempotent_25;
    ack(f.id, true);
    return;
  }
  if (dev_.download_active() || dev_.erase_active()) {
    ack(f.id, false);
    return;
  }
  if (rec) {
    dev_.reconfigure_recording(c);
    ack(f.id, true);
    return;
  }
  if (!dev_.can_start_recording()) {
    ack(f.id, false);  // memória cheia (o Racevo tolera: o chip de backup não aparece)
    return;
  }
  dev_.start_recording(c);
  ack(f.id, true);
}

void Processor::on_download(const rb::Frame& f) {
  if (f.len == 1) {  // cancel
    if (dev_.download_active()) dev_.cancel_download();
    else ack(f.id, true);
    return;
  }
  if (f.len != 0 || dev_.erase_active() || dev_.download_active()) {
    ack(f.id, false);
    return;
  }
  uint32_t expected = 0;
  if (!dev_.prepare_download(&expected)) {
    ack(f.id, false);
    return;
  }
  uint8_t p[4];
  rb::put_u32(p, expected);
  uint8_t b[16];
  const size_t n = rb::encode_frame(rb::kClass, rb::msg::kDownload, p, 4, b, sizeof b);
  out_.bulk(b, n);  // no mesmo stream dos registros: sai antes deles
  dev_.begin_download();
}

void Processor::on_erase(const rb::Frame& f) {
  if (f.len == 1) {  // cancel
    if (dev_.erase_active()) dev_.cancel_erase();
    else ack(f.id, true);
    return;
  }
  if (f.len != 0 || dev_.recording() || dev_.download_active() || dev_.erase_active() ||
      !dev_.start_erase()) {
    ack(f.id, false);
  }
}

void Processor::on_gnss_config(const rb::Frame& f) {
  if (f.len == 0) {
    uint8_t p[rb::kGnssConfigLen];
    rb::encode_gnss_config(dev_.gnss_config(), p);
    reply(rb::msg::kGnssConfig, p, sizeof p);
    return;
  }
  if (f.len != rb::kGnssConfigLen) {
    ack(f.id, false);
    return;
  }
  const rb::GnssConfig c = rb::decode_gnss_config(f.payload);
  if (!rb::valid_gnss_config(c)) {
    ack(f.id, false);
    return;
  }
  dev_.set_gnss_config(c);
  ack(f.id, true);
}

}  // namespace cmd
