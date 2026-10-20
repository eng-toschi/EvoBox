#include "storage/flash_log.h"

#include <cstring>

namespace storage {
namespace {

constexpr size_t kSector = FlashIo::kSector;

bool valid_header(uint8_t tag, uint8_t len) {
  return (tag == FlashLog::kTagHistory && len == 80) || (tag == FlashLog::kTagState && len == 12);
}

}  // namespace

uint8_t crc8(const uint8_t* p, size_t n, uint8_t crc) {
  for (size_t i = 0; i < n; ++i) {
    crc ^= p[i];
    for (int b = 0; b < 8; ++b) crc = static_cast<uint8_t>((crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1);
  }
  return crc;
}

bool FlashLog::mount() {
  n21_ = n26_ = 0;
  wp_ = 0;
  const size_t n = sectors();
  for (size_t s = 0; s < n; ++s) {
    const size_t base = s * kSector;
    if (!io_.read(base, buf_, kSector)) return false;
    if (buf_[0] == 0xFF) {  // primeiro setor vazio = fim do log
      wp_ = base;
      break;
    }
    size_t o = 0;
    bool closed = false;
    while (o + kRecordOverhead <= kSector) {
      const uint8_t tag = buf_[o], len = buf_[o + 1];
      if (tag == 0xFF) break;  // resto do setor apagado
      if (!valid_header(tag, len) || o + len + kRecordOverhead > kSector ||
          crc8(buf_ + o, 2u + len) != buf_[o + 2 + len]) {
        closed = true;  // escrita rasgada: o setor não recebe mais nada
        break;
      }
      if (tag == kTagHistory) ++n21_; else ++n26_;
      o += len + kRecordOverhead;
    }
    wp_ = (closed || o + kRecordOverhead > kSector) ? base + kSector : base + o;
  }
  flushed_ = wp_;
  base_ = wp_ - wp_ % kSector;
  return true;
}

bool FlashLog::next_wp(size_t wp, size_t rec, size_t size, size_t* out) {
  if (wp >= size) return false;
  const size_t sector_end = (wp / kSector + 1) * kSector;
  const size_t start = wp + rec <= sector_end ? wp : sector_end;
  if (start + rec > size) return false;
  *out = start + rec;
  return true;
}

bool FlashLog::can_append(size_t rec) const {
  size_t out;
  return next_wp(wp_, rec, io_.size(), &out);
}

bool FlashLog::full() const { return !can_append(kHistoryRecord); }

bool FlashLog::can_start() const { return can_append(kStateRecord + kHistoryRecord); }

uint32_t FlashLog::capacity_history() const {
  return static_cast<uint32_t>(sectors() * (kSector / kHistoryRecord));
}

uint8_t FlashLog::memory_pct() const {
  if (full()) return 100;
  if (wp_ == 0) return 0;
  const size_t pct = wp_ * 100 / io_.size();
  return static_cast<uint8_t>(pct == 0 ? 1 : (pct >= 100 ? 99 : pct));
}

FlashLog::Append FlashLog::append(uint8_t tag, const uint8_t* payload, uint8_t len) {
  if (!valid_header(tag, len)) return Append::kBadRecord;
  const size_t rec = len + kRecordOverhead;
  if (!can_append(rec)) return Append::kFull;
  const size_t sector_end = (wp_ / kSector + 1) * kSector;
  if (wp_ + rec > sector_end) {
    if (!flush()) return Append::kIoError;
    wp_ = flushed_ = base_ = sector_end;  // resto do setor fica apagado
  }
  if (wp_ % kSector == 0 && base_ != wp_) {
    // setor anterior terminou exatamente cheio: programa o que faltava dele
    if (!flush()) return Append::kIoError;
    base_ = wp_;
  }
  uint8_t* p = buf_ + (wp_ - base_);
  p[0] = tag;
  p[1] = len;
  std::memcpy(p + 2, payload, len);
  p[2 + len] = crc8(p, 2u + len);
  wp_ += rec;
  if (tag == kTagHistory) ++n21_; else ++n26_;
  return Append::kOk;
}

bool FlashLog::flush() {
  if (flushed_ >= wp_) return true;
  if (!io_.write(flushed_, buf_ + (flushed_ - base_), wp_ - flushed_)) return false;
  flushed_ = wp_;
  return true;
}

bool FlashLog::read_next(size_t* cursor, size_t end, uint8_t* tag, uint8_t* payload, uint8_t* len) {
  while (*cursor < end) {
    const size_t sector_end = (*cursor / kSector + 1) * kSector;
    if (*cursor + kRecordOverhead > sector_end) {
      *cursor = sector_end;
      continue;
    }
    uint8_t hdr[2];
    if (!io_.read(*cursor, hdr, 2)) return false;
    if (hdr[0] == 0xFF || !valid_header(hdr[0], hdr[1]) ||
        *cursor + hdr[1] + kRecordOverhead > sector_end) {
      *cursor = sector_end;
      continue;
    }
    uint8_t rec[2 + 80 + 1];
    if (!io_.read(*cursor, rec, hdr[1] + kRecordOverhead)) return false;
    if (crc8(rec, 2u + hdr[1]) != rec[2 + hdr[1]]) {
      *cursor = sector_end;  // registro rasgado fecha o setor
      continue;
    }
    if (*cursor + hdr[1] + kRecordOverhead > end) return false;
    *tag = hdr[0];
    *len = hdr[1];
    std::memcpy(payload, rec + 2, hdr[1]);
    *cursor += hdr[1] + kRecordOverhead;
    return true;
  }
  return false;
}

size_t FlashLog::erase_blocks() const { return (sectors() + kEraseBlockSectors - 1) / kEraseBlockSectors; }

bool FlashLog::sector_empty(size_t sector) {
  uint8_t chunk[256];
  for (size_t o = 0; o < kSector; o += sizeof chunk) {
    if (!io_.read(sector * kSector + o, chunk, sizeof chunk)) return false;
    for (uint8_t b : chunk)
      if (b != 0xFF) return false;
  }
  return true;
}

bool FlashLog::erase_block_from_top(size_t block) {
  const size_t nblocks = erase_blocks();
  if (block >= nblocks) return false;
  const size_t first = (nblocks - 1 - block) * kEraseBlockSectors;
  size_t last = first + kEraseBlockSectors;
  if (last > sectors()) last = sectors();
  for (size_t s = last; s-- > first;) {
    if (!sector_empty(s) && !io_.erase_sector(s)) return false;
  }
  return true;
}

}  // namespace storage
