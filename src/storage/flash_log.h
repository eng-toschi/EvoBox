// Log append-only direto na partição (SPEC §8.2) — C++ puro sobre uma
// interface de flash NOR (FlashIo), testado em native contra uma flash
// simulada com semântica NOR e queda de energia (test_flash_log).
//
// Formato: registro = tag(1) | len(1) | payload(len) | crc8(1); tags 0x21
// (80 B) e 0x26 (12 B). Registros nunca atravessam setor (4 KB): se não cabe,
// o resto do setor fica apagado (0xFF) e o registro vai para o próximo setor.
// Leitura: tag 0xFF = fim do setor; registro inválido (escrita rasgada por
// queda de energia) fecha o setor. Nunca sobrescreve: cheio = para (vendor).
//
// Escrita: buffer em RAM do setor aberto, programado na flash por partes
// (`flush`) — a política de quando chamar flush é do chamador (D17: ≤ 1 s).
// Boot (`mount`): varre a partição e recupera ponteiro de escrita e contagens
// a partir do próprio conteúdo (sem índice separado que possa divergir).
#pragma once

#include <cstddef>
#include <cstdint>

namespace storage {

class FlashIo {
 public:
  static constexpr size_t kSector = 4096;
  virtual ~FlashIo() = default;
  virtual size_t size() const = 0;  // múltiplo de kSector
  virtual bool read(size_t off, void* buf, size_t len) = 0;
  virtual bool write(size_t off, const void* buf, size_t len) = 0;  // NOR: só 1→0
  virtual bool erase_sector(size_t sector) = 0;
};

uint8_t crc8(const uint8_t* p, size_t n, uint8_t crc = 0);

class FlashLog {
 public:
  static constexpr uint8_t kTagHistory = 0x21;
  static constexpr uint8_t kTagState = 0x26;
  static constexpr size_t kRecordOverhead = 3;
  static constexpr size_t kHistoryRecord = 80 + kRecordOverhead;  // 83
  static constexpr size_t kStateRecord = 12 + kRecordOverhead;    // 15
  static constexpr size_t kEraseBlockSectors = 16;                // 64 KB (vendor)

  enum class Append { kOk, kFull, kIoError, kBadRecord };

  explicit FlashLog(FlashIo& io) : io_(io) {}

  // Regra de posicionamento (registro não atravessa setor): onde fica o
  // ponteiro depois de um registro de `rec` bytes a partir de `wp`. false se
  // não cabe. Usada também para reservar espaço antes do append assíncrono.
  static bool next_wp(size_t wp, size_t rec, size_t size, size_t* out);

  bool mount();
  Append append(uint8_t tag, const uint8_t* payload, uint8_t len);
  bool flush();

  uint32_t history_count() const { return n21_; }
  uint32_t state_count() const { return n26_; }
  uint32_t capacity_history() const;  // registros 0x21 que cabem na partição
  size_t used_bytes() const { return wp_; }
  size_t flushed_end() const { return flushed_; }
  size_t pending_bytes() const { return wp_ - flushed_; }
  bool full() const;                 // não cabe mais nenhum 0x21
  bool can_start() const;            // cabe marcador + ao menos um 0x21
  bool can_append(size_t record_len) const;
  uint8_t memory_pct() const;
  size_t sectors() const { return io_.size() / FlashIo::kSector; }

  // Leitura sequencial de [cursor, end). Retorna false no fim.
  bool read_next(size_t* cursor, size_t end, uint8_t* tag, uint8_t* payload, uint8_t* len);

  // Erase do vendor: de cima para baixo em blocos de 64 KB, pulando blocos já
  // vazios. `block` conta de 0 (topo) a erase_blocks()-1 (início). Depois de
  // terminar ou cancelar, chamar mount() para reconstruir o índice.
  size_t erase_blocks() const;
  bool erase_block_from_top(size_t block);

 private:
  bool sector_empty(size_t sector);

  FlashIo& io_;
  uint32_t n21_ = 0;
  uint32_t n26_ = 0;
  size_t wp_ = 0;       // próximo byte a escrever (lógico)
  size_t flushed_ = 0;  // até onde já está programado na flash (≤ wp_)
  size_t base_ = 0;     // início do setor aberto; buf_[i] ↔ flash[base_ + i]
  uint8_t buf_[FlashIo::kSector];
};

}  // namespace storage
