// Task storage (SPEC §8, §9.1): dona do FlashLog na partição `storage`.
// Append via fila (timeout 0 + descarte contado), flush ≤ 1 s (D17),
// download com snapshot do ponteiro de escrita (R12) e backpressure,
// erase de cima para baixo com progresso e cancel (§7.5).
#pragma once

#include <Arduino.h>

#include <cstddef>
#include <cstdint>

namespace storage {

struct Info {
  bool mounted = false;
  uint32_t history = 0;    // registros 0x21
  uint32_t states = 0;     // marcadores 0x26
  uint32_t capacity = 0;   // registros 0x21 que cabem
  uint8_t memory_pct = 0;
  size_t used_bytes = 0;
  size_t size_bytes = 0;
  bool full = false;
  bool download_active = false;
  bool erase_active = false;
  uint32_t log_q_dropped = 0;
  uint32_t flushes = 0;
  uint32_t flush_ms_max = 0;
  uint32_t mount_ms = 0;
  uint32_t last_download_frames = 0;
};

bool start();
Info info();

// Reserva espaço para um registro (regra do FlashLog) e o enfileira.
// false = não cabe (memória cheia) ou fila cheia (contado).
bool append(uint8_t tag, const uint8_t* payload, uint8_t len);
bool can_start();  // cabe marcador + 1 registro (pelo ponteiro reservado)

// Download (chamadas da task BLE).
bool prepare_download(uint32_t* expected);  // flush + snapshot; silencia o live
void begin_download();
void cancel_download();
// Erase.
bool start_erase();
void cancel_erase();
// Flush síncrono (auto-shutdown, console). Espera até `timeout_ms`.
bool flush_now(uint32_t timeout_ms);

}  // namespace storage
