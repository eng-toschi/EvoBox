// Fila TX com prioridade (SPEC §4.3): respostas > download (bulk) > live.
// - live: fila curta; cheia → descarta o MAIS ANTIGO e conta (nunca bloqueia o builder);
// - bulk: stream de bytes do download (0x23 + 0x21/0x26 + ACK final, em ordem);
//   o produtor espera (backpressure) — nunca descarta registro de histórico;
// - respostas: frames curtos de comando.
#pragma once

#include <Arduino.h>

#include <cstddef>
#include <cstdint>

namespace txq {

constexpr size_t kMaxResponse = 32;
constexpr size_t kLiveFrame = 88;

struct Stats {
  uint32_t live_queued = 0;
  uint32_t live_dropped = 0;  // descartados por fila cheia (o mais antigo sai)
  uint32_t live_muted = 0;    // descartados por download/erase em andamento
  uint32_t resp_dropped = 0;
};

void init();

// Produtores (qualquer task).
bool push_response(const uint8_t* frame, size_t len);
void push_live(const uint8_t frame[kLiveFrame]);
// Escreve no stream de download; bloqueia até `wait`. Retorna bytes aceitos.
size_t push_bulk(const uint8_t* data, size_t len, TickType_t wait);
size_t bulk_pending();

void set_live_muted(bool muted);
bool live_muted();
void clear();  // desconexão: descarta tudo o que estava pendente

// Consumidor (task BLE): próximo bloco a notificar, até `max_len` bytes.
// Respostas/live saem um frame por vez (fragmentados pelo chamador se
// maiores que max_len); bulk sai empacotado até max_len.
// Retorna 0 se nada pendente. *is_live indica a origem.
size_t next(uint8_t* out, size_t max_len, bool* is_live);

Stats stats();
void reset_stats();

// A task BLE espera nesta notificação (produtores acordam a task).
void set_consumer(TaskHandle_t t);

}  // namespace txq
