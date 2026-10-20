#include "ble/tx_queue.h"

#include <freertos/queue.h>
#include <freertos/stream_buffer.h>

#include <cstring>

namespace txq {
namespace {

struct Resp {
  uint8_t len;
  uint8_t data[kMaxResponse];
};
struct Live {
  uint8_t data[kLiveFrame];
};

QueueHandle_t g_resp = nullptr;
QueueHandle_t g_live = nullptr;
StreamBufferHandle_t g_bulk = nullptr;
volatile bool g_muted = false;
TaskHandle_t g_consumer = nullptr;
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
Stats g_stats;

void wake() {
  if (g_consumer) xTaskNotifyGive(g_consumer);
}

}  // namespace

void init() {
  g_resp = xQueueCreate(16, sizeof(Resp));
  g_live = xQueueCreate(4, sizeof(Live));
  g_bulk = xStreamBufferCreate(4096, 1);
}

bool push_response(const uint8_t* frame, size_t len) {
  if (len > kMaxResponse) return false;
  Resp r;
  r.len = static_cast<uint8_t>(len);
  memcpy(r.data, frame, len);
  const bool ok = xQueueSend(g_resp, &r, 0) == pdTRUE;
  if (!ok) {
    portENTER_CRITICAL(&g_mux);
    ++g_stats.resp_dropped;
    portEXIT_CRITICAL(&g_mux);
  }
  wake();
  return ok;
}

void push_live(const uint8_t frame[kLiveFrame]) {
  if (g_muted) {
    portENTER_CRITICAL(&g_mux);
    ++g_stats.live_muted;
    portEXIT_CRITICAL(&g_mux);
    return;
  }
  Live l;
  memcpy(l.data, frame, kLiveFrame);
  if (xQueueSend(g_live, &l, 0) != pdTRUE) {
    Live old;
    xQueueReceive(g_live, &old, 0);  // descarta o mais antigo
    xQueueSend(g_live, &l, 0);
    portENTER_CRITICAL(&g_mux);
    ++g_stats.live_dropped;
    portEXIT_CRITICAL(&g_mux);
  }
  portENTER_CRITICAL(&g_mux);
  ++g_stats.live_queued;
  portEXIT_CRITICAL(&g_mux);
  wake();
}

size_t push_bulk(const uint8_t* data, size_t len, TickType_t wait) {
  size_t sent = 0;
  const TickType_t t0 = xTaskGetTickCount();
  while (sent < len) {
    const TickType_t elapsed = xTaskGetTickCount() - t0;
    if (elapsed >= wait && sent > 0) break;
    const size_t n = xStreamBufferSend(g_bulk, data + sent, len - sent,
                                       elapsed >= wait ? 0 : wait - elapsed);
    sent += n;
    wake();
    if (n == 0 && elapsed >= wait) break;
  }
  return sent;
}

size_t bulk_pending() { return xStreamBufferBytesAvailable(g_bulk); }

void set_live_muted(bool muted) {
  g_muted = muted;
  if (muted) xQueueReset(g_live);
}

bool live_muted() { return g_muted; }

void clear() {
  xQueueReset(g_resp);
  xQueueReset(g_live);
  xStreamBufferReset(g_bulk);
}

size_t next(uint8_t* out, size_t max_len, bool* is_live) {
  *is_live = false;
  Resp r;
  if (xQueueReceive(g_resp, &r, 0) == pdTRUE) {
    memcpy(out, r.data, r.len);
    return r.len;
  }
  if (xStreamBufferBytesAvailable(g_bulk) > 0) {
    return xStreamBufferReceive(g_bulk, out, max_len, 0);
  }
  Live l;
  if (!g_muted && xQueueReceive(g_live, &l, 0) == pdTRUE) {
    *is_live = true;
    memcpy(out, l.data, kLiveFrame);
    return kLiveFrame;
  }
  return 0;
}

Stats stats() {
  portENTER_CRITICAL(&g_mux);
  Stats s = g_stats;
  portEXIT_CRITICAL(&g_mux);
  return s;
}

void reset_stats() {
  portENTER_CRITICAL(&g_mux);
  g_stats = Stats{};
  portEXIT_CRITICAL(&g_mux);
}

void set_consumer(TaskHandle_t t) { g_consumer = t; }

}  // namespace txq
