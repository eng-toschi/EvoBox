#include <Arduino.h>
#include <esp_partition.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "app/log.h"
#include "ble/ble_server.h"
#include "ble/tx_queue.h"
#include "frame.h"
#include "messages.h"
#include "payload.h"
#include "storage/flash_log.h"
#include "storage/storage.h"

namespace storage {
namespace {

constexpr size_t kFlushBytes = 1024;      // D17: flush a cada ~1 KB ...
constexpr uint32_t kFlushMs = 1000;       // ... ou 1 s, o que vier antes (perda ≤ 1 s)
constexpr size_t kDownloadBatch = 32;     // frames por volta do loop (appends seguem atendidos)

class PartitionFlash : public FlashIo {
 public:
  explicit PartitionFlash(const esp_partition_t* p) : p_(p) {}
  size_t size() const override { return p_->size - p_->size % kSector; }
  bool read(size_t off, void* buf, size_t len) override {
    return esp_partition_read(p_, off, buf, len) == ESP_OK;
  }
  bool write(size_t off, const void* buf, size_t len) override {
    return esp_partition_write(p_, off, buf, len) == ESP_OK;
  }
  bool erase_sector(size_t s) override {
    return esp_partition_erase_range(p_, s * kSector, kSector) == ESP_OK;
  }

 private:
  const esp_partition_t* p_;
};

struct Item {
  uint8_t tag;
  uint8_t len;
  uint8_t p[80];
};

PartitionFlash* g_io = nullptr;
FlashLog* g_log = nullptr;
QueueHandle_t g_q = nullptr;
TaskHandle_t g_task = nullptr;
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
Info g_info;
size_t g_shadow = 0;  // ponteiro de escrita reservado (inclui o que está na fila)

enum class Req : uint8_t { kNone, kPrepare, kFlush };
volatile Req g_req = Req::kNone;
SemaphoreHandle_t g_req_done = nullptr;
uint32_t g_req_expected = 0;
bool g_req_ok = false;

volatile bool g_begin = false;
volatile bool g_cancel_dl = false;
volatile bool g_erase_req = false;
volatile bool g_cancel_erase = false;

// Download.
bool g_dl = false;
size_t g_dl_cursor = 0, g_dl_end = 0;
uint8_t g_dl_frame[rb::kLivePayloadLen + rb::kOverhead];
size_t g_dl_len = 0, g_dl_off = 0;
uint32_t g_dl_frames = 0;
uint32_t g_first_pending_ms = 0;

void publish() {
  Info i;
  i.mounted = true;
  i.history = g_log->history_count();
  i.states = g_log->state_count();
  i.capacity = g_log->capacity_history();
  i.memory_pct = g_log->memory_pct();
  i.used_bytes = g_log->used_bytes();
  i.size_bytes = g_io->size();
  i.full = g_log->full();
  portENTER_CRITICAL(&g_mux);
  i.download_active = g_info.download_active;
  i.erase_active = g_info.erase_active;
  i.log_q_dropped = g_info.log_q_dropped;
  i.flushes = g_info.flushes;
  i.flush_ms_max = g_info.flush_ms_max;
  i.mount_ms = g_info.mount_ms;
  i.last_download_frames = g_info.last_download_frames;
  g_info = i;
  portEXIT_CRITICAL(&g_mux);
}

void do_flush() {
  if (g_log->pending_bytes() == 0) return;
  const uint32_t t0 = millis();
  if (!g_log->flush()) EVO_LOG("store", "flash write FAILED");
  const uint32_t dt = millis() - t0;
  g_first_pending_ms = 0;
  portENTER_CRITICAL(&g_mux);
  ++g_info.flushes;
  if (dt > g_info.flush_ms_max) g_info.flush_ms_max = dt;
  portEXIT_CRITICAL(&g_mux);
}

void set_flag(bool Info::*field, bool v) {
  portENTER_CRITICAL(&g_mux);
  g_info.*field = v;
  portEXIT_CRITICAL(&g_mux);
}

void push_bulk_all(const uint8_t* b, size_t n) {
  size_t off = 0;
  for (int i = 0; i < 200 && off < n && ble::status().connected; ++i)
    off += txq::push_bulk(b + off, n - off, pdMS_TO_TICKS(10));
}

void finish_download(bool send_ack) {
  if (send_ack) {
    uint8_t b[16];
    const size_t n = rb::encode_ack(true, rb::kClass, rb::msg::kDownload, b, sizeof b);
    push_bulk_all(b, n);  // no stream: sai depois do último registro
  }
  g_dl = false;
  g_dl_len = 0;
  g_cancel_dl = false;
  txq::set_live_muted(false);
  portENTER_CRITICAL(&g_mux);
  g_info.download_active = false;
  g_info.last_download_frames = g_dl_frames;
  portEXIT_CRITICAL(&g_mux);
  EVO_LOG("store", "download %s: %u frames", send_ack ? "done" : "aborted (client gone)",
          (unsigned)g_dl_frames);
}

void download_step() {
  if (!ble::status().connected) {
    finish_download(false);
    return;
  }
  for (size_t k = 0; k < kDownloadBatch; ++k) {
    if (g_dl_len == 0) {
      if (g_cancel_dl) {
        finish_download(true);  // cancel: para de ler, drena o que já estava no stream, ACK
        return;
      }
      uint8_t tag, len, p[80];
      if (!g_log->read_next(&g_dl_cursor, g_dl_end, &tag, p, &len)) {
        finish_download(true);
        return;
      }
      g_dl_len = rb::encode_frame(rb::kClass, tag, p, len, g_dl_frame, sizeof g_dl_frame);
      g_dl_off = 0;
    }
    g_dl_off += txq::push_bulk(g_dl_frame + g_dl_off, g_dl_len - g_dl_off, pdMS_TO_TICKS(5));
    if (g_dl_off < g_dl_len) return;  // stream cheio: backpressure, volta depois
    g_dl_len = 0;
    ++g_dl_frames;
  }
}

void erase_run() {
  g_erase_req = false;
  g_cancel_erase = false;
  txq::set_live_muted(true);
  do_flush();
  const size_t nb = g_log->erase_blocks();
  const uint32_t t0 = millis();
  bool cancelled = false;
  for (size_t b = 0; b < nb; ++b) {
    if (g_cancel_erase) {
      cancelled = true;
      break;
    }
    if (!g_log->erase_block_from_top(b)) EVO_LOG("store", "erase block %u FAILED", (unsigned)b);
    const uint8_t pct = static_cast<uint8_t>((b + 1) * 100 / nb);
    uint8_t f[16];
    const size_t n = rb::encode_frame(rb::kClass, rb::msg::kErase, &pct, 1, f, sizeof f);
    txq::push_response(f, n);
    vTaskDelay(1);
  }
  g_log->mount();  // índice reflete o que sobrou (cancel = parcialmente apagado)
  portENTER_CRITICAL(&g_mux);
  g_shadow = g_log->used_bytes();
  portEXIT_CRITICAL(&g_mux);
  uint8_t a[16];
  const size_t n = rb::encode_ack(true, rb::kClass, rb::msg::kErase, a, sizeof a);
  txq::push_response(a, n);
  txq::set_live_muted(false);
  set_flag(&Info::erase_active, false);
  publish();
  EVO_LOG("store", "erase %s in %u ms: %u records left", cancelled ? "CANCELLED" : "done",
          (unsigned)(millis() - t0), (unsigned)g_log->history_count());
}

void task(void*) {
  for (;;) {
    Item it;
    TickType_t wait = (g_dl || g_req != Req::kNone || g_begin || g_erase_req) ? 0 : pdMS_TO_TICKS(20);
    while (xQueueReceive(g_q, &it, wait) == pdTRUE) {
      wait = 0;
      const FlashLog::Append r = g_log->append(it.tag, it.p, it.len);
      if (r != FlashLog::Append::kOk)
        EVO_LOG("store", "append 0x%02X failed (%d)", it.tag, static_cast<int>(r));
      if (g_first_pending_ms == 0) g_first_pending_ms = millis();
    }
    const size_t pend = g_log->pending_bytes();
    if (pend >= kFlushBytes || (pend > 0 && millis() - g_first_pending_ms >= kFlushMs)) do_flush();

    if (g_req == Req::kPrepare) {
      do_flush();
      g_dl_cursor = 0;
      g_dl_end = g_log->flushed_end();  // snapshot: a gravação segue depois daqui (R12)
      g_req_expected = g_log->history_count() + g_log->state_count();
      g_dl_frames = 0;
      txq::set_live_muted(true);
      set_flag(&Info::download_active, true);
      g_req_ok = true;
      g_req = Req::kNone;
      xSemaphoreGive(g_req_done);
    } else if (g_req == Req::kFlush) {
      do_flush();
      g_req_ok = true;
      g_req = Req::kNone;
      xSemaphoreGive(g_req_done);
    }
    if (g_begin) {
      g_begin = false;
      g_dl = true;
    }
    if (g_dl) download_step();
    if (g_erase_req) erase_run();
    publish();
  }
}

bool request(Req r, uint32_t timeout_ms) {
  xSemaphoreTake(g_req_done, 0);  // limpa sinal antigo
  g_req_ok = false;
  g_req = r;
  xTaskNotifyGive(g_task);
  if (xSemaphoreTake(g_req_done, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return false;
  return g_req_ok;
}

}  // namespace

bool start() {
  const esp_partition_t* p = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, static_cast<esp_partition_subtype_t>(0x40), "storage");
  if (!p) {
    EVO_LOG("store", "partition 'storage' not found");
    return false;
  }
  g_io = new PartitionFlash(p);
  g_log = new FlashLog(*g_io);
  const uint32_t t0 = millis();
  if (!g_log->mount()) {
    EVO_LOG("store", "mount FAILED");
    return false;
  }
  g_info.mount_ms = millis() - t0;
  g_shadow = g_log->used_bytes();
  g_q = xQueueCreate(48, sizeof(Item));
  g_req_done = xSemaphoreCreateBinary();
  publish();
  EVO_LOG("store", "log mounted in %u ms: %u records + %u markers, %u%% of %u KB (capacity %u records)",
          (unsigned)g_info.mount_ms, (unsigned)g_log->history_count(), (unsigned)g_log->state_count(),
          g_log->memory_pct(), (unsigned)(g_io->size() / 1024), (unsigned)g_log->capacity_history());
  xTaskCreatePinnedToCore(task, "storage", 6144, nullptr, 2, &g_task, 0);
  return true;
}

Info info() {
  portENTER_CRITICAL(&g_mux);
  Info i = g_info;
  portEXIT_CRITICAL(&g_mux);
  return i;
}

bool append(uint8_t tag, const uint8_t* payload, uint8_t len) {
  if (!g_log || len > 80) return false;
  const size_t rec = len + FlashLog::kRecordOverhead;
  portENTER_CRITICAL(&g_mux);
  size_t next;
  const bool fits = FlashLog::next_wp(g_shadow, rec, g_io->size(), &next);
  if (fits) g_shadow = next;
  portEXIT_CRITICAL(&g_mux);
  if (!fits) return false;
  Item it;
  it.tag = tag;
  it.len = len;
  memcpy(it.p, payload, len);
  if (xQueueSend(g_q, &it, 0) != pdTRUE) {
    portENTER_CRITICAL(&g_mux);
    ++g_info.log_q_dropped;  // reserva fica (conservador): nunca sobrescreve
    portEXIT_CRITICAL(&g_mux);
    return true;  // a gravação segue; a perda está contada
  }
  return true;
}

bool can_start() {
  if (!g_log) return false;
  portENTER_CRITICAL(&g_mux);
  size_t next;
  const bool ok = FlashLog::next_wp(g_shadow, FlashLog::kStateRecord + FlashLog::kHistoryRecord,
                                    g_io->size(), &next);
  portEXIT_CRITICAL(&g_mux);
  return ok;
}

bool prepare_download(uint32_t* expected) {
  if (!g_log) return false;
  if (!request(Req::kPrepare, 1000)) return false;
  *expected = g_req_expected;
  return true;
}

void begin_download() {
  g_begin = true;
  xTaskNotifyGive(g_task);
}

void cancel_download() { g_cancel_dl = true; }

bool start_erase() {
  if (!g_log) return false;
  set_flag(&Info::erase_active, true);
  g_erase_req = true;
  xTaskNotifyGive(g_task);
  return true;
}

void cancel_erase() { g_cancel_erase = true; }

bool flush_now(uint32_t timeout_ms) { return g_log && request(Req::kFlush, timeout_ms); }

}  // namespace storage
