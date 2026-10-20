#include "app/device.h"

#include <freertos/semphr.h>

#include "app/builder.h"
#include "app/log.h"
#include "app/settings.h"
#include "ble/ble_server.h"
#include "ble/tx_queue.h"
#include "config.h"
#include "frame.h"
#include "gnss/gnss.h"
#include "storage/storage.h"

namespace device {
namespace {

SemaphoreHandle_t g_lock = nullptr;  // recorder + config (builder × BLE × console)
struct Lock {
  Lock() { xSemaphoreTake(g_lock, portMAX_DELAY); }
  ~Lock() { xSemaphoreGive(g_lock); }
};

rb::RecordingConfig g_cfg;  // espelho da NVS
volatile uint8_t g_default_rate = GA_DEFAULT_LIVE_RATE;
volatile bool g_shutdown = false;

bool connected() { return ble::status().connected; }

// Live segue a taxa da gravação; sem gravação, a padrão (SPEC §5.3).
void apply_rate(bool rec, uint8_t rec_rate) { gnss::request_rate(rec ? rec_rate : g_default_rate); }

class Sink : public rec::Sink {
 public:
  bool store(uint8_t tag, const uint8_t* p, uint8_t len) override { return storage::append(tag, p, len); }
  void live_state(const rb::StateChange& s) override {
    if (!ble::status().subscribed) return;
    uint8_t p[rb::kStateChangeLen];
    rb::encode_state_change(s, p);
    uint8_t f[rb::kStateChangeLen + rb::kOverhead];
    const size_t n = rb::encode_frame(rb::kClass, rb::msg::kStateChange, p, sizeof p, f, sizeof f);
    txq::push_response(f, n);
  }
  void stopped_by_itself(bool memory_full) override {
    // Chamado com o lock do recorder já tomado.
    EVO_LOG("rec", "recording stopped by itself (%s); live continues", memory_full ? "memory full" : "?");
    g_cfg.enable = 0;
    settings::set_rec_config(g_cfg);
    apply_rate(false, 0);
  }
  void shutdown() override {
    EVO_LOG("rec", "auto-shutdown: no data for %u s and no BLE client", g_cfg.shutdown_interval_s);
    g_shutdown = true;  // a task power faz flush e deep sleep
  }
};

Sink g_sink;
rec::Recorder g_rec(g_sink);

class Dev : public cmd::Device {
 public:
  bool recording() const override {
    Lock l;
    return g_rec.recording();
  }
  rb::RecordingConfig rec_config() const override {
    Lock l;
    return g_rec.recording() ? g_rec.config() : g_cfg;
  }
  rb::RecordingStatus status() const override {
    const storage::Info i = storage::info();
    rb::RecordingStatus s;
    {
      Lock l;
      s.recording = g_rec.recording() ? 1 : 0;
    }
    s.memory_pct = i.memory_pct;
    s.security = 0x00;  // segurança desabilitada (SPEC §7.7)
    s.stored = i.history;
    s.capacity = i.capacity;  // o que cabe de verdade (SPEC §7.1)
    return s;
  }
  bool can_start_recording() const override { return storage::can_start(); }
  bool download_active() const override { return storage::info().download_active; }
  bool erase_active() const override { return storage::info().erase_active; }
  void start_recording(const rb::RecordingConfig& c) override {
    Lock l;
    g_rec.start(c, millis(), connected());
    if (!g_rec.recording()) return;  // nem o marcador coube
    g_cfg = c;
    g_cfg.enable = 1;
    settings::set_rec_config(g_cfg);
    apply_rate(true, c.rate);
    EVO_LOG("rec", "recording started: rate code %u, flags 0x%02X", c.rate, c.flags);
  }
  void reconfigure_recording(const rb::RecordingConfig& c) override {
    Lock l;
    const uint8_t old_rate = g_rec.config().rate;
    g_rec.reconfigure(c, millis());
    g_cfg = c;
    g_cfg.enable = g_rec.recording() ? 1 : 0;
    settings::set_rec_config(g_cfg);
    if (c.rate != old_rate) apply_rate(g_rec.recording(), c.rate);
    EVO_LOG("rec", "recording reconfigured: rate code %u, flags 0x%02X", c.rate, c.flags);
  }
  void stop_recording() override {
    Lock l;
    g_rec.stop();
    g_cfg.enable = 0;
    settings::set_rec_config(g_cfg);
    apply_rate(false, 0);
    EVO_LOG("rec", "recording stopped");
  }
  bool prepare_download(uint32_t* expected) override { return storage::prepare_download(expected); }
  void begin_download() override { storage::begin_download(); }
  void cancel_download() override { storage::cancel_download(); }
  bool start_erase() override { return storage::start_erase(); }
  void cancel_erase() override { storage::cancel_erase(); }
  rb::GnssConfig gnss_config() const override { return settings::gnss_config(); }
  void set_gnss_config(const rb::GnssConfig& c) override {
    const rb::GnssConfig old = settings::gnss_config();
    settings::set_gnss_config(c);
    builder::set_gnss_config(c);
    if (c.dyn_model != old.dyn_model) gnss::request_dyn_model(c.dyn_model);
  }
};

class Out : public cmd::Output {
 public:
  void response(const uint8_t* f, size_t n) override { txq::push_response(f, n); }
  void bulk(const uint8_t* f, size_t n) override { txq::push_bulk(f, n, pdMS_TO_TICKS(100)); }
};

Dev g_dev;
Out g_out;
cmd::Processor g_proc(g_dev, g_out);

}  // namespace

void start() {
  g_lock = xSemaphoreCreateMutex();
  g_cfg = settings::rec_config();
  if (g_cfg.enable) {
    // Retoma a gravação salva com um 0x26 state=1 novo (SPEC §7.2).
    EVO_LOG("rec", "resuming saved recording (rate code %u, flags 0x%02X)", g_cfg.rate, g_cfg.flags);
    Lock l;
    if (storage::can_start()) {
      g_rec.start(g_cfg, millis(), false);
      apply_rate(g_rec.recording(), g_cfg.rate);
    } else {
      EVO_LOG("rec", "memory full: saved recording not resumed");
      g_cfg.enable = 0;
      settings::set_rec_config(g_cfg);
    }
  }
}

void on_live(const uint8_t frame[88], const rb::LiveData& d, uint32_t) {
  Lock l;
  g_rec.on_frame(d, frame + 6, millis(), connected());
}

void on_command(const rb::Frame& f) {
  EVO_LOG("cmd", "0x%02X 0x%02X len=%u", f.cls, f.id, f.len);
  g_proc.handle(f);
}

void tick() {
  Lock l;
  g_rec.on_tick(millis(), connected());
}

bool rec_start() {
  rb::RecordingConfig c;
  {
    Lock l;
    if (g_rec.recording()) return true;
    c = g_cfg;
  }
  if (!storage::can_start()) return false;
  g_dev.start_recording(c);
  return recording();
}

void rec_stop() { g_dev.stop_recording(); }

bool recording() {
  Lock l;
  return g_rec.recording();
}

rec::State rec_state() {
  Lock l;
  return g_rec.state();
}

rb::RecordingConfig rec_config() { return g_dev.rec_config(); }

rec::Stats rec_stats() {
  Lock l;
  return g_rec.stats();
}

cmd::Stats cmd_stats() { return g_proc.stats(); }

void set_default_live_rate(uint8_t code) {
  g_default_rate = code;
  Lock l;
  if (!g_rec.recording()) apply_rate(false, 0);
}

uint8_t default_live_rate() { return g_default_rate; }

bool console_erase() {
  if (recording() || storage::info().download_active || storage::info().erase_active) return false;
  return storage::start_erase();
}

bool shutdown_requested() { return g_shutdown; }

}  // namespace device
