// Task GNSS — ver gnss.h.
#include <Arduino.h>

#include "app/log.h"
#include "app/settings.h"
#include "assembler.h"
#include "config.h"
#include "frame.h"
#include "gnss/gnss.h"
#include "itow.h"
#include "messages.h"

namespace gnss {
namespace {

HardwareSerial& uart = Serial2;
QueueHandle_t g_pvt_q = nullptr;
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
Status g_status;
Stats g_stats;
volatile bool g_log_pvt = false;

// Pedidos de outras tasks (0xFF = nenhum).
volatile uint8_t g_req_rate = 0xFF;
volatile uint8_t g_req_cap = 0xFF;
volatile uint8_t g_req_dyn = 0xFF;

rb::FrameAssembler g_asm(rb::kMaxPayload);

// Última resposta (não NAV-PVT) para wait_for().
struct Resp {
  uint8_t cls, id;
  uint16_t len;
  uint8_t payload[rb::kMaxPayload];
  bool valid;
} g_resp;
struct AckState {
  uint8_t cls, id;
  bool ack;
  bool valid;
} g_ack;

// Estado de Δ iTOW para estatística.
bool g_have_last = false;
bool g_skip_next_dt = true;  // primeiro Δ após (re)configuração não conta
uint32_t g_last_rx_ms = 0;

void on_pvt(const uint8_t* p, uint32_t now) {
  const uint32_t itow = rb::get_u32(p);
  const uint16_t period = g_status.period_ms;
  portENTER_CRITICAL(&g_mux);
  ++g_stats.pvt;
  if (g_have_last && !g_skip_next_dt) {
    const uint32_t dt = (itow + rb::kMsPerWeek - g_stats.last_itow) % rb::kMsPerWeek;
    if (dt == period) ++g_stats.dt_on_period; else ++g_stats.dt_off_period;
    if (period && dt * 2 > period * 3u) ++g_stats.gaps;
    if (g_stats.dt_min == 0 || dt < g_stats.dt_min) g_stats.dt_min = dt;
    if (dt > g_stats.dt_max) g_stats.dt_max = dt;
    const uint32_t arr = now - g_last_rx_ms;
    const uint32_t jit = arr > period ? arr - period : period - arr;
    if (jit > g_stats.rx_jitter_max_ms) g_stats.rx_jitter_max_ms = jit;
  }
  g_skip_next_dt = false;
  g_have_last = true;
  g_stats.last_itow = itow;
  if (g_status.first_pvt_ms == 0) g_status.first_pvt_ms = now;
  portEXIT_CRITICAL(&g_mux);
  g_last_rx_ms = now;

  Pvt item;
  memcpy(item.raw, p, rb::kNavPvtLen);
  item.rx_ms = now;
  if (g_pvt_q && xQueueSend(g_pvt_q, &item, 0) != pdTRUE) {
    portENTER_CRITICAL(&g_mux);
    ++g_stats.pvt_dropped;
    portEXIT_CRITICAL(&g_mux);
  }
  if (g_log_pvt) {
    EVO_LOG("pvt", "iTOW=%u fix=%u flags=0x%02X valid=0x%02X sv=%u hAcc=%umm gSpeed=%dmm/s",
            (unsigned)itow, p[20], p[21], p[11], p[23], (unsigned)rb::get_u32(p + 40),
            (int)rb::get_i32(p + 60));
  }
}

void on_frame(const rb::Frame& f) {
  const uint32_t now = millis();
  if (f.cls == ubx::kClsNav && f.id == ubx::kIdNavPvt && f.len == rb::kNavPvtLen) {
    on_pvt(f.payload, now);
    return;
  }
  if (f.cls == ubx::kClsAck && f.len == 2) {
    g_ack.cls = f.payload[0];
    g_ack.id = f.payload[1];
    g_ack.ack = f.id == ubx::kIdAckAck;
    g_ack.valid = true;
    return;
  }
  g_resp.cls = f.cls;
  g_resp.id = f.id;
  g_resp.len = f.len;
  memcpy(g_resp.payload, f.payload, f.len);
  g_resp.valid = true;
}

void pump() {
  uint8_t buf[256];
  int n;
  while ((n = uart.available()) > 0) {
    const int k = uart.read(buf, n > (int)sizeof buf ? (int)sizeof buf : n);
    if (k <= 0) break;
    g_asm.feed(buf, static_cast<size_t>(k), on_frame);
  }
  portENTER_CRITICAL(&g_mux);
  g_stats.bad_checksum = g_asm.stats().bad_checksum;
  g_stats.noise_bytes = g_asm.stats().noise_bytes;
  portEXIT_CRITICAL(&g_mux);
}

void send(const uint8_t* b, size_t n) {
  uart.write(b, n);
  uart.flush();
}

bool wait_resp(uint8_t cls, uint8_t id, uint32_t timeout_ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    pump();
    if (g_resp.valid && g_resp.cls == cls && g_resp.id == id) return true;
    vTaskDelay(1);
  }
  return false;
}

// Manda um CFG e espera ACK/NAK. +1 ACK, 0 NAK, −1 sem resposta.
int send_cfg(const uint8_t* b, size_t n, uint32_t timeout_ms = 500) {
  for (int attempt = 0; attempt < 3; ++attempt) {
    g_ack.valid = false;
    send(b, n);
    const uint32_t t0 = millis();
    while (millis() - t0 < timeout_ms) {
      pump();
      if (g_ack.valid && g_ack.cls == b[2] && g_ack.id == b[3]) return g_ack.ack ? 1 : 0;
      vTaskDelay(1);
    }
  }
  return -1;
}

bool poll_resp(const uint8_t* b, size_t n, uint8_t cls, uint8_t id, uint32_t timeout_ms = 400) {
  for (int attempt = 0; attempt < 3; ++attempt) {
    g_resp.valid = false;
    send(b, n);
    if (wait_resp(cls, id, timeout_ms)) return true;
  }
  return false;
}

void open_uart(uint32_t baud) {
  uart.end();
  uart.setRxBufferSize(4096);
  uart.begin(baud, SERIAL_8N1, pins::kGnssRx, pins::kGnssTx);
  g_asm.reset();
  vTaskDelay(pdMS_TO_TICKS(20));
  while (uart.available()) uart.read();
}

bool probe_mon_ver(ubx::ModuleInfo* info) {
  uint8_t b[16];
  const size_t n = ubx::poll(ubx::kClsMon, ubx::kIdMonVer, b, sizeof b);
  for (int attempt = 0; attempt < 2; ++attempt) {
    g_resp.valid = false;
    send(b, n);
    if (wait_resp(ubx::kClsMon, ubx::kIdMonVer, 300))
      return ubx::parse_mon_ver(g_resp.payload, g_resp.len, info);
  }
  return false;
}

bool autobaud(uint32_t preferred, ubx::ModuleInfo* info, uint32_t* found) {
  const uint32_t candidates[] = {preferred, cfg::kGnssBaudM8, cfg::kGnssBootBaud,
                                 cfg::kGnssBaudM9, 38400, 57600, 460800};
  for (size_t i = 0; i < sizeof candidates / sizeof candidates[0]; ++i) {
    const uint32_t b = candidates[i];
    bool dup = false;
    for (size_t j = 0; j < i; ++j) dup |= candidates[j] == b;
    if (dup) continue;
    open_uart(b);
    if (probe_mon_ver(info)) {
      *found = b;
      return true;
    }
  }
  return false;
}

void set_status_rate(const ubx::RateDecision& d) {
  portENTER_CRITICAL(&g_mux);
  g_status.requested_code = d.requested_code;
  g_status.effective_code = d.effective_code;
  g_status.period_ms = d.period_ms;
  g_status.limited = d.limited;
  g_skip_next_dt = true;
  portEXIT_CRITICAL(&g_mux);
}

void log_rate(const ubx::RateDecision& d) {
  if (d.limited) {
    EVO_LOG("gnss", "rate %u Hz requested (code %u) > limit (module %u Hz, cap %u Hz): LIMITED to %u Hz (%u ms)",
            rb::rate_hz(d.requested_code), d.requested_code, g_status.info.max_rate_hz,
            g_status.cap_hz, rb::rate_hz(d.effective_code), d.period_ms);
  } else {
    EVO_LOG("gnss", "rate %u Hz (%u ms)", rb::rate_hz(d.effective_code), d.period_ms);
  }
}

// Aplica a taxa no receptor (só RAM: não grava na flash do módulo).
bool apply_rate(uint8_t code) {
  const ubx::RateDecision d = ubx::decide_rate(code, g_status.info.max_rate_hz, g_status.cap_hz);
  uint8_t b[ubx::kMaxCmd + 8];
  size_t n;
  if (g_status.info.uses_valset) {
    const ubx::KeyVal kv[] = {{ubx::key::kRateMeas, d.period_ms}, {ubx::key::kRateNav, 1}};
    n = ubx::cfg_valset(ubx::kLayerRam, kv, 2, b, sizeof b);
  } else {
    n = ubx::cfg_rate(d.period_ms, b, sizeof b);
  }
  const int r = send_cfg(b, n);
  if (r != 1) {
    EVO_LOG("gnss", "rate change to %u ms: %s", d.period_ms, r == 0 ? "NAK" : "no answer");
    return false;
  }
  const bool changed = d.period_ms != g_status.period_ms;
  set_status_rate(d);
  if (changed) {
    portENTER_CRITICAL(&g_mux);
    ++g_stats.rate_changes;
    portEXIT_CRITICAL(&g_mux);
  }
  log_rate(d);
  return true;
}

bool apply_dyn_model(uint8_t model, bool persist) {
  uint8_t b[ubx::kMaxCmd + 8];
  size_t n;
  if (g_status.info.uses_valset) {
    const ubx::KeyVal kv[] = {{ubx::key::kNavspgDynModel, model}};
    n = ubx::cfg_valset(persist ? (ubx::kLayerRam | ubx::kLayerBbr) : ubx::kLayerRam, kv, 1, b,
                        sizeof b);
  } else {
    n = ubx::cfg_nav5_dyn(model, b, sizeof b);
  }
  if (send_cfg(b, n) != 1) {
    EVO_LOG("gnss", "dynModel %u: rejected/no answer", model);
    return false;
  }
  g_status.dyn_model = model;
  return true;
}

bool save_m8() {
  uint8_t b[32];
  const size_t n = ubx::cfg_save(b, sizeof b);
  const int r = send_cfg(b, n, 1500);
  EVO_LOG("gnss", "CFG-CFG save (BBR+flash): %s", r == 1 ? "ACK" : r == 0 ? "NAK" : "no answer");
  return r == 1;
}

// Configuração M8 (CFG legado). Compara antes de escrever: só grava na
// flash do módulo se algo persistente mudou (evita desgaste a cada boot).
bool configure_m8(uint8_t rate_code, uint8_t dyn_model) {
  uint8_t b[ubx::kMaxCmd + 8];
  bool persistent_change = false;

  size_t n = ubx::cfg_prt_poll_uart1(b, sizeof b);
  ubx::PrtUart prt;
  if (poll_resp(b, n, ubx::kClsCfg, ubx::kIdCfgPrt) && ubx::parse_cfg_prt(g_resp.payload, g_resp.len, &prt)) {
    if (prt.out_proto != 0x0001 || prt.baud != g_status.baud) {
      n = ubx::cfg_prt_uart1(g_status.baud, b, sizeof b);
      if (send_cfg(b, n) != 1) EVO_LOG("gnss", "CFG-PRT (UBX only): no ACK");
      persistent_change = true;
    }
  } else {
    EVO_LOG("gnss", "CFG-PRT poll: no answer");
    return false;
  }

  n = ubx::cfg_msg_poll(ubx::kClsNav, ubx::kIdNavPvt, b, sizeof b);
  uint8_t pvt_rate = 0;
  if (poll_resp(b, n, ubx::kClsCfg, ubx::kIdCfgMsg) && ubx::parse_cfg_msg_uart1(g_resp.payload, g_resp.len, &pvt_rate) &&
      pvt_rate == 1) {
    // já habilitado
  } else {
    n = ubx::cfg_msg(ubx::kClsNav, ubx::kIdNavPvt, 1, b, sizeof b);
    if (send_cfg(b, n) != 1) {
      EVO_LOG("gnss", "CFG-MSG NAV-PVT: no ACK");
      return false;
    }
    persistent_change = true;
  }

  n = ubx::poll(ubx::kClsCfg, ubx::kIdCfgNav5, b, sizeof b);
  uint8_t dyn = 0xFF;
  if (poll_resp(b, n, ubx::kClsCfg, ubx::kIdCfgNav5) && ubx::parse_cfg_nav5_dyn(g_resp.payload, g_resp.len, &dyn) &&
      dyn == dyn_model) {
    g_status.dyn_model = dyn;
  } else {
    if (!apply_dyn_model(dyn_model, true)) return false;
    persistent_change = true;
  }

  if (!apply_rate(rate_code)) return false;

  if (persistent_change) g_status.saved_to_module = save_m8();
  return true;
}

// Configuração M9/M10 (CFG-VALSET em RAM+BBR — BBR não desgasta).
// ‹VALIDAR›: sem NEO-M9N na bancada, este caminho não foi testado em hardware.
bool configure_valset(uint8_t rate_code, uint8_t dyn_model) {
  uint8_t b[ubx::kMaxCmd + 8];
  const ubx::KeyVal kv[] = {{ubx::key::kUart1OutUbx, 1},
                            {ubx::key::kUart1OutNmea, 0},
                            {ubx::key::kMsgNavPvtUart1, 1},
                            {ubx::key::kNavspgDynModel, dyn_model}};
  const size_t n = ubx::cfg_valset(ubx::kLayerRam | ubx::kLayerBbr, kv, 4, b, sizeof b);
  if (send_cfg(b, n) != 1) {
    EVO_LOG("gnss", "CFG-VALSET base config: rejected/no answer");
    return false;
  }
  g_status.dyn_model = dyn_model;
  g_status.saved_to_module = true;
  return apply_rate(rate_code);
}

// Troca a baud do receptor e da UART local; confirma com MON-VER.
bool switch_baud(uint32_t target, ubx::ModuleInfo* info) {
  uint8_t b[ubx::kMaxCmd + 8];
  size_t n;
  if (info->uses_valset) {
    const ubx::KeyVal kv[] = {{ubx::key::kUart1Baud, target}};
    n = ubx::cfg_valset(ubx::kLayerRam | ubx::kLayerBbr, kv, 1, b, sizeof b);
  } else {
    n = ubx::cfg_prt_uart1(target, b, sizeof b);
  }
  send(b, n);  // o ACK pode sair já na baud nova: não espera
  vTaskDelay(pdMS_TO_TICKS(150));
  open_uart(target);
  return probe_mon_ver(info);
}

void poll_mon_gnss() {
  uint8_t b[16];
  const size_t n = ubx::poll(ubx::kClsMon, ubx::kIdMonGnss, b, sizeof b);
  ubx::GnssMask m;
  if (poll_resp(b, n, ubx::kClsMon, ubx::kIdMonGnss) && ubx::parse_mon_gnss(g_resp.payload, g_resp.len, &m))
    g_status.gnss = m;
}

bool bring_up(uint8_t rate_code, uint8_t dyn_model) {
  ubx::ModuleInfo info;
  uint32_t baud = 0;
  const uint32_t last = settings::gnss_baud();
  if (!autobaud(last ? last : cfg::kGnssBaudM8, &info, &baud)) {
    EVO_LOG("gnss", "no u-blox receiver answered MON-VER on any baud");
    return false;
  }
  EVO_LOG("gnss", "found %s (sw=\"%s\" hw=%s fw=\"%s\" mod=\"%s\" protver=%u.%02u) at %u baud",
          ubx::module_name(info.module), info.sw, info.hw, info.fwver, info.mod,
          info.protver_x100 / 100, info.protver_x100 % 100, (unsigned)baud);
  // 230400 só onde há 25 Hz (M9); M8/M10 a 10 Hz cabem em 115200 (SPEC §5.4).
  const uint32_t target = info.max_rate_hz > 10 ? cfg::kGnssBaudM9 : cfg::kGnssBaudM8;
  if (baud != target) {
    if (!switch_baud(target, &info)) {
      EVO_LOG("gnss", "switch to %u baud failed; retrying bring-up", (unsigned)target);
      return false;
    }
    baud = target;
    EVO_LOG("gnss", "now at %u baud", (unsigned)baud);
  }
  settings::set_gnss_baud(baud);
  portENTER_CRITICAL(&g_mux);
  g_status.info = info;
  g_status.present = true;
  g_status.baud = baud;
  portEXIT_CRITICAL(&g_mux);

  const bool ok = info.uses_valset ? configure_valset(rate_code, dyn_model)
                                   : configure_m8(rate_code, dyn_model);
  if (!ok) return false;
  poll_mon_gnss();
  char gbuf[24];
  EVO_LOG("gnss", "configured: NAV-PVT only, dynModel=%u, GNSS=%s%s", g_status.dyn_model,
          ubx::gnss_mask_str(g_status.gnss.enabled, gbuf, sizeof gbuf),
          g_status.saved_to_module ? ", saved to module" : "");
  g_status.configured = true;
  return true;
}

uint8_t g_rate_code = 0;
uint8_t g_dyn_model = 4;

void task(void*) {
  for (;;) {
    if (bring_up(g_rate_code, g_dyn_model)) break;
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  // Tempo até o 1º NAV-PVT configurado (D10).
  const uint32_t t0 = millis();
  while (g_status.first_pvt_ms == 0 && millis() - t0 < 3000) {
    pump();
    vTaskDelay(1);
  }
  g_status.bringup_ms = g_status.first_pvt_ms;
  EVO_LOG("gnss", "first NAV-PVT at %u ms after boot", (unsigned)g_status.bringup_ms);

  for (;;) {
    pump();
    if (g_req_cap != 0xFF) {
      g_status.cap_hz = g_req_cap;
      g_req_cap = 0xFF;
      apply_rate(g_status.requested_code);
    }
    if (g_req_rate != 0xFF) {
      const uint8_t code = g_req_rate;
      g_req_rate = 0xFF;
      const ubx::RateDecision d = ubx::decide_rate(code, g_status.info.max_rate_hz, g_status.cap_hz);
      if (d.period_ms != g_status.period_ms || code != g_status.requested_code) apply_rate(code);
    }
    if (g_req_dyn != 0xFF) {
      const uint8_t m = g_req_dyn;
      g_req_dyn = 0xFF;
      if (apply_dyn_model(m, true) && !g_status.info.uses_valset) save_m8();
    }
    vTaskDelay(1);
  }
}

}  // namespace

void start(QueueHandle_t pvt_q, uint8_t rate_code, uint8_t cap_hz, uint8_t dyn_model) {
  g_pvt_q = pvt_q;
  g_rate_code = rate_code;
  g_dyn_model = dyn_model;
  g_status.cap_hz = cap_hz;
  g_status.requested_code = rate_code;
  xTaskCreatePinnedToCore(task, "gnss", 6144, nullptr, 5, nullptr, 1);
}

void request_rate(uint8_t code) { g_req_rate = code; }
void request_cap(uint8_t cap_hz) { g_req_cap = cap_hz; }
void request_dyn_model(uint8_t model) { g_req_dyn = model; }

Status status() {
  portENTER_CRITICAL(&g_mux);
  Status s = g_status;
  portEXIT_CRITICAL(&g_mux);
  return s;
}

Stats stats() {
  portENTER_CRITICAL(&g_mux);
  Stats s = g_stats;
  portEXIT_CRITICAL(&g_mux);
  return s;
}

void reset_stats() {
  portENTER_CRITICAL(&g_mux);
  const uint32_t last = g_stats.last_itow;
  g_stats = Stats{};
  g_stats.last_itow = last;
  g_skip_next_dt = true;
  portEXIT_CRITICAL(&g_mux);
}

void set_log_pvt(bool on) { g_log_pvt = on; }

}  // namespace gnss
