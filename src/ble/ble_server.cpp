#include "ble/ble_server.h"

#include <NimBLEDevice.h>
#include <esp_system.h>
#include <freertos/stream_buffer.h>

#include "app/log.h"
#include "assembler.h"
#include "ble/identity.h"
#include "ble/tx_queue.h"
#include "config.h"

namespace ble {
namespace {

constexpr const char* kUartService = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr const char* kUartRx = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr const char* kUartTx = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr uint16_t kAdvFast = 160;  // 100 ms (unidades de 0,625 ms)
constexpr uint16_t kAdvSlow = 800;  // 500 ms
constexpr uint32_t kFastWindowMs = 30000;
constexpr uint16_t kDefaultMtu = 517;

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
Status g_st;
Stats g_stats;
CommandHandler g_handler = nullptr;
TaskHandle_t g_task = nullptr;
StreamBufferHandle_t g_rx = nullptr;
NimBLEServer* g_server = nullptr;
NimBLECharacteristic* g_tx = nullptr;
NimBLECharacteristic* g_hw = nullptr;
uint16_t g_tx_handle = 0;
volatile bool g_adv_restart = false;
uint32_t g_adv_start_ms = 0;
volatile bool g_first_live_pending = false;
rb::FrameAssembler g_rx_asm;

void set_conn(bool connected, uint16_t handle) {
  portENTER_CRITICAL(&g_mux);
  g_st.connected = connected;
  g_st.conn_handle = handle;
  if (!connected) {
    g_st.subscribed = false;
    g_st.mtu = 23;
  }
  portEXIT_CRITICAL(&g_mux);
}

class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, ble_gap_conn_desc* d) override {
    set_conn(true, d->conn_handle);
    portENTER_CRITICAL(&g_mux);
    g_st.advertising = false;
    g_st.conn_interval_x125 = d->conn_itvl;
    ++g_stats.connects;
    portEXIT_CRITICAL(&g_mux);
    // 7,5–15 ms, latência 0, supervision 4 s; a central decide (SPEC §4.3).
    s->updateConnParams(d->conn_handle, 6, 12, 0, 400);
    s->setDataLen(d->conn_handle, 251);
    g_rx_asm.reset();
    xTaskNotifyGive(g_task);
  }
  void onDisconnect(NimBLEServer*, ble_gap_conn_desc*) override {
    set_conn(false, 0xFFFF);
    txq::clear();
    portENTER_CRITICAL(&g_mux);
    ++g_stats.disconnects;
    portEXIT_CRITICAL(&g_mux);
    g_adv_restart = true;  // R15: volta a anunciar imediatamente
    xTaskNotifyGive(g_task);
  }
  void onMTUChange(uint16_t mtu, ble_gap_conn_desc*) override {
    portENTER_CRITICAL(&g_mux);
    g_st.mtu = mtu;
    portEXIT_CRITICAL(&g_mux);
  }
};

class TxCB : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* c, ble_gap_conn_desc*, uint16_t sub) override {
    const bool on = (sub & 0x0001) != 0;
    // Handles só existem depois que o servidor GATT inicia (no 1º advertising).
    g_tx_handle = c->getHandle();
    portENTER_CRITICAL(&g_mux);
    g_st.subscribed = on;
    if (on) {
      g_stats.subscribe_ms = millis();
      g_first_live_pending = true;
    }
    portEXIT_CRITICAL(&g_mux);
    xTaskNotifyGive(g_task);
  }
};

class RxCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, ble_gap_conn_desc*) override {
    NimBLEAttValue v = c->getValue();
    const size_t n = xStreamBufferSend(g_rx, v.data(), v.length(), 0);
    portENTER_CRITICAL(&g_mux);
    g_stats.rx_bytes += n;
    if (n < v.length()) g_stats.rx_bad += 1;
    portEXIT_CRITICAL(&g_mux);
    xTaskNotifyGive(g_task);
  }
};

// Strings do DIS: bytes exatos, sem '\0' (R5, CLAUDE.md regra 11). O
// setValue(const T&) templado do NimBLE 1.4 grava sizeof(T) bytes — para
// `const char*` isso é o ponteiro, para `char[N]` inclui o terminador.
void set_str(NimBLECharacteristic* c, const char* s) {
  c->setValue(reinterpret_cast<const uint8_t*>(s), strlen(s));
}

void adv_start(bool fast) {
  NimBLEAdvertising* a = NimBLEDevice::getAdvertising();
  if (a->isAdvertising()) a->stop();
  const uint16_t itv = fast ? kAdvFast : kAdvSlow;
  a->setMinInterval(itv);
  a->setMaxInterval(itv);
  const bool ok = a->start();
  g_adv_start_ms = millis();
  portENTER_CRITICAL(&g_mux);
  g_st.advertising = ok;
  g_st.adv_fast = fast;
  portEXIT_CRITICAL(&g_mux);
  if (!ok) EVO_LOG("ble", "advertising start failed");
}

enum class Tx { kOk, kRetry, kFail };

Tx notify_raw(uint16_t conn, const uint8_t* data, size_t len) {
  struct os_mbuf* om = ble_hs_mbuf_from_flat(data, len);
  if (!om) return Tx::kRetry;
  const int rc = ble_gattc_notify_custom(conn, g_tx_handle, om);  // consome o mbuf
  if (rc == 0) return Tx::kOk;
  if (rc == BLE_HS_ENOMEM || rc == BLE_HS_EBUSY) return Tx::kRetry;
  return Tx::kFail;
}

void on_rx_frame(const rb::Frame& f) {
  portENTER_CRITICAL(&g_mux);
  ++g_stats.rx_frames;
  portEXIT_CRITICAL(&g_mux);
  if (g_handler) g_handler(f);
}

void pump_rx() {
  uint8_t buf[128];
  size_t n;
  while ((n = xStreamBufferReceive(g_rx, buf, sizeof buf, 0)) > 0) g_rx_asm.feed(buf, n, on_rx_frame);
  portENTER_CRITICAL(&g_mux);
  g_stats.rx_bad = g_rx_asm.stats().bad_checksum + g_rx_asm.stats().bad_length;
  portEXIT_CRITICAL(&g_mux);
}

// Bloco pendente de TX (fragmentação por MTU sobrevive a ENOMEM).
uint8_t g_pend[520];
size_t g_pend_len = 0;
size_t g_pend_off = 0;
bool g_pend_live = false;

void pump_tx() {
  for (;;) {
    portENTER_CRITICAL(&g_mux);
    const bool ready = g_st.connected && g_st.subscribed;
    const uint16_t conn = g_st.conn_handle;
    const uint16_t mtu = g_st.mtu;
    portEXIT_CRITICAL(&g_mux);
    if (!ready) {
      g_pend_len = 0;
      return;
    }
    const size_t chunk = mtu > 3 ? mtu - 3 : 20;
    if (g_pend_len == 0) {
      g_pend_off = 0;
      g_pend_len = txq::next(g_pend, chunk < sizeof g_pend ? chunk : sizeof g_pend, &g_pend_live);
      if (g_pend_len == 0) return;
    }
    while (g_pend_off < g_pend_len) {
      const size_t piece = g_pend_len - g_pend_off < chunk ? g_pend_len - g_pend_off : chunk;
      const Tx r = notify_raw(conn, g_pend + g_pend_off, piece);
      if (r == Tx::kRetry) {
        portENTER_CRITICAL(&g_mux);
        ++g_stats.enomem_retries;
        portEXIT_CRITICAL(&g_mux);
        return;  // tenta de novo no próximo ciclo (1 ms); o builder nunca espera
      }
      if (r == Tx::kFail) {
        g_pend_len = 0;
        return;
      }
      g_pend_off += piece;
      portENTER_CRITICAL(&g_mux);
      ++g_stats.notifies;
      g_stats.notify_bytes += piece;
      portEXIT_CRITICAL(&g_mux);
    }
    if (g_pend_live && g_first_live_pending) {
      g_first_live_pending = false;
      portENTER_CRITICAL(&g_mux);
      g_stats.first_live_after_sub_ms = millis() - g_stats.subscribe_ms;
      portEXIT_CRITICAL(&g_mux);
    }
    g_pend_len = 0;
  }
}

void task(void*) {
  adv_start(true);
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(g_pend_len ? 1 : 20));
    pump_rx();
    if (g_adv_restart) {
      g_adv_restart = false;
      adv_start(true);
    }
    portENTER_CRITICAL(&g_mux);
    const bool adv = g_st.advertising, fast = g_st.adv_fast, conn = g_st.connected;
    portEXIT_CRITICAL(&g_mux);
    if (!conn && adv && fast && millis() - g_adv_start_ms > kFastWindowMs) adv_start(false);
    pump_tx();
  }
}

}  // namespace

void start(CommandHandler handler) {
  g_handler = handler;
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  identity::serial_from_mac(mac, g_st.serial);
  identity::advertised_name(GA_IDENTITY == GA_IDENTITY_GOAWAKE, g_st.serial, g_st.name,
                            sizeof g_st.name);

  txq::init();
  g_rx = xStreamBufferCreate(1024, 1);

  NimBLEDevice::init(g_st.name);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(kDefaultMtu);
  NimBLEDevice::setSecurityAuth(false, false, false);  // sem pairing/bonding (SPEC §4.1)

  g_server = NimBLEDevice::createServer();
  g_server->setCallbacks(new ServerCB());
  g_server->advertiseOnDisconnect(false);  // a task controla (intervalo rápido/lento)

  NimBLEService* uart = g_server->createService(kUartService);
  NimBLECharacteristic* rx =
      uart->createCharacteristic(kUartRx, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rx->setCallbacks(new RxCB());
  g_tx = uart->createCharacteristic(kUartTx, NIMBLE_PROPERTY::NOTIFY);
  g_tx->setCallbacks(new TxCB());
  uart->start();

  NimBLEService* dis = g_server->createService("180A");
  set_str(dis->createCharacteristic("2A24", NIMBLE_PROPERTY::READ),
          GA_IDENTITY == GA_IDENTITY_GOAWAKE ? "GoAwake Logger" : identity::kModelMiniS);
  set_str(dis->createCharacteristic("2A25", NIMBLE_PROPERTY::READ), g_st.serial);
  set_str(dis->createCharacteristic("2A26", NIMBLE_PROPERTY::READ), GA_DIS_FIRMWARE);
  g_hw = dis->createCharacteristic("2A27", NIMBLE_PROPERTY::READ);
  set_str(g_hw, "GA-ESP32");
  set_str(dis->createCharacteristic("2A29", NIMBLE_PROPERTY::READ), GA_DIS_MANUFACTURER);
  dis->start();

  // Advertising: flags + nome completo (30 B); UUID do UART no scan response (SPEC §4.1).
  NimBLEAdvertising* a = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData ad;
  ad.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  ad.setName(g_st.name);
  a->setAdvertisementData(ad);
  NimBLEAdvertisementData sr;
  sr.setCompleteServices(NimBLEUUID(kUartService));
  a->setScanResponseData(sr);

  xTaskCreatePinnedToCore(task, "ble", 4096, nullptr, 3, &g_task, 0);
  txq::set_consumer(g_task);
  EVO_LOG("ble", "advertising as \"%s\" (serial %s)", g_st.name, g_st.serial);
}

void set_hw_revision(const char* hw) {
  if (g_hw) set_str(g_hw, hw);
}

void on_live_frame(const uint8_t frame[88]) {
  portENTER_CRITICAL(&g_mux);
  const bool ready = g_st.connected && g_st.subscribed;
  portEXIT_CRITICAL(&g_mux);
  if (ready) txq::push_live(frame);
}

Status status() {
  portENTER_CRITICAL(&g_mux);
  Status s = g_st;
  portEXIT_CRITICAL(&g_mux);
  if (s.connected) {
    ble_gap_conn_desc d;
    if (ble_gap_conn_find(s.conn_handle, &d) == 0) s.conn_interval_x125 = d.conn_itvl;
  }
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
  const uint32_t c = g_stats.connects, d = g_stats.disconnects;
  g_stats = Stats{};
  g_stats.connects = c;
  g_stats.disconnects = d;
  portEXIT_CRITICAL(&g_mux);
}

void set_mtu_limit(uint16_t mtu) {
  const uint16_t m = mtu == 0 ? kDefaultMtu : mtu;
  NimBLEDevice::setMTU(m);
  portENTER_CRITICAL(&g_mux);
  g_st.mtu_limit = mtu;
  portEXIT_CRITICAL(&g_mux);
}

void disconnect() {
  portENTER_CRITICAL(&g_mux);
  const bool c = g_st.connected;
  const uint16_t h = g_st.conn_handle;
  portEXIT_CRITICAL(&g_mux);
  if (c && g_server) g_server->disconnect(h);
}

}  // namespace ble
