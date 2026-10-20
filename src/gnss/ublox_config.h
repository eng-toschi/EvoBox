// UBX do receptor u-blox: montagem de comandos de configuração, parse de
// respostas e política de taxa (SPEC §2.2, §5.3, §5.4). C++ puro — testado em
// native (test_ublox). Nada aqui toca a UART.
#pragma once

#include <cstddef>
#include <cstdint>

namespace ubx {

// Classes / ids u-blox usados.
constexpr uint8_t kClsNav = 0x01;
constexpr uint8_t kClsAck = 0x05;
constexpr uint8_t kClsCfg = 0x06;
constexpr uint8_t kClsMon = 0x0A;
constexpr uint8_t kIdNavPvt = 0x07;
constexpr uint8_t kIdAckNak = 0x00;
constexpr uint8_t kIdAckAck = 0x01;
constexpr uint8_t kIdCfgPrt = 0x00;
constexpr uint8_t kIdCfgMsg = 0x01;
constexpr uint8_t kIdCfgRate = 0x08;
constexpr uint8_t kIdCfgCfg = 0x09;
constexpr uint8_t kIdCfgNav5 = 0x24;
constexpr uint8_t kIdCfgValset = 0x8A;
constexpr uint8_t kIdMonVer = 0x04;
constexpr uint8_t kIdMonGnss = 0x28;

constexpr size_t kMaxCmd = 64;  // maior comando que montamos

enum class Module : uint8_t { kUnknown, kM8, kM9, kM10 };
const char* module_name(Module m);

struct ModuleInfo {
  Module module = Module::kUnknown;
  char sw[31] = {};
  char hw[11] = {};
  char mod[31] = {};    // extensão "MOD=..." (vazia se o firmware não manda)
  char fwver[31] = {};  // extensão "FWVER=..."
  uint16_t protver_x100 = 0;  // "PROTVER=18.00" → 1800
  uint8_t max_rate_hz = 10;   // SPEC §2.2: M8 → 10, M9 → 25
  bool uses_valset = false;   // M9/M10: CFG-VALSET; M8: CFG legado
};

// Decodifica o payload de UBX-MON-VER. false se curto demais.
bool parse_mon_ver(const uint8_t* p, size_t len, ModuleInfo* out);

// UBX-MON-GNSS (M8+): bitmask de constelações habilitadas.
// b0 GPS, b1 GLONASS, b2 BeiDou, b3 Galileo.
struct GnssMask {
  uint8_t supported = 0;
  uint8_t enabled = 0;
  uint8_t simultaneous = 0;
};
bool parse_mon_gnss(const uint8_t* p, size_t len, GnssMask* out);
// "GPS+GLO+GAL" etc. em buf. Retorna buf.
const char* gnss_mask_str(uint8_t mask, char* buf, size_t cap);

// --- Montagem de comandos (cada função devolve o frame UBX completo) --------
size_t poll(uint8_t cls, uint8_t id, uint8_t* out, size_t cap);
// CFG-PRT UART1: 8N1, `baud`, entrada UBX+NMEA+RTCM, saída só UBX (SPEC §5.4:
// nenhuma sentença NMEA na UART).
size_t cfg_prt_uart1(uint32_t baud, uint8_t* out, size_t cap);
size_t cfg_prt_poll_uart1(uint8_t* out, size_t cap);
// CFG-MSG na porta atual (forma de 3 B).
size_t cfg_msg(uint8_t cls, uint8_t id, uint8_t rate, uint8_t* out, size_t cap);
size_t cfg_msg_poll(uint8_t cls, uint8_t id, uint8_t* out, size_t cap);
size_t cfg_rate(uint16_t meas_ms, uint8_t* out, size_t cap);
// CFG-NAV5: só dynModel (mask = 0x0001).
size_t cfg_nav5_dyn(uint8_t dyn_model, uint8_t* out, size_t cap);
// CFG-CFG: salva toda a config atual em BBR + flash (SPEC §5.4). Sem clear,
// sem load — nunca reset de fábrica.
size_t cfg_save(uint8_t* out, size_t cap);

// CFG-VALSET (M9/M10). Camadas: b0 RAM, b1 BBR, b2 flash.
constexpr uint8_t kLayerRam = 0x01;
constexpr uint8_t kLayerBbr = 0x02;
struct KeyVal {
  uint32_t key;
  uint32_t value;
};
size_t cfg_valset(uint8_t layers, const KeyVal* kv, size_t n, uint8_t* out, size_t cap);

// Chaves CFG (u-blox M9 interface description).
namespace key {
constexpr uint32_t kUart1Baud = 0x40520001;          // U4
constexpr uint32_t kUart1InUbx = 0x10730001;         // L
constexpr uint32_t kUart1OutUbx = 0x10740001;        // L
constexpr uint32_t kUart1OutNmea = 0x10740002;       // L
constexpr uint32_t kMsgNavPvtUart1 = 0x20910007;     // U1
constexpr uint32_t kRateMeas = 0x30210001;           // U2
constexpr uint32_t kRateNav = 0x30210002;            // U2
constexpr uint32_t kNavspgDynModel = 0x20110021;     // E1
constexpr uint32_t kSignalGpsEna = 0x1031001F;       // L
constexpr uint32_t kSignalGalEna = 0x10310021;       // L
constexpr uint32_t kSignalBdsEna = 0x10310022;       // L
constexpr uint32_t kSignalGloEna = 0x10310025;       // L
}  // namespace key

// --- Respostas de poll do protocolo legado (M8) ---------------------------
struct PrtUart {
  uint32_t baud = 0;
  uint16_t in_proto = 0;
  uint16_t out_proto = 0;
  uint32_t mode = 0;
};
bool parse_cfg_prt(const uint8_t* p, size_t len, PrtUart* out);
bool parse_cfg_rate(const uint8_t* p, size_t len, uint16_t* meas_ms, uint16_t* nav_rate);
bool parse_cfg_nav5_dyn(const uint8_t* p, size_t len, uint8_t* dyn_model);
// Resposta ao poll de CFG-MSG (8 B: class, id, rate[6]); rate da UART1.
bool parse_cfg_msg_uart1(const uint8_t* p, size_t len, uint8_t* rate);

// --- Política de taxa (SPEC §2.2, §5.3, D12) ---------------------------------
struct RateDecision {
  uint8_t requested_code;
  uint8_t effective_code;
  uint16_t period_ms;
  bool limited;  // a taxa pedida passou do máximo do módulo ou do teto
};
// module_max_hz: limite de hardware; cap_hz: teto `max_rate` (0 = sem teto).
RateDecision decide_rate(uint8_t requested_code, uint8_t module_max_hz, uint8_t cap_hz);

}  // namespace ubx
