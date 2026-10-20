// Pinos e build flags — único lugar (CLAUDE.md, SPEC §2, §9).
#pragma once

#include <cstdint>

// --- Versão / identidade (SPEC §4.1–§4.2) -----------------------------------
#define EVOBOX_FW_VERSION "0.1.0"
#define GA_DIS_FIRMWARE "3.3"  // ≥ 3.3 libera GNSS config e preset 20 Hz no cliente
#define GA_DIS_MANUFACTURER "GoAwake"

#define GA_IDENTITY_RACEBOX_MINI_S 1
#define GA_IDENTITY_GOAWAKE 2
#ifndef GA_IDENTITY
#define GA_IDENTITY GA_IDENTITY_RACEBOX_MINI_S
#endif

// --- Simulação (SPEC §9.3) -----------------------------------------------------
#ifndef GA_SIM
#define GA_SIM 0
#endif

// --- Taxa padrão do live sem gravação (SPEC §5.3): 0 = 25 Hz / máx. do módulo
#ifndef GA_DEFAULT_LIVE_RATE
#define GA_DEFAULT_LIVE_RATE 0
#endif

namespace pins {
// GNSS na UART2 (SPEC §2.1)
constexpr int kGnssRx = 16;  // ← TX do GPS
constexpr int kGnssTx = 17;  // → RX do GPS
// MPU6050 em I²C
constexpr int kI2cSda = 21;
constexpr int kI2cScl = 22;
constexpr uint8_t kMpuAddr = 0x68;
// Opcionais (SPEC §2.3) — não ligados na placa atual
constexpr int kMpuInt = 4;
constexpr int kPps = 34;
constexpr int kBatteryAdc = 35;
constexpr int kButton = 0;  // BOOT
constexpr int kLed = 2;
}  // namespace pins

namespace cfg {
constexpr uint32_t kConsoleBaud = 115200;
constexpr uint32_t kGnssBootBaud = 9600;     // default de fábrica
constexpr uint32_t kGnssBaudM8 = 115200;     // SPEC §5.4
constexpr uint32_t kGnssBaudM9 = 230400;
constexpr uint32_t kI2cHz = 400000;
// Bateria: placa atual sem divisor → sem medição (SPEC §5.5, D3).
constexpr bool kBatteryMeasured = false;
}  // namespace cfg
