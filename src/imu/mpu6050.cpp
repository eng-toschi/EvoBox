#include "imu/mpu6050.h"

#include <Wire.h>

#include "config.h"

namespace mpu {
namespace {

constexpr uint8_t kRegSmplrtDiv = 0x19;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegFifoEn = 0x23;
constexpr uint8_t kRegIntEnable = 0x38;
constexpr uint8_t kRegIntStatus = 0x3A;
constexpr uint8_t kRegUserCtrl = 0x6A;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegFifoCountH = 0x72;
constexpr uint8_t kRegFifoRw = 0x74;
constexpr uint8_t kRegWhoAmI = 0x75;

bool write_reg(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(pins::kMpuAddr);
  Wire.write(reg);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}

size_t g_last_got = 0;

bool read_regs(uint8_t reg, uint8_t* out, size_t n) {
  Wire.beginTransmission(pins::kMpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  g_last_got = Wire.requestFrom(static_cast<uint16_t>(pins::kMpuAddr), static_cast<uint8_t>(n));
  if (g_last_got != n) return false;
  for (size_t i = 0; i < n; ++i) out[i] = static_cast<uint8_t>(Wire.read());
  return true;
}

int16_t be16(const uint8_t* p) { return static_cast<int16_t>((p[0] << 8) | p[1]); }

}  // namespace

bool begin(Info* info) {
  Wire.begin(pins::kI2cSda, pins::kI2cScl, cfg::kI2cHz);
  uint8_t who = 0;
  if (!read_regs(kRegWhoAmI, &who, 1)) return false;
  info->who_am_i = who;
  bool ok = write_reg(kRegPwrMgmt1, 0x80);  // DEVICE_RESET
  delay(100);
  ok &= write_reg(0x68, 0x07);               // SIGNAL_PATH_RESET gyro/accel/temp (datasheet RM §4.28)
  delay(100);
  ok &= write_reg(kRegPwrMgmt1, 0x01);       // acorda, clock = PLL do giro X
  ok &= write_reg(0x6C, 0x00);               // PWR_MGMT_2: nenhum eixo em standby
  delay(50);
  ok &= write_reg(kRegConfig, 0x01);         // DLPF_CFG=1: accel 184 Hz / gyro 188 Hz → 1 kHz
  ok &= write_reg(kRegSmplrtDiv, 0x00);      // 1 kHz / (1 + 0)
  ok &= write_reg(kRegGyroConfig, 0x08);     // ±500 dps
  ok &= write_reg(kRegAccelConfig, 0x10);    // ±8 g
  ok &= write_reg(kRegIntEnable, 0x00);
  ok &= write_reg(kRegFifoEn, 0x78);         // XG, YG, ZG, ACCEL (sem temperatura)
  reset_fifo();
  delay(100);
  uint8_t c[2] = {};
  read_regs(kRegPwrMgmt1, &info->pwr_mgmt_1, 1);
  read_regs(kRegUserCtrl, &info->user_ctrl, 1);
  read_regs(kRegFifoEn, &info->fifo_en, 1);
  read_regs(kRegConfig, &info->config, 1);
  info->count_read_ok = read_regs(kRegFifoCountH, c, 2);
  info->fifo_count_20ms = static_cast<uint16_t>((c[0] << 8) | c[1]);
  read_regs(kRegIntStatus, &info->int_status, 1);
  uint8_t d[14];
  info->raw_read_ok = read_regs(0x3B, d, 14);
  info->raw_got = g_last_got;
  if (info->raw_read_ok) info->raw = {be16(d), be16(d + 2), be16(d + 4), be16(d + 8), be16(d + 10), be16(d + 12)};
  info->temp_raw = be16(d + 6);
  if (info->raw.ax == 0 && info->raw.ay == 0 && info->raw.az == 0 && info->temp_raw == 0) {
    // Nenhum ADC rodando com o PLL do giro: tenta o oscilador interno de 8 MHz.
    write_reg(kRegPwrMgmt1, 0x00);
    delay(100);
    reset_fifo();
    delay(100);
    read_regs(kRegFifoCountH, c, 2);
    info->fifo_count_internal_clk = static_cast<uint16_t>((c[0] << 8) | c[1]);
    if (read_regs(0x3B, d, 14)) {
      info->raw = {be16(d), be16(d + 2), be16(d + 4), be16(d + 8), be16(d + 10), be16(d + 12)};
      info->temp_raw = be16(d + 6);
    }
    info->tried_internal_clk = true;
  }
  info->present = ok;
  return ok;
}

void reset_fifo() {
  write_reg(kRegUserCtrl, 0x04);  // FIFO_RESET
  write_reg(kRegUserCtrl, 0x40);  // FIFO_EN
}

size_t read_fifo(Sample* out, size_t max, bool* overflow) {
  *overflow = false;
  uint8_t st = 0;
  if (read_regs(kRegIntStatus, &st, 1) && (st & 0x10)) {  // FIFO_OFLOW_INT
    reset_fifo();
    *overflow = true;
    return 0;
  }
  uint8_t c[2];
  if (!read_regs(kRegFifoCountH, c, 2)) return 0;
  size_t avail = static_cast<size_t>((c[0] << 8) | c[1]) / kSampleBytes;
  if (avail > max) avail = max;
  size_t got = 0;
  uint8_t buf[kSampleBytes * 10];  // 120 B < buffer de 128 B do Wire
  while (got < avail) {
    const size_t n = (avail - got) < 10 ? (avail - got) : 10;
    if (!read_regs(kRegFifoRw, buf, n * kSampleBytes)) break;
    for (size_t i = 0; i < n; ++i) {
      const uint8_t* p = buf + i * kSampleBytes;
      out[got + i] = {be16(p), be16(p + 2), be16(p + 4), be16(p + 6), be16(p + 8), be16(p + 10)};
    }
    got += n;
  }
  return got;
}

}  // namespace mpu
