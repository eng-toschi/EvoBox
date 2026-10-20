// MPU6050 em I²C (SPEC §6.1): ±8 g, ±500 dps, 1 kHz, DLPF ~188 Hz, FIFO
// accel+gyro (12 B/amostra), I²C 400 kHz.
#pragma once

#include <Arduino.h>

namespace mpu {

constexpr float kAccelLsbPerG = 4096.0f;   // ±8 g
constexpr float kGyroLsbPerDps = 65.5f;    // ±500 dps
constexpr size_t kSampleBytes = 12;

struct Sample {
  int16_t ax, ay, az, gx, gy, gz;
};

struct Info {
  bool present = false;
  uint8_t who_am_i = 0;
};

// Configura e liga o FIFO. false se o sensor não responder.
bool begin(Info* info);
// Lê até `max` amostras do FIFO. Retorna o número lido; *overflow = true se
// o FIFO transbordou (o FIFO é resetado e as amostras perdidas contadas pelo chamador).
size_t read_fifo(Sample* out, size_t max, bool* overflow);
// Reinicia o FIFO (descarta o conteúdo).
void reset_fifo();

}  // namespace mpu
