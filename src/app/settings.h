// Configuração persistente em NVS (SPEC §5.3 max_rate, §7.2, §7.6, §6.4).
#pragma once

#include <cstddef>
#include <cstdint>

#include "messages.h"

namespace settings {

void begin();

// Última baud em que o receptor respondeu (acelera o autobaud no boot, D10).
uint32_t gnss_baud();
void set_gnss_baud(uint32_t baud);

uint8_t max_rate_hz();  // teto D12; 0 = sem teto
void set_max_rate_hz(uint8_t hz);

rb::RecordingConfig rec_config();
void set_rec_config(const rb::RecordingConfig& c);

rb::GnssConfig gnss_config();
void set_gnss_config(const rb::GnssConfig& c);

// Blob opaco da calibração de montagem (imu/calibration). false se ausente.
bool load_blob(const char* key, void* data, size_t len);
void save_blob(const char* key, const void* data, size_t len);
void erase_key(const char* key);

}  // namespace settings
