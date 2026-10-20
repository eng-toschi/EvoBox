#include "app/settings.h"

#include <Preferences.h>

#include "app/log.h"

namespace settings {
namespace {

Preferences g_prefs;
SemaphoreHandle_t g_lock = nullptr;

struct Lock {
  Lock() { xSemaphoreTake(g_lock, portMAX_DELAY); }
  ~Lock() { xSemaphoreGive(g_lock); }
};

}  // namespace

void begin() {
  g_lock = xSemaphoreCreateMutex();
  if (!g_prefs.begin("evobox", false)) EVO_LOG("nvs", "Preferences.begin failed");
}

uint32_t gnss_baud() {
  Lock l;
  return g_prefs.getULong("gnss_baud", 0);
}

void set_gnss_baud(uint32_t baud) {
  Lock l;
  if (g_prefs.getULong("gnss_baud", 0) != baud) g_prefs.putULong("gnss_baud", baud);
}

uint8_t max_rate_hz() {
  Lock l;
  return g_prefs.getUChar("max_rate", 0);
}

void set_max_rate_hz(uint8_t hz) {
  Lock l;
  g_prefs.putUChar("max_rate", hz);
}

rb::RecordingConfig rec_config() {
  Lock l;
  uint8_t b[rb::kRecConfigLen];
  rb::RecordingConfig c;
  if (g_prefs.getBytes("rec_cfg", b, sizeof b) == sizeof b)
    rb::decode_rec_config(b, sizeof b, c, &c);
  return c;
}

void set_rec_config(const rb::RecordingConfig& c) {
  Lock l;
  uint8_t b[rb::kRecConfigLen];
  rb::encode_rec_config(c, b);
  g_prefs.putBytes("rec_cfg", b, sizeof b);
}

rb::GnssConfig gnss_config() {
  Lock l;
  uint8_t b[rb::kGnssConfigLen];
  if (g_prefs.getBytes("gnss_cfg", b, sizeof b) == sizeof b) {
    const rb::GnssConfig c = rb::decode_gnss_config(b);
    if (rb::valid_gnss_config(c)) return c;
  }
  return rb::GnssConfig{};
}

void set_gnss_config(const rb::GnssConfig& c) {
  Lock l;
  uint8_t b[rb::kGnssConfigLen];
  rb::encode_gnss_config(c, b);
  g_prefs.putBytes("gnss_cfg", b, sizeof b);
}

bool load_blob(const char* key, void* data, size_t len) {
  Lock l;
  return g_prefs.getBytesLength(key) == len && g_prefs.getBytes(key, data, len) == len;
}

void save_blob(const char* key, const void* data, size_t len) {
  Lock l;
  g_prefs.putBytes(key, data, len);
}

void erase_key(const char* key) {
  Lock l;
  g_prefs.remove(key);
}

}  // namespace settings
