#include <cmath>

#include "app/log.h"
#include "app/settings.h"
#include "axis.h"
#include "config.h"
#include "frame.h"
#include "imu/imu.h"
#include "imu/mpu6050.h"
#include "itow.h"

namespace imu {
namespace {

constexpr const char* kCalKey = "cal";

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
bool g_present = false;
Stats g_stats;

// Acumuladores da janela corrente (task IMU escreve, builder fecha).
int64_t g_sum[6] = {};
uint32_t g_n = 0;

// Calibração: só o builder mexe no Calibrator; o console pede por flags.
cal::Calibrator g_cal;
CalStatus g_cal_snapshot;
volatile bool g_req_static = false;
volatile bool g_req_reset = false;
volatile bool g_persist_pending = false;
volatile bool g_erase_pending = false;
cal::Blob g_persist_blob;
bool g_static_running = false;
bool g_auto_static = false;
bool g_have_prev_itow = false;
uint32_t g_prev_itow = 0;
cal::Vec3 g_last_acc, g_last_gyro;
uint32_t g_last_stall_log_ms = 0;

void task(void*) {
  mpu::Sample buf[64];
  uint32_t t_rate = millis();
  uint32_t n_rate = 0;
  for (;;) {
    bool ovf = false;
    const size_t n = mpu::read_fifo(buf, 64, &ovf);
    if (ovf) {
      portENTER_CRITICAL(&g_mux);
      ++g_stats.fifo_overflows;
      portEXIT_CRITICAL(&g_mux);
    }
    if (n) {
      int64_t s[6] = {};
      for (size_t i = 0; i < n; ++i) {
        s[0] += buf[i].ax;
        s[1] += buf[i].ay;
        s[2] += buf[i].az;
        s[3] += buf[i].gx;
        s[4] += buf[i].gy;
        s[5] += buf[i].gz;
      }
      portENTER_CRITICAL(&g_mux);
      for (int k = 0; k < 6; ++k) g_sum[k] += s[k];
      g_n += n;
      g_stats.samples += n;
      portEXIT_CRITICAL(&g_mux);
      n_rate += n;
    }
    const uint32_t now = millis();
    if (now - t_rate >= 2000) {
      portENTER_CRITICAL(&g_mux);
      g_stats.rate_hz = n_rate * 1000.0f / (now - t_rate);
      portEXIT_CRITICAL(&g_mux);
      n_rate = 0;
      t_rate = now;
    }
    vTaskDelay(pdMS_TO_TICKS(5));  // FIFO de 1 KB = 85 ms a 1 kHz: folga grande
  }
}

void snapshot() {
  CalStatus s;
  s.level = g_cal.level();
  s.blob = g_cal.blob();
  s.dyn = g_cal.dynamic_stats();
  s.static_running = g_static_running;
  s.static_seconds = g_cal.static_seconds();
  s.auto_static = g_auto_static;
  portENTER_CRITICAL(&g_mux);
  g_cal_snapshot = s;
  portEXIT_CRITICAL(&g_mux);
}

void persist(const cal::Blob& b) {
  portENTER_CRITICAL(&g_mux);
  g_persist_blob = b;
  portEXIT_CRITICAL(&g_mux);
  g_persist_pending = true;
}

void begin_static(bool automatic) {
  g_cal.static_begin();
  g_static_running = true;
  g_auto_static = automatic;
}

}  // namespace

void start() {
  mpu::Info info;
  g_present = mpu::begin(&info);
  g_stats.present = g_present;
  g_stats.who_am_i = info.who_am_i;
  cal::Blob b;
  if (settings::load_blob(kCalKey, &b, sizeof b) && g_cal.load(b)) {
    EVO_LOG("imu", "calibration loaded (level %u: %s)", g_cal.level(),
            g_cal.level() == cal::kDynamic ? "static+dynamic" : "static only");
  } else {
    EVO_LOG("imu", "no saved calibration: wire axes = raw device axes until the device is still for 10 s (auto static)");
    begin_static(true);
  }
  snapshot();
  if (!g_present) {
    EVO_LOG("imu", "MPU6050 not found at 0x%02X (WHO_AM_I read failed)", pins::kMpuAddr);
    return;
  }
  EVO_LOG("imu", "MPU6050 WHO_AM_I=0x%02X, +-8 g, +-500 dps, 1 kHz FIFO", info.who_am_i);
  xTaskCreatePinnedToCore(task, "imu", 4096, nullptr, 5, nullptr, 1);
}

bool present() { return g_present; }

bool motion_for_epoch(const uint8_t* pvt, uint32_t, rb::Motion* out) {
  if (g_req_reset) {
    g_req_reset = false;
    g_cal.reset();
    g_persist_pending = false;
    g_erase_pending = true;  // NVS fora do builder
    begin_static(true);
    EVO_LOG("imu", "calibration reset");
  }
  if (g_req_static) {
    g_req_static = false;
    begin_static(false);
    EVO_LOG("imu", "static calibration: keep the device still for 10 s");
  }

  int64_t s[6];
  uint32_t n;
  portENTER_CRITICAL(&g_mux);
  for (int k = 0; k < 6; ++k) {
    s[k] = g_sum[k];
    g_sum[k] = 0;
  }
  n = g_n;
  g_n = 0;
  g_stats.last_window_n = n;
  if (n) {
    if (g_stats.min_window_n == 0 || n < g_stats.min_window_n) g_stats.min_window_n = n;
    if (n > g_stats.max_window_n) g_stats.max_window_n = n;
  } else {
    ++g_stats.empty_windows;
  }
  portEXIT_CRITICAL(&g_mux);

  const uint32_t itow = rb::get_u32(pvt);
  const float dt = g_have_prev_itow ? ((itow + rb::kMsPerWeek - g_prev_itow) % rb::kMsPerWeek) / 1000.0f : 0.0f;
  g_prev_itow = itow;
  g_have_prev_itow = true;

  if (n == 0) {
    if (g_present && millis() - g_last_stall_log_ms > 5000) {
      g_last_stall_log_ms = millis();
      EVO_LOG("imu", "empty IMU window (I2C stalled?) - repeating last values");
    }
    return false;
  }

  const float inv = 1.0f / n;
  const cal::Vec3 acc{s[0] * inv / mpu::kAccelLsbPerG, s[1] * inv / mpu::kAccelLsbPerG,
                      s[2] * inv / mpu::kAccelLsbPerG};
  const cal::Vec3 gyro{s[3] * inv / mpu::kGyroLsbPerDps, s[4] * inv / mpu::kGyroLsbPerDps,
                       s[5] * inv / mpu::kGyroLsbPerDps};
  g_last_acc = acc;
  g_last_gyro = gyro;

  if (g_static_running && dt > 0.0f) {
    g_cal.static_add(acc, gyro, dt);
    if (g_cal.static_seconds() >= cal::Calibrator::kStaticMinSeconds) {
      const cal::StaticResult r = g_cal.static_finish();
      if (r.ok) {
        g_static_running = false;
        EVO_LOG("imu", "static calibration OK: |g|=%.3f g, accel std=%.4f g, gyro bias=(%.2f, %.2f, %.2f) dps",
                r.gravity_g, r.accel_std_g, r.gyro_mean_dps.x, r.gyro_mean_dps.y, r.gyro_mean_dps.z);
        persist(g_cal.save());
      } else if (g_auto_static) {
        g_cal.static_begin();  // automática: tenta de novo na próxima janela parada
      } else {
        g_static_running = false;
        EVO_LOG("imu", "static calibration FAILED: %s (accel std=%.4f g, gyro std=%.3f dps, |g|=%.3f)",
                r.reason, r.accel_std_g, r.gyro_std_dps, r.gravity_g);
      }
    }
  }

  const bool good_fix = pvt[20] == 3 && (pvt[21] & 0x01);
  const float speed = rb::get_i32(pvt + 60) / 1000.0f;
  const cal::Level before = g_cal.level();
  if (g_cal.epoch_update(acc, gyro, speed, good_fix, dt)) {
    const cal::DynamicStats& d = g_cal.dynamic_stats();
    EVO_LOG("imu", "dynamic calibration %s: heading %.1f deg (corr %.2f, %u epochs)",
            before == cal::kDynamic ? "updated" : "converged", d.heading_deg, d.corr,
            (unsigned)d.samples);
    persist(g_cal.save());
  }
  snapshot();

  cal::Vec3 av, gv;
  g_cal.apply(acc, gyro, &av, &gv);
  const rb::AxisConvention& c = rb::kWireFromFlu;
  out->gx_mg = c.acc[0] * av.x * 1000.0f;
  out->gy_mg = c.acc[1] * av.y * 1000.0f;
  out->gz_mg = c.acc[2] * av.z * 1000.0f;
  out->rx_cdps = c.gyr[0] * gv.x * 100.0f;
  out->ry_cdps = c.gyr[1] * gv.y * 100.0f;
  out->rz_cdps = c.gyr[2] * gv.z * 100.0f;
  return true;
}

void request_static_cal() { g_req_static = true; }
void request_cal_reset() { g_req_reset = true; }

CalStatus cal_status() {
  portENTER_CRITICAL(&g_mux);
  CalStatus s = g_cal_snapshot;
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
  const bool p = g_stats.present;
  const uint8_t w = g_stats.who_am_i;
  g_stats = Stats{};
  g_stats.present = p;
  g_stats.who_am_i = w;
  portEXIT_CRITICAL(&g_mux);
}

void last_device_window(cal::Vec3* acc_g, cal::Vec3* gyro_dps) {
  *acc_g = g_last_acc;
  *gyro_dps = g_last_gyro;
}

void service_persistence() {
  if (g_erase_pending) {
    g_erase_pending = false;
    settings::erase_key(kCalKey);
    EVO_LOG("imu", "calibration erased from NVS");
  }
  if (!g_persist_pending) return;
  g_persist_pending = false;
  cal::Blob b;
  portENTER_CRITICAL(&g_mux);
  b = g_persist_blob;
  portEXIT_CRITICAL(&g_mux);
  settings::save_blob(kCalKey, &b, sizeof b);
  EVO_LOG("imu", "calibration saved to NVS (level %u)", b.level);
}

}  // namespace imu
