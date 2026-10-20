#include "app/builder.h"

#include "app/log.h"
#include "config.h"
#include "frame.h"
#include "gnss/gnss.h"
#include "itow.h"

namespace builder {
namespace {

constexpr size_t kMaxSinks = 4;
QueueHandle_t g_pvt_q = nullptr;
LiveSink g_sinks[kMaxSinks] = {};
size_t g_nsinks = 0;
MotionSource g_motion = nullptr;
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
Stats g_stats;
rb::GnssConfig g_gnss_cfg;
volatile bool g_log_frames = false;
rb::ItowMonotonic g_itow;
rb::Motion g_last_motion = [] {
  rb::Motion m;
  m.gz_mg = 1000.0f;  // sem IMU: parado e nivelado (SPEC §11 F2)
  return m;
}();

void task(void*) {
  gnss::Pvt pvt;
  for (;;) {
    if (xQueueReceive(g_pvt_q, &pvt, portMAX_DELAY) != pdTRUE) continue;
    const uint32_t t0 = micros();

    rb::Motion m = g_last_motion;
    bool stall = false;
    if (g_motion) {
      if (g_motion(&m)) g_last_motion = m;
      else {
        m = g_last_motion;
        stall = true;
      }
    }

    portENTER_CRITICAL(&g_mux);
    const rb::GnssConfig gc = g_gnss_cfg;
    portEXIT_CRITICAL(&g_mux);

    rb::BuildOptions opt;
    opt.speed_3d = gc.speed_3d != 0;
    opt.battery = rb::battery_byte(cfg::kBatteryMeasured, 0, false);
    rb::LiveData d = rb::live_from_navpvt(pvt.raw, m, opt);
    rb::apply_min_hacc(d, gc.min_hacc_dm);
    const bool valid_time = (pvt.raw[11] & 0x02) != 0;
    const uint32_t resolved_before = g_itow.time_resolved_events();
    const uint32_t back_before = g_itow.backstep_fixes();
    d.itow = g_itow.process(d.itow, valid_time, pvt.rx_ms);
    if (g_itow.time_resolved_events() != resolved_before)
      EVO_LOG("build", "GNSS time resolved: iTOW offset %u ms applied until reboot",
              (unsigned)g_itow.offset());
    if (g_itow.backstep_fixes() != back_before)
      EVO_LOG("build", "receiver iTOW stepped back; wire iTOW kept monotonic (offset %u)",
              (unsigned)g_itow.offset());

    uint8_t payload[rb::kLivePayloadLen];
    rb::encode_live(d, payload);
    uint8_t frame[kFrameLen];
    rb::encode_frame(rb::kClass, rb::msg::kLive, payload, sizeof payload, frame, sizeof frame);
    for (size_t i = 0; i < g_nsinks; ++i) g_sinks[i](frame, d, pvt.rx_ms);

    const uint32_t us = micros() - t0;
    const uint32_t lat = millis() - pvt.rx_ms;
    portENTER_CRITICAL(&g_mux);
    ++g_stats.frames;
    if (stall) ++g_stats.imu_stall;
    g_stats.itow_time_resolved = g_itow.time_resolved_events();
    g_stats.itow_backsteps = g_itow.backstep_fixes();
    if (us > g_stats.build_us_max) g_stats.build_us_max = us;
    if (lat > g_stats.latency_ms_max) g_stats.latency_ms_max = lat;
    g_stats.last_itow_out = d.itow;
    portEXIT_CRITICAL(&g_mux);

    if (g_log_frames) {
      EVO_LOG("frame", "iTOW=%u fix=%u ok=%u sv=%u spd=%d g=(%d,%d,%d) rot=(%d,%d,%d)",
              (unsigned)d.itow, d.fix_status, d.fix_flags & 1, d.num_sv, (int)d.speed_mmps, d.gx,
              d.gy, d.gz, d.rx, d.ry, d.rz);
    }
  }
}

}  // namespace

void start(QueueHandle_t pvt_q) {
  g_pvt_q = pvt_q;
  xTaskCreatePinnedToCore(task, "builder", 4096, nullptr, 4, nullptr, 1);
}

void add_sink(LiveSink sink) {
  if (g_nsinks < kMaxSinks) g_sinks[g_nsinks++] = sink;
}

void set_motion_source(MotionSource src) { g_motion = src; }

void set_gnss_config(const rb::GnssConfig& c) {
  portENTER_CRITICAL(&g_mux);
  g_gnss_cfg = c;
  portEXIT_CRITICAL(&g_mux);
}

Stats stats() {
  portENTER_CRITICAL(&g_mux);
  Stats s = g_stats;
  portEXIT_CRITICAL(&g_mux);
  return s;
}

void reset_stats() {
  portENTER_CRITICAL(&g_mux);
  const uint32_t last = g_stats.last_itow_out;
  g_stats = Stats{};
  g_stats.last_itow_out = last;
  portEXIT_CRITICAL(&g_mux);
}

void set_log_frames(bool on) { g_log_frames = on; }

}  // namespace builder
