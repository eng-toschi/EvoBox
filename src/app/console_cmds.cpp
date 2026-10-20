// Comandos do console (SPEC §9.2) que leem/mexem nos módulos.
#include "app/console_cmds.h"

#include <Arduino.h>

#include <cstdlib>
#include <cstring>

#include "app/builder.h"
#include "app/console.h"
#include "app/device.h"
#include "app/settings.h"
#include "app/sim_runtime.h"
#include "ble/ble_server.h"
#include "ble/tx_queue.h"
#include "config.h"
#include "gnss/gnss.h"
#include "imu/imu.h"
#include "messages.h"
#include "storage/storage.h"

namespace console_cmds {
namespace {

bool parse_onoff(const char* s, bool* on) {
  if (strcmp(s, "on") == 0) *on = true;
  else if (strcmp(s, "off") == 0) *on = false;
  else return false;
  return true;
}

const char* state_name(rec::State s) {
  switch (s) {
    case rec::State::kWaitingFix: return "WAITING FIX";
    case rec::State::kRunning: return "RECORDING";
    case rec::State::kPaused: return "PAUSED";
    default: return "stopped";
  }
}

bool cmd_gnss_info(const char*) {
  const gnss::Status s = gnss::status();
  char gbuf[24];
  Serial.printf("module    : %s (present=%d configured=%d)\n", ubx::module_name(s.info.module),
                s.present, s.configured);
  Serial.printf("sw / hw   : \"%s\" / %s\n", s.info.sw, s.info.hw);
  Serial.printf("fw / mod  : \"%s\" / \"%s\" protver %u.%02u\n", s.info.fwver, s.info.mod,
                s.info.protver_x100 / 100, s.info.protver_x100 % 100);
  Serial.printf("uart      : %u baud\n", (unsigned)s.baud);
  Serial.printf("max rate  : %u Hz (module)  cap %u Hz (0 = none)\n", s.info.max_rate_hz, s.cap_hz);
  Serial.printf("rate      : requested %u Hz -> effective %u Hz (%u ms)%s\n",
                rb::rate_hz(s.requested_code), rb::rate_hz(s.effective_code), s.period_ms,
                s.limited ? "  [LIMITED]" : "");
  Serial.printf("gnss      : enabled %s (supported %s)\n",
                ubx::gnss_mask_str(s.gnss.enabled, gbuf, sizeof gbuf),
                ubx::gnss_mask_str(s.gnss.supported, gbuf + 12, 12));
  Serial.printf("dynModel  : %u\n", s.dyn_model);
  Serial.printf("first PVT : %u ms after boot; saved to module this boot: %d\n",
                (unsigned)s.first_pvt_ms, s.saved_to_module);
  return true;
}

bool cmd_stats(const char* args) {
  if (strcmp(args, "reset") == 0) {
    gnss::reset_stats();
    builder::reset_stats();
    ble::reset_stats();
    imu::reset_stats();
    txq::reset_stats();
    Serial.println("stats reset");
    return true;
  }
  if (*args) return false;
  const gnss::Stats g = gnss::stats();
  const gnss::Status s = gnss::status();
  const builder::Stats b = builder::stats();
  const uint32_t n = g.dt_on_period + g.dt_off_period;
  Serial.printf("gnss: pvt=%u dropped=%u badck=%u noise=%u rate_changes=%u\n", (unsigned)g.pvt,
                (unsigned)g.pvt_dropped, (unsigned)g.bad_checksum, (unsigned)g.noise_bytes,
                (unsigned)g.rate_changes);
  Serial.printf("gnss: dITOW period=%u ms: on=%u off=%u gaps=%u min=%u max=%u (n=%u) rx_jitter_max=%u ms\n",
                s.period_ms, (unsigned)g.dt_on_period, (unsigned)g.dt_off_period, (unsigned)g.gaps,
                (unsigned)g.dt_min, (unsigned)g.dt_max, (unsigned)n, (unsigned)g.rx_jitter_max_ms);
  const ble::Stats bs = ble::stats();
  const txq::Stats ts = txq::stats();
  Serial.printf("ble: connects=%u disconnects=%u notifies=%u bytes=%u enomem_retries=%u rx_frames=%u rx_bad=%u first_live_after_sub=%u ms\n",
                (unsigned)bs.connects, (unsigned)bs.disconnects, (unsigned)bs.notifies,
                (unsigned)bs.notify_bytes, (unsigned)bs.enomem_retries, (unsigned)bs.rx_frames,
                (unsigned)bs.rx_bad, (unsigned)bs.first_live_after_sub_ms);
  Serial.printf("txq: live_queued=%u live_dropped=%u live_muted=%u resp_dropped=%u\n",
                (unsigned)ts.live_queued, (unsigned)ts.live_dropped, (unsigned)ts.live_muted,
                (unsigned)ts.resp_dropped);
  const imu::Stats is = imu::stats();
  Serial.printf("imu: present=%d who=0x%02X samples=%u rate=%.1f Hz overflows=%u empty_windows=%u window_n last=%u min=%u max=%u\n",
                is.present, is.who_am_i, (unsigned)is.samples, is.rate_hz, (unsigned)is.fifo_overflows,
                (unsigned)is.empty_windows, (unsigned)is.last_window_n, (unsigned)is.min_window_n,
                (unsigned)is.max_window_n);
  const rec::Stats rs = device::rec_stats();
  const cmd::Stats cs = device::cmd_stats();
  Serial.printf("rec: stored=%u markers=%u pauses=%u resumes=%u full_stops=%u\n", (unsigned)rs.stored,
                (unsigned)rs.markers, (unsigned)rs.pauses, (unsigned)rs.resumes, (unsigned)rs.full_stops);
  Serial.printf("cmd: handled=%u acks=%u nacks=%u idempotent_25=%u foreign_ignored=%u\n", (unsigned)cs.handled,
                (unsigned)cs.acks, (unsigned)cs.nacks, (unsigned)cs.idempotent_25,
                (unsigned)cs.ignored_foreign_class);
  Serial.printf("build: frames=%u imu_stall=%u itow_resolved=%u itow_backsteps=%u build_us_max=%u latency_ms_max=%u\n",
                (unsigned)b.frames, (unsigned)b.imu_stall, (unsigned)b.itow_time_resolved,
                (unsigned)b.itow_backsteps, (unsigned)b.build_us_max, (unsigned)b.latency_ms_max);
  return true;
}

bool cmd_status(const char*) {
  const ble::Status b = ble::status();
  const gnss::Status g = gnss::status();
  Serial.printf("fw        : EvoBox %s (DIS fw %s), sim=%d\n", EVOBOX_FW_VERSION, GA_DIS_FIRMWARE, GA_SIM);
  Serial.printf("ble       : \"%s\" %s%s mtu=%u interval=%u.%02u ms%s\n", b.name,
                b.connected ? "CONNECTED" : (b.advertising ? (b.adv_fast ? "advertising(100ms)" : "advertising(500ms)") : "idle"),
                b.subscribed ? " SUBSCRIBED" : "", b.mtu, b.conn_interval_x125 * 125 / 100,
                b.conn_interval_x125 * 125 % 100, b.mtu_limit ? " [mtu limit set]" : "");
  Serial.printf("gnss      : %s %u Hz%s\n", ubx::module_name(g.info.module), rb::rate_hz(g.effective_code),
                g.limited ? " (limited)" : "");
  const rb::RecordingConfig rc = device::rec_config();
  const storage::Info si = storage::info();
  Serial.printf("recording : %s  rate code %u flags 0x%02X stat %u mm/s %u s nofix %u s shutdown %u s\n",
                state_name(device::rec_state()), rc.rate, rc.flags, rc.stationary_speed_mmps,
                rc.stationary_interval_s, rc.nofix_interval_s, rc.shutdown_interval_s);
  Serial.printf("memory    : %u records, %u%%\n", (unsigned)si.history, si.memory_pct);
  return true;
}

bool cmd_ble(const char* args) {
  if (strncmp(args, "mtu ", 4) == 0) {
    const long v = strtol(args + 4, nullptr, 10);
    if (v != 0 && (v < 23 || v > 517)) return false;
    ble::set_mtu_limit(static_cast<uint16_t>(v));
    Serial.printf("local MTU limit = %ld (0 = default 517); applies to the next connection\n", v);
    return true;
  }
  if (strcmp(args, "disconnect") == 0) {
    ble::disconnect();
    return true;
  }
  return false;
}

bool cmd_cal(const char* args) {
  if (strcmp(args, "static") == 0) {
    imu::request_static_cal();
    Serial.println("static calibration requested (keep still 10 s)");
    return true;
  }
  if (strcmp(args, "reset") == 0) {
    imu::request_cal_reset();
    Serial.println("calibration reset requested");
    return true;
  }
  if (strcmp(args, "show") != 0) return false;
  const imu::CalStatus c = imu::cal_status();
  const cal::Blob& b = c.blob;
  Serial.printf("level     : %u (%s)%s%s\n", c.level,
                c.level == cal::kDynamic ? "static+dynamic" : c.level == cal::kStatic ? "static only" : "none",
                c.static_running ? " [static running]" : "", c.auto_static ? " [auto]" : "");
  Serial.printf("R_dv      : [% .4f % .4f % .4f]\n            [% .4f % .4f % .4f]\n            [% .4f % .4f % .4f]\n",
                b.r[0], b.r[1], b.r[2], b.r[3], b.r[4], b.r[5], b.r[6], b.r[7], b.r[8]);
  Serial.printf("gyro bias : (% .3f, % .3f, % .3f) dps\n", b.gyro_bias[0], b.gyro_bias[1], b.gyro_bias[2]);
  Serial.printf("dynamic   : samples=%u s_aa=%.2f corr=%.3f heading=%.1f deg converged=%d bias_updates=%u\n",
                (unsigned)c.dyn.samples, c.dyn.s_aa, c.dyn.corr, c.dyn.heading_deg, c.dyn.converged,
                (unsigned)c.dyn.bias_updates);
  cal::Vec3 a, g;
  imu::last_device_window(&a, &g);
  Serial.printf("raw window: acc=(% .4f, % .4f, % .4f) g  gyro=(% .3f, % .3f, % .3f) dps (device frame)\n",
                a.x, a.y, a.z, g.x, g.y, g.z);
  return true;
}

bool cmd_rec(const char* args) {
  if (strcmp(args, "start") == 0) {
    Serial.println(device::rec_start() ? "recording" : "cannot start (memory full?)");
    return true;
  }
  if (strcmp(args, "stop") == 0) {
    device::rec_stop();
    Serial.println("stopped");
    return true;
  }
  return false;
}

bool cmd_mem(const char*) {
  const storage::Info i = storage::info();
  Serial.printf("log       : %u records (0x21) + %u markers (0x26), capacity %u records\n",
                (unsigned)i.history, (unsigned)i.states, (unsigned)i.capacity);
  Serial.printf("usage     : %u / %u bytes = %u%%%s\n", (unsigned)i.used_bytes, (unsigned)i.size_bytes,
                i.memory_pct, i.full ? " FULL" : "");
  Serial.printf("ops       : download=%d erase=%d log_q_dropped=%u flushes=%u flush_ms_max=%u mount_ms=%u last_download=%u frames\n",
                i.download_active, i.erase_active, (unsigned)i.log_q_dropped, (unsigned)i.flushes,
                (unsigned)i.flush_ms_max, (unsigned)i.mount_ms, (unsigned)i.last_download_frames);
  return true;
}

bool cmd_erase(const char*) {
  Serial.println(device::console_erase() ? "erasing (see log)" : "cannot erase now (recording/download/erase active)");
  return true;
}

bool cmd_sim(const char* args) {
  if (strcmp(args, "on") == 0) {
    sim_rt::set_enabled(true);
    return true;
  }
  if (strcmp(args, "off") == 0) {
    sim_rt::set_enabled(false);
    return true;
  }
  sim::TrackConfig c = sim_rt::track();
  if (strncmp(args, "radius ", 7) == 0) {
    const float v = strtof(args + 7, nullptr);
    if (v < 10 || v > 5000) return false;
    c.radius_m = v;
  } else if (strncmp(args, "speed ", 6) == 0) {
    const float v = strtof(args + 6, nullptr);
    if (v < 1 || v > 100) return false;
    c.speed_mps = v;
  } else if (*args == 0) {
    Serial.printf("sim %s: r=%.0f m v=%.1f m/s lap=%.2f s generated=%u dropped=%u\n",
                  sim_rt::active() ? "ON" : "off", c.radius_m, c.speed_mps,
                  sim::TrackSim(c).lap_seconds(), (unsigned)sim_rt::generated(), (unsigned)sim_rt::dropped());
    return true;
  } else {
    return false;
  }
  sim_rt::set_track(c);
  Serial.println("track updated (applies immediately)");
  return true;
}

bool cmd_rate(const char* args) {
  char* end = nullptr;
  const long v = strtol(args, &end, 10);
  if (end == args || *end || !rb::valid_rate(static_cast<uint8_t>(v)) || v < 0) return false;
  device::set_default_live_rate(static_cast<uint8_t>(v));
  Serial.printf("live rate (no recording) = code %ld (%u Hz)%s\n", v, rb::rate_hz(static_cast<uint8_t>(v)),
                device::recording() ? " - recording active: live follows the recording rate" : "");
  return true;
}

bool cmd_maxrate(const char* args) {
  char* end = nullptr;
  const long v = strtol(args, &end, 10);
  if (end == args || *end) return false;
  if (v != 0 && rb::rate_code_for_hz(static_cast<uint8_t>(v)) == 0xFF) return false;
  settings::set_max_rate_hz(static_cast<uint8_t>(v));
  gnss::request_cap(static_cast<uint8_t>(v));
  Serial.printf("max_rate = %ld Hz (0 = no cap), saved\n", v);
  return true;
}

bool cmd_log(const char* args) {
  char what[8] = {};
  const char* sp = strchr(args, ' ');
  if (!sp || sp - args >= (long)sizeof what) return false;
  memcpy(what, args, sp - args);
  bool on;
  if (!parse_onoff(sp + 1, &on)) return false;
  if (strcmp(what, "pvt") == 0) gnss::set_log_pvt(on);
  else if (strcmp(what, "frames") == 0) builder::set_log_frames(on);
  else return false;
  return true;
}

bool cmd_reboot(const char*) {
  Serial.println("rebooting");
  Serial.flush();
  ESP.restart();
  return true;
}

}  // namespace

void register_base() {
  console::add("status", "", cmd_status);
  console::add("ble", "mtu <23..517|0> | disconnect", cmd_ble);
  console::add("gnss info", "", cmd_gnss_info);
  console::add("cal", "static | reset | show", cmd_cal);
  console::add("rec", "start | stop", cmd_rec);
  console::add("mem", "", cmd_mem);
  console::add("erase", "", cmd_erase);
  console::add("sim", "[on | off | radius <m> | speed <m/s>]", cmd_sim);
  console::add("stats", "[reset]", cmd_stats);
  console::add("rate", "<0..4>  (0=25 1=10 2=5 3=1 4=20 Hz)", cmd_rate);
  console::add("maxrate", "<0|1|5|10|20|25>  (Hz cap, 0 = none; NVS)", cmd_maxrate);
  console::add("log", "<pvt|frames> <on|off>", cmd_log);
  console::add("reboot", "", cmd_reboot);
}

}  // namespace console_cmds
