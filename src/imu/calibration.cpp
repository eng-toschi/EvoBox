#include "imu/calibration.h"

#include <cmath>
#include <cstring>

namespace cal {
namespace {

constexpr float kG = 9.80665f;
constexpr float kDegPerRad = 57.29578f;
constexpr float kDtEps = 1e-3f;  // tolerância de soma de dt em float (50 × 0,1f = 4,9999995)

float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
Vec3 scale(const Vec3& a, float k) { return {a.x * k, a.y * k, a.z * k}; }
Vec3 sub(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 add(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
float norm(const Vec3& a) { return std::sqrt(dot(a, a)); }
Vec3 unit(const Vec3& a) {
  const float n = norm(a);
  return n > 0 ? scale(a, 1.0f / n) : Vec3{0, 0, 1};
}

// Frente provisória: eixo X do device no plano horizontal (Y se X ~ vertical).
void provisional_axes(const Vec3& up, Vec3* f0, Vec3* l0) {
  Vec3 d{1, 0, 0};
  if (std::fabs(dot(d, up)) > 0.9f) d = Vec3{0, 1, 0};
  *f0 = unit(sub(d, scale(up, dot(d, up))));
  *l0 = cross(up, *f0);
}

}  // namespace

Vec3 Mat3::mul(const Vec3& v) const {
  return {m[0] * v.x + m[1] * v.y + m[2] * v.z, m[3] * v.x + m[4] * v.y + m[5] * v.z,
          m[6] * v.x + m[7] * v.y + m[8] * v.z};
}

void Calibrator::reset() {
  blob_ = Blob{};
  dyn_ = DynamicStats{};
  s_ax_ = s_ay_ = s_hh_ = 0;
  have_prev_speed_ = false;
  stop_seconds_ = 0;
  stop_n_ = 0;
  std::memset(stop_sum_g_, 0, sizeof stop_sum_g_);
  static_begin();
}

bool Calibrator::load(const Blob& b) {
  if (b.magic != Blob{}.magic || b.level > kDynamic) return false;
  for (float v : b.r)
    if (!std::isfinite(v)) return false;
  reset();
  blob_ = b;
  if (blob_.level >= kStatic) rebuild();
  dyn_.converged = blob_.level == kDynamic;
  dyn_.heading_deg = blob_.heading_rad * kDegPerRad;
  return true;
}

Blob Calibrator::save() const { return blob_; }

void Calibrator::rebuild() {
  const Vec3 up = unit({blob_.up[0], blob_.up[1], blob_.up[2]});
  Vec3 f0, l0;
  provisional_axes(up, &f0, &l0);
  const float c = std::cos(blob_.heading_rad), s = std::sin(blob_.heading_rad);
  const Vec3 f = add(scale(f0, c), scale(l0, s));
  const Vec3 l = cross(up, f);
  const float r[9] = {f.x, f.y, f.z, l.x, l.y, l.z, up.x, up.y, up.z};
  std::memcpy(blob_.r, r, sizeof r);
}

void Calibrator::static_begin() {
  std::memset(st_sum_a_, 0, sizeof st_sum_a_);
  std::memset(st_sum_a2_, 0, sizeof st_sum_a2_);
  std::memset(st_sum_g_, 0, sizeof st_sum_g_);
  std::memset(st_sum_g2_, 0, sizeof st_sum_g2_);
  st_n_ = 0;
  st_seconds_ = 0;
}

void Calibrator::static_add(const Vec3& a, const Vec3& g, float dt_s) {
  const float av[3] = {a.x, a.y, a.z}, gv[3] = {g.x, g.y, g.z};
  for (int i = 0; i < 3; ++i) {
    st_sum_a_[i] += av[i];
    st_sum_a2_[i] += static_cast<double>(av[i]) * av[i];
    st_sum_g_[i] += gv[i];
    st_sum_g2_[i] += static_cast<double>(gv[i]) * gv[i];
  }
  ++st_n_;
  st_seconds_ += dt_s;
}

StaticResult Calibrator::static_finish() {
  StaticResult r;
  r.samples = st_n_;
  if (st_n_ < 2 || st_seconds_ + kDtEps < kStaticMinSeconds) {
    r.reason = "need >= 10 s of samples";
    return r;
  }
  double ma[3], mg[3], va = 0, vg = 0;
  for (int i = 0; i < 3; ++i) {
    ma[i] = st_sum_a_[i] / st_n_;
    mg[i] = st_sum_g_[i] / st_n_;
    const double sa = st_sum_a2_[i] / st_n_ - ma[i] * ma[i];
    const double sg = st_sum_g2_[i] / st_n_ - mg[i] * mg[i];
    va += sa > 0 ? sa : 0;
    vg += sg > 0 ? sg : 0;
  }
  r.accel_mean_g = {static_cast<float>(ma[0]), static_cast<float>(ma[1]), static_cast<float>(ma[2])};
  r.gyro_mean_dps = {static_cast<float>(mg[0]), static_cast<float>(mg[1]), static_cast<float>(mg[2])};
  r.accel_std_g = static_cast<float>(std::sqrt(va));
  r.gyro_std_dps = static_cast<float>(std::sqrt(vg));
  r.gravity_g = norm(r.accel_mean_g);
  if (r.accel_std_g > kStillAccelStdG || r.gyro_std_dps > kStillGyroStdDps) {
    r.reason = "device not still";
    return r;
  }
  if (r.gravity_g < 0.8f || r.gravity_g > 1.2f) {
    r.reason = "|accel| not ~1 g";
    return r;
  }
  const Vec3 up = unit(r.accel_mean_g);
  blob_.up[0] = up.x;
  blob_.up[1] = up.y;
  blob_.up[2] = up.z;
  blob_.gyro_bias[0] = r.gyro_mean_dps.x;
  blob_.gyro_bias[1] = r.gyro_mean_dps.y;
  blob_.gyro_bias[2] = r.gyro_mean_dps.z;
  blob_.heading_rad = 0;  // nova montagem: a dinâmica recomeça
  blob_.level = kStatic;
  rebuild();
  dyn_ = DynamicStats{};
  s_ax_ = s_ay_ = s_hh_ = 0;
  r.ok = true;
  r.reason = "ok";
  return r;
}

void Calibrator::apply(const Vec3& acc, const Vec3& gyro, Vec3* acc_v, Vec3* gyro_v) const {
  Mat3 R;
  std::memcpy(R.m, blob_.r, sizeof R.m);
  *acc_v = R.mul(acc);
  *gyro_v = R.mul(sub(gyro, {blob_.gyro_bias[0], blob_.gyro_bias[1], blob_.gyro_bias[2]}));
}

bool Calibrator::epoch_update(const Vec3& acc, const Vec3& gyro, float speed, bool good_fix, float dt_s) {
  bool changed = false;
  const bool dt_ok = dt_s > 0.0f && dt_s <= 1.5f;

  // Bias do giro em paradas (RAM; não persiste a cada parada — desgaste de NVS).
  if (good_fix && speed < kBiasMaxSpeedMps && dt_ok) {
    stop_seconds_ += dt_s;
    stop_sum_g_[0] += gyro.x;
    stop_sum_g_[1] += gyro.y;
    stop_sum_g_[2] += gyro.z;
    ++stop_n_;
    if (stop_seconds_ + kDtEps >= kBiasStopSeconds) {
      for (int i = 0; i < 3; ++i) blob_.gyro_bias[i] = static_cast<float>(stop_sum_g_[i] / stop_n_);
      ++dyn_.bias_updates;
      stop_seconds_ = 0;
      stop_n_ = 0;
      std::memset(stop_sum_g_, 0, sizeof stop_sum_g_);
    }
  } else {
    stop_seconds_ = 0;
    stop_n_ = 0;
    std::memset(stop_sum_g_, 0, sizeof stop_sum_g_);
  }

  const bool had_prev = have_prev_speed_;
  const float a_long_g = had_prev && dt_ok ? (speed - prev_speed_) / dt_s / kG : 0.0f;
  prev_speed_ = speed;
  have_prev_speed_ = good_fix && dt_ok;
  if (blob_.level < kStatic || !had_prev || !good_fix || !dt_ok) return changed;

  Vec3 acc_v, gyro_v;
  apply(acc, gyro, &acc_v, &gyro_v);
  const float yaw_rad = gyro_v.z / kDegPerRad;
  if (speed <= kDynMinSpeedMps || std::fabs(yaw_rad) > kDynMaxYawRadS) return changed;

  const Vec3 up = unit({blob_.up[0], blob_.up[1], blob_.up[2]});
  Vec3 f0, l0;
  provisional_axes(up, &f0, &l0);
  const float hx = dot(f0, acc), hy = dot(l0, acc);
  s_ax_ += static_cast<double>(a_long_g) * hx;
  s_ay_ += static_cast<double>(a_long_g) * hy;
  dyn_.s_aa += a_long_g * a_long_g;
  s_hh_ += static_cast<double>(hx) * hx + static_cast<double>(hy) * hy;
  ++dyn_.samples;
  const double den = std::sqrt(static_cast<double>(dyn_.s_aa) * s_hh_);
  dyn_.corr = den > 0 ? static_cast<float>(std::sqrt(s_ax_ * s_ax_ + s_ay_ * s_ay_) / den) : 0.0f;
  const float heading = static_cast<float>(std::atan2(s_ay_, s_ax_));
  dyn_.heading_deg = heading * kDegPerRad;

  if (dyn_.s_aa >= kDynMinSaa && dyn_.corr >= kDynMinCorr) {
    float diff = std::fabs(heading - blob_.heading_rad);
    if (diff > 3.14159265f) diff = 2 * 3.14159265f - diff;
    const bool first = !dyn_.converged;
    // Depois de convergir só atualiza com evidência forte de mudança (> 2°).
    if (first || (diff * kDegPerRad > 2.0f && dyn_.corr >= 0.9f && dyn_.s_aa >= 3 * kDynMinSaa)) {
      blob_.heading_rad = heading;
      blob_.level = kDynamic;
      rebuild();
      dyn_.converged = true;
      changed = true;
    }
  }
  return changed;
}

}  // namespace cal
