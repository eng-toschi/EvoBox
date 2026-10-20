// Calibração de montagem: rotação sintética conhecida → R_dv recuperada (SPEC §6.4, §10.1).
#include <unity.h>

#include <cmath>
#include <initializer_list>

#include "imu/calibration.h"

using cal::Vec3;

void setUp() {}
void tearDown() {}

constexpr float kPi = 3.14159265f;
constexpr float kG = 9.80665f;

struct Rng {
  uint32_t s;
  float uniform() {  // [-1, 1)
    s = s * 1664525u + 1013904223u;
    return static_cast<float>(s >> 8) / 8388608.0f - 1.0f;
  }
};

// R (device → veículo) a partir de roll/pitch/yaw da montagem.
struct Rot {
  float m[9];
  Vec3 to_vehicle(const Vec3& d) const {
    return {m[0] * d.x + m[1] * d.y + m[2] * d.z, m[3] * d.x + m[4] * d.y + m[5] * d.z,
            m[6] * d.x + m[7] * d.y + m[8] * d.z};
  }
  Vec3 to_device(const Vec3& v) const {  // Rᵀ
    return {m[0] * v.x + m[3] * v.y + m[6] * v.z, m[1] * v.x + m[4] * v.y + m[7] * v.z,
            m[2] * v.x + m[5] * v.y + m[8] * v.z};
  }
};

static Rot make_rot(float roll_deg, float pitch_deg, float yaw_deg) {
  const float r = roll_deg * kPi / 180, p = pitch_deg * kPi / 180, y = yaw_deg * kPi / 180;
  const float cr = std::cos(r), sr = std::sin(r), cp = std::cos(p), sp = std::sin(p),
              cy = std::cos(y), sy = std::sin(y);
  // Rz(y)·Ry(p)·Rx(r)
  return Rot{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
              sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
              -sp, cp * sr, cp * cr}};
}

static float row_angle_deg(const float* a, const float* b) {
  const float d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const float c = d > 1 ? 1 : (d < -1 ? -1 : d);
  return std::acos(c) * 180.0f / kPi;
}

static void static_cal(cal::Calibrator& c, const Rot& R, Vec3 bias, Rng& rng, float noise_g = 0.003f) {
  c.static_begin();
  for (int i = 0; i < 120; ++i) {  // 12 s a 10 Hz
    Vec3 a = R.to_device({0, 0, 1});
    a.x += noise_g * rng.uniform();
    a.y += noise_g * rng.uniform();
    a.z += noise_g * rng.uniform();
    Vec3 g = bias;
    g.x += 0.05f * rng.uniform();
    c.static_add(a, g, 0.1f);
  }
}

void test_static_inverted_mount_maps_gravity_to_plus_z_and_removes_bias() {
  for (float roll : {180.0f, 0.0f, 90.0f, -90.0f}) {
    Rng rng{7};
    const Rot R = make_rot(roll, 4.0f, 33.0f);
    cal::Calibrator c;
    const Vec3 bias{1.5f, -2.0f, 0.7f};
    static_cal(c, R, bias, rng);
    const cal::StaticResult r = c.static_finish();
    TEST_ASSERT_TRUE_MESSAGE(r.ok, r.reason);
    TEST_ASSERT_EQUAL(cal::kStatic, c.level());
    Vec3 av, gv;
    c.apply(R.to_device({0, 0, 1}), bias, &av, &gv);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.0f, av.x);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.0f, av.y);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 1.0f, av.z);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, gv.x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, gv.y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, gv.z);
    // eixo vertical recuperado
    TEST_ASSERT_TRUE(row_angle_deg(c.blob().r + 6, R.m + 6) < 0.5f);
  }
}

void test_static_rejects_short_moving_or_not_1g() {
  Rng rng{1};
  const Rot R = make_rot(180, 0, 0);
  cal::Calibrator c;
  c.static_begin();
  for (int i = 0; i < 50; ++i) c.static_add(R.to_device({0, 0, 1}), {}, 0.1f);  // 5 s
  TEST_ASSERT_FALSE(c.static_finish().ok);

  c.static_begin();
  for (int i = 0; i < 120; ++i) {
    Vec3 a = R.to_device({0.3f * rng.uniform(), 0, 1});  // mexendo
    c.static_add(a, {}, 0.1f);
  }
  TEST_ASSERT_FALSE(c.static_finish().ok);

  c.static_begin();
  for (int i = 0; i < 120; ++i) c.static_add({0, 0, 0.5f}, {}, 0.1f);  // 0,5 g
  TEST_ASSERT_FALSE(c.static_finish().ok);
  TEST_ASSERT_EQUAL(cal::kNone, c.level());
}

// Dirige em linha reta acelerando e freando; IMU coerente com a rotação real.
static void drive_straight(cal::Calibrator& c, const Rot& R, const Vec3& bias, Rng& rng, int epochs,
                           float yaw_rate_dps = 0.0f) {
  float speed = 8.0f;
  for (int k = 0; k < epochs; ++k) {
    const float a_long = 0.35f * std::sin(k * 0.05f);  // g, acelera e freia
    const float new_speed = speed + a_long * kG * 0.1f;
    const float s = new_speed < 6.0f ? 6.0f : new_speed;
    const float a_eff = (s - speed) / 0.1f / kG;  // coerente com o GNSS
    speed = s;
    Vec3 a = R.to_device({a_eff, 0, 1});
    a.x += 0.01f * rng.uniform();
    a.y += 0.01f * rng.uniform();
    a.z += 0.01f * rng.uniform();
    Vec3 g = R.to_device({0, 0, yaw_rate_dps});
    g.x += bias.x;
    g.y += bias.y;
    g.z += bias.z;
    c.epoch_update(a, g, speed, true, 0.1f);
  }
}

void test_dynamic_recovers_full_rotation_with_correct_sign() {
  const float yaws[] = {0.0f, 37.0f, 180.0f, -120.0f, 90.0f};
  const float rolls[] = {180.0f, 0.0f, 180.0f, 10.0f, -170.0f};
  for (int i = 0; i < 5; ++i) {
    Rng rng{static_cast<uint32_t>(100 + i)};
    const Rot R = make_rot(rolls[i], 3.0f, yaws[i]);
    const Vec3 bias{0.8f, -0.4f, 0.3f};
    cal::Calibrator c;
    static_cal(c, R, bias, rng);
    TEST_ASSERT_TRUE(c.static_finish().ok);
    drive_straight(c, R, bias, rng, 600);
    TEST_ASSERT_TRUE(c.dynamic_stats().converged);
    TEST_ASSERT_EQUAL(cal::kDynamic, c.level());
    TEST_ASSERT_TRUE_MESSAGE(row_angle_deg(c.blob().r + 0, R.m + 0) < 1.5f, "forward axis");
    TEST_ASSERT_TRUE_MESSAGE(row_angle_deg(c.blob().r + 3, R.m + 3) < 1.5f, "left axis");
    TEST_ASSERT_TRUE_MESSAGE(row_angle_deg(c.blob().r + 6, R.m + 6) < 1.0f, "up axis");
    // Sinal: acelerar → +x do veículo; curva à esquerda → +y (FLU).
    Vec3 av, gv;
    c.apply(R.to_device({0.5f, 0.2f, 1.0f}), bias, &av, &gv);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.5f, av.x);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.2f, av.y);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 1.0f, av.z);
  }
}

void test_dynamic_ignores_turns_and_slow_or_no_fix() {
  Rng rng{5};
  const Rot R = make_rot(180, 0, 45);
  const Vec3 bias{};
  cal::Calibrator c;
  static_cal(c, R, bias, rng);
  TEST_ASSERT_TRUE(c.static_finish().ok);
  drive_straight(c, R, bias, rng, 400, 20.0f);  // 20 °/s = 0,35 rad/s > 0,10
  TEST_ASSERT_EQUAL_UINT32(0, c.dynamic_stats().samples);
  TEST_ASSERT_FALSE(c.dynamic_stats().converged);
  for (int k = 0; k < 100; ++k) c.epoch_update(R.to_device({0.3f, 0, 1}), {}, 3.0f, true, 0.1f);  // lento
  for (int k = 0; k < 100; ++k) c.epoch_update(R.to_device({0.3f, 0, 1}), {}, 9.0f, false, 0.1f);  // sem fix
  TEST_ASSERT_EQUAL_UINT32(0, c.dynamic_stats().samples);
}

void test_dynamic_needs_static_first() {
  Rng rng{9};
  const Rot R = make_rot(0, 0, 0);
  cal::Calibrator c;
  drive_straight(c, R, {}, rng, 300);
  TEST_ASSERT_EQUAL(cal::kNone, c.level());
}

void test_gyro_bias_reestimated_at_stops() {
  Rng rng{3};
  const Rot R = make_rot(180, 0, 0);
  cal::Calibrator c;
  static_cal(c, R, {1, 1, 1}, rng);
  TEST_ASSERT_TRUE(c.static_finish().ok);
  const Vec3 drifted{1.4f, 0.6f, 1.2f};
  for (int k = 0; k < 49; ++k) c.epoch_update(R.to_device({0, 0, 1}), drifted, 0.1f, true, 0.1f);
  TEST_ASSERT_EQUAL_UINT32(0, c.dynamic_stats().bias_updates);  // 4,9 s ainda não
  c.epoch_update(R.to_device({0, 0, 1}), drifted, 0.1f, true, 0.1f);
  TEST_ASSERT_EQUAL_UINT32(1, c.dynamic_stats().bias_updates);
  Vec3 av, gv;
  c.apply(R.to_device({0, 0, 1}), drifted, &av, &gv);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, gv.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, gv.z);
  // Parada sem fix não conta.
  for (int k = 0; k < 100; ++k) c.epoch_update(R.to_device({0, 0, 1}), {9, 9, 9}, 0.0f, false, 0.1f);
  TEST_ASSERT_EQUAL_UINT32(1, c.dynamic_stats().bias_updates);
}

void test_blob_round_trip_and_invalid_rejected() {
  Rng rng{11};
  const Rot R = make_rot(180, 2, 70);
  cal::Calibrator c;
  static_cal(c, R, {0.5f, 0, 0}, rng);
  TEST_ASSERT_TRUE(c.static_finish().ok);
  drive_straight(c, R, {0.5f, 0, 0}, rng, 600);
  const cal::Blob b = c.save();
  cal::Calibrator d;
  TEST_ASSERT_TRUE(d.load(b));
  TEST_ASSERT_EQUAL(cal::kDynamic, d.level());
  for (int i = 0; i < 9; ++i) TEST_ASSERT_FLOAT_WITHIN(1e-6f, b.r[i], d.blob().r[i]);
  cal::Blob bad = b;
  bad.magic = 0;
  TEST_ASSERT_FALSE(d.load(bad));
  bad = b;
  bad.r[4] = std::nanf("");
  TEST_ASSERT_FALSE(d.load(bad));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_static_inverted_mount_maps_gravity_to_plus_z_and_removes_bias);
  RUN_TEST(test_static_rejects_short_moving_or_not_1g);
  RUN_TEST(test_dynamic_recovers_full_rotation_with_correct_sign);
  RUN_TEST(test_dynamic_ignores_turns_and_slow_or_no_fix);
  RUN_TEST(test_dynamic_needs_static_first);
  RUN_TEST(test_gyro_bias_reestimated_at_stops);
  RUN_TEST(test_blob_round_trip_and_invalid_rejected);
  return UNITY_END();
}
