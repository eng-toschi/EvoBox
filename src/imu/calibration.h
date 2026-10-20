// Calibração de montagem (SPEC §6.4) — C++ puro, testado em native
// (test_calibration). Resultado: R_dv (device → veículo FLU) + bias do giro.
//
// Frame do veículo: FLU destro (x frente, y esquerda, z cima), força
// específica (parado e nivelado: a = (0, 0, +1 g)). A conversão para os
// sinais do fio fica em rb::AxisConvention (lib/racebox_protocol/axis.h).
//
// 1. Estática (parado ≥ 10 s): média do acelerômetro = gravidade → eixo
//    vertical; média do giro = bias. Frente provisória = eixo X do device
//    projetado no plano horizontal (Y se X estiver quase vertical).
// 2. Dinâmica (em movimento, automática): em linha reta (|yaw| ≤ 0,10 rad/s,
//    speed > 5 m/s, good fix), correlaciona a aceleração horizontal medida
//    com d(speed)/dt do GNSS → ângulo da frente no plano horizontal, com o
//    sinal certo por construção (acelerar = +frente).
// 3. Bias do giro reestimado a cada parada > 5 s (good fix, speed < 0,3 m/s).
#pragma once

#include <cstddef>
#include <cstdint>

namespace cal {

struct Vec3 {
  float x = 0, y = 0, z = 0;
};

// Linhas = eixos do veículo (frente, esquerda, cima) expressos no device.
struct Mat3 {
  float m[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  Vec3 row(int i) const { return {m[3 * i], m[3 * i + 1], m[3 * i + 2]}; }
  Vec3 mul(const Vec3& v) const;
};

enum Level : uint8_t { kNone = 0, kStatic = 1, kDynamic = 2 };

// Blob persistido em NVS.
struct Blob {
  uint32_t magic = 0x45564331;  // "EVC1"
  uint8_t level = kNone;
  uint8_t reserved[3] = {};
  float r[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  float gyro_bias[3] = {};
  float up[3] = {0, 0, 1};
  float heading_rad = 0;  // ângulo da frente a partir da frente provisória
};

struct StaticResult {
  bool ok = false;
  const char* reason = "";
  Vec3 accel_mean_g;
  Vec3 gyro_mean_dps;
  float accel_std_g = 0;
  float gyro_std_dps = 0;
  float gravity_g = 0;  // |média do acelerômetro|
  uint32_t samples = 0;
};

struct DynamicStats {
  uint32_t samples = 0;     // épocas aceitas pelo filtro de linha reta
  float s_aa = 0;           // Σ a_long² (g²)
  float corr = 0;           // correlação |Σ a·h| / sqrt(Σa² Σ|h|²)
  float heading_deg = 0;    // estimativa corrente
  bool converged = false;
  uint32_t bias_updates = 0;
};

class Calibrator {
 public:
  // Limiares (SPEC §6.4).
  static constexpr float kStaticMinSeconds = 10.0f;
  static constexpr float kStillAccelStdG = 0.02f;
  static constexpr float kStillGyroStdDps = 1.0f;
  static constexpr float kDynMaxYawRadS = 0.10f;
  static constexpr float kDynMinSpeedMps = 5.0f;
  static constexpr float kDynMinSaa = 10.0f;   // g²·épocas para convergir
  static constexpr float kDynMinCorr = 0.80f;
  static constexpr float kBiasStopSeconds = 5.0f;
  static constexpr float kBiasMaxSpeedMps = 0.3f;

  Calibrator() { reset(); }
  void reset();
  bool load(const Blob& b);  // false se o blob for inválido
  Blob save() const;
  Level level() const { return static_cast<Level>(blob_.level); }
  const Blob& blob() const { return blob_; }

  // --- Estática: amostras (médias de janela) do device parado. ---
  void static_begin();
  void static_add(const Vec3& acc_g, const Vec3& gyro_dps, float dt_s);
  float static_seconds() const { return st_seconds_; }
  StaticResult static_finish();  // aplica se ok

  // --- Aplicação: device → veículo FLU (g, °/s), bias removido. ---
  void apply(const Vec3& acc_dev_g, const Vec3& gyro_dev_dps, Vec3* acc_v, Vec3* gyro_v) const;

  // --- Dinâmica + bias em paradas, uma chamada por época GNSS. ---
  // speed_mps = gSpeed do GNSS; dt_s = intervalo desde a época anterior.
  // Retorna true se a calibração mudou (convergiu ou bias atualizado) e
  // deve ser persistida.
  bool epoch_update(const Vec3& acc_dev_g, const Vec3& gyro_dev_dps, float speed_mps, bool good_fix,
                    float dt_s);
  const DynamicStats& dynamic_stats() const { return dyn_; }

 private:
  void rebuild();  // R a partir de up + heading

  Blob blob_;
  // estática
  double st_sum_a_[3], st_sum_a2_[3], st_sum_g_[3], st_sum_g2_[3];
  uint32_t st_n_ = 0;
  float st_seconds_ = 0;
  // dinâmica
  DynamicStats dyn_;
  double s_ax_ = 0, s_ay_ = 0, s_hh_ = 0;
  bool have_prev_speed_ = false;
  float prev_speed_ = 0;
  // bias em parada
  float stop_seconds_ = 0;
  double stop_sum_g_[3] = {};
  uint32_t stop_n_ = 0;
};

}  // namespace cal
