// Convenção de eixos do fio (SPEC §6.4). Os seis sinais ficam isolados aqui.
//
// O firmware calcula o movimento no frame do veículo FLU, destro:
//   x = frente, y = esquerda, z = cima; acelerômetro = força específica
//   (parado e nivelado: z = +1 g); giro pela regra da mão direita.
// O fio do RaceBox (vendor p.6–7, Racevo g_ball.dart): gX + acelerando,
// gY + curva à direita, gZ ≈ +1 g parado; rot X = roll, Y = pitch, Z = yaw.
// Os sinais do giro são ‹VALIDAR› (SPEC §13 D5, P3): a hipótese é a regra da
// mão direita sobre os eixos do fio (frente, direita, cima).
#pragma once

#include <cstdint>

namespace rb {

struct AxisConvention {
  int8_t acc[3];  // sinal aplicado a FLU x, y, z → fio gX, gY, gZ
  int8_t gyr[3];  // sinal aplicado a FLU ωx, ωy, ωz → fio rotX, rotY, rotZ
};

constexpr AxisConvention kWireFromFlu = {{+1, -1, +1}, {+1, -1, +1}};

}  // namespace rb
