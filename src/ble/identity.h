// Identidade BLE (SPEC §4.1, CLAUDE.md regra 11). C++ puro — testado em native.
#pragma once

#include <cstddef>
#include <cstdint>

namespace identity {

constexpr const char* kModelMiniS = "RaceBox Mini S";  // DIS Model exato (R5)
constexpr const char* kNamePrefixMiniS = "RaceBox Mini S ";
constexpr const char* kNamePrefixGoAwake = "GoAwake Logger ";

// Serial de 10 dígitos: últimos 4 bytes do MAC de fábrica como uint32
// big-endian (ordem de leitura do MAC), em %010u. Determinístico por hardware.
void serial_from_mac(const uint8_t mac[6], char out[11]);

// Nome anunciado: prefixo + serial. out precisa de ≥ 32 B. Retorna o tamanho.
size_t advertised_name(bool goawake, const char serial[11], char* out, size_t cap);

}  // namespace identity
