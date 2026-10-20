#include "ble/identity.h"

#include <cstdio>

namespace identity {

void serial_from_mac(const uint8_t mac[6], char out[11]) {
  const uint32_t v = (static_cast<uint32_t>(mac[2]) << 24) | (static_cast<uint32_t>(mac[3]) << 16) |
                     (static_cast<uint32_t>(mac[4]) << 8) | static_cast<uint32_t>(mac[5]);
  std::snprintf(out, 11, "%010lu", static_cast<unsigned long>(v));
}

size_t advertised_name(bool goawake, const char serial[11], char* out, size_t cap) {
  const int n = std::snprintf(out, cap, "%s%s", goawake ? kNamePrefixGoAwake : kNamePrefixMiniS, serial);
  return n < 0 ? 0 : static_cast<size_t>(n);
}

}  // namespace identity
