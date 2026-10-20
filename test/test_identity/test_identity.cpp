// Identidade BLE: serial de 10 dígitos e nome anunciado (SPEC §4.1, R1/R5/R15).
#include <unity.h>

#include <cstring>

#include "ble/identity.h"

void setUp() {}
void tearDown() {}

void test_serial_is_10_digits_from_last_4_mac_bytes() {
  const uint8_t mac[6] = {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF};
  char s[11];
  identity::serial_from_mac(mac, s);
  TEST_ASSERT_EQUAL_STRING("0682348015", s);  // 0x28ABCDEF
  const uint8_t max[6] = {0, 0, 0xFF, 0xFF, 0xFF, 0xFF};
  identity::serial_from_mac(max, s);
  TEST_ASSERT_EQUAL_STRING("4294967295", s);
  const uint8_t zero[6] = {0xAA, 0xBB, 0, 0, 0, 1};
  identity::serial_from_mac(zero, s);
  TEST_ASSERT_EQUAL_STRING("0000000001", s);  // zeros à esquerda: sempre 10 dígitos
  TEST_ASSERT_EQUAL_size_t(10, strlen(s));
}

void test_serial_is_deterministic() {
  const uint8_t mac[6] = {1, 2, 3, 4, 5, 6};
  char a[11], b[11];
  identity::serial_from_mac(mac, a);
  identity::serial_from_mac(mac, b);
  TEST_ASSERT_EQUAL_STRING(a, b);
}

void test_name_fits_advertising_and_matches_racevo_prefix() {
  char name[32];
  const size_t n = identity::advertised_name(false, "0682348015", name, sizeof name);
  TEST_ASSERT_EQUAL_STRING("RaceBox Mini S 0682348015", name);
  TEST_ASSERT_EQUAL_size_t(25, n);
  // flags (3 B) + complete local name (2 + 25) = 30 ≤ 31 (SPEC §4.1)
  TEST_ASSERT_TRUE(3 + 2 + n <= 31);
  TEST_ASSERT_EQUAL_INT(0, strncmp(name, "RaceBox Mini S ", 15));
  TEST_ASSERT_EQUAL_STRING("RaceBox Mini S", identity::kModelMiniS);  // sem \0 extra/espaço
  identity::advertised_name(true, "0682348015", name, sizeof name);
  TEST_ASSERT_EQUAL_STRING("GoAwake Logger 0682348015", name);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_serial_is_10_digits_from_last_4_mac_bytes);
  RUN_TEST(test_serial_is_deterministic);
  RUN_TEST(test_name_fits_advertising_and_matches_racevo_prefix);
  return UNITY_END();
}
