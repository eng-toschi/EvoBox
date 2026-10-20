#include "app/console.h"

#include <Arduino.h>

#include <cstring>

namespace console {
namespace {

struct Cmd {
  const char* name;
  const char* usage;
  Handler h;
};

constexpr size_t kMaxCmds = 40;
Cmd g_cmds[kMaxCmds];
size_t g_n = 0;
char g_line[96];
size_t g_len = 0;

void help() {
  Serial.println("commands:");
  for (size_t i = 0; i < g_n; ++i) Serial.printf("  %-12s %s\n", g_cmds[i].name, g_cmds[i].usage);
}

void dispatch(char* line) {
  while (*line == ' ') ++line;
  size_t n = strlen(line);
  while (n && (line[n - 1] == ' ')) line[--n] = 0;
  if (n == 0) return;
  if (strcmp(line, "help") == 0) {
    help();
    return;
  }
  // Maior prefixo de comando que casa com a linha ("gnss info" antes de "gnss").
  const Cmd* best = nullptr;
  size_t best_len = 0;
  for (size_t i = 0; i < g_n; ++i) {
    const size_t k = strlen(g_cmds[i].name);
    if (k > best_len && strncmp(line, g_cmds[i].name, k) == 0 && (line[k] == 0 || line[k] == ' ')) {
      best = &g_cmds[i];
      best_len = k;
    }
  }
  if (!best) {
    Serial.printf("unknown command: %s (try 'help')\n", line);
    return;
  }
  const char* args = line + best_len;
  while (*args == ' ') ++args;
  if (!best->h(args)) Serial.printf("usage: %s %s\n", best->name, best->usage);
}

}  // namespace

void add(const char* name, const char* usage, Handler h) {
  if (g_n < kMaxCmds) g_cmds[g_n++] = {name, usage, h};
}

void poll() {
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c < 0) break;
    if (c == '\r' || c == '\n') {
      g_line[g_len] = 0;
      if (g_len) dispatch(g_line);
      g_len = 0;
    } else if (g_len + 1 < sizeof g_line) {
      g_line[g_len++] = static_cast<char>(c);
    }
  }
}

}  // namespace console
