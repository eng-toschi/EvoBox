// Cola entre os comandos/recorder (puros) e o hardware: storage, GNSS, NVS,
// BLE, energia. Implementa cmd::Device e rec::Sink.
#pragma once

#include <Arduino.h>

#include "app/commands.h"
#include "app/recorder.h"
#include "payload.h"

namespace device {

void start();  // carrega config; retoma a gravação se Enable=1 salvo (SPEC §7.2)

// Sink do builder: cada frame live vai para o recorder.
void on_live(const uint8_t frame[88], const rb::LiveData& d, uint32_t rx_ms);
// Handler de comandos da task BLE.
void on_command(const rb::Frame& f);
// 1 Hz (task power): auto-shutdown sem frames.
void tick();

// Console.
bool rec_start();
void rec_stop();
bool recording();
rec::State rec_state();
rb::RecordingConfig rec_config();
rec::Stats rec_stats();
cmd::Stats cmd_stats();
// Taxa do live sem gravação (console `rate`): 0..4.
void set_default_live_rate(uint8_t code);
uint8_t default_live_rate();
bool console_erase();

// Pedido de desligamento (auto-shutdown) pendente — a task power executa.
bool shutdown_requested();

}  // namespace device
