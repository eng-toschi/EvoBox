// Comandos Mini S (SPEC §7) — C++ puro, testado em native (test_commands).
// Decide ACK/NACK e respostas; a execução fica atrás da interface Device.
#pragma once

#include <cstddef>
#include <cstdint>

#include "frame.h"
#include "messages.h"

namespace cmd {

class Device {
 public:
  virtual ~Device() = default;
  virtual bool recording() const = 0;
  virtual rb::RecordingConfig rec_config() const = 0;  // config corrente (persistida)
  virtual rb::RecordingStatus status() const = 0;
  virtual bool can_start_recording() const = 0;  // memória com espaço
  virtual bool download_active() const = 0;
  virtual bool erase_active() const = 0;
  virtual void start_recording(const rb::RecordingConfig& c) = 0;
  virtual void reconfigure_recording(const rb::RecordingConfig& c) = 0;
  virtual void stop_recording() = 0;
  // Download em duas fases: snapshot (silencia o live) → resposta 0x23 no
  // stream → início do envio. false = não pode começar (NACK).
  virtual bool prepare_download(uint32_t* expected) = 0;
  virtual void begin_download() = 0;
  virtual void cancel_download() = 0;  // o dono manda o ACK ao terminar de drenar
  virtual bool start_erase() = 0;      // progresso e ACK final pelo dono
  virtual void cancel_erase() = 0;     // idem
  virtual rb::GnssConfig gnss_config() const = 0;
  virtual void set_gnss_config(const rb::GnssConfig& c) = 0;
};

class Output {
 public:
  virtual ~Output() = default;
  virtual void response(const uint8_t* frame, size_t len) = 0;  // canal de respostas
  virtual void bulk(const uint8_t* frame, size_t len) = 0;      // stream ordenado do download
};

struct Stats {
  uint32_t handled = 0;
  uint32_t acks = 0;
  uint32_t nacks = 0;
  uint32_t ignored_foreign_class = 0;  // UBX cru do cliente nunca é aceito (SPEC §7.6)
  uint32_t idempotent_25 = 0;
};

class Processor {
 public:
  Processor(Device& d, Output& o) : dev_(d), out_(o) {}
  void handle(const rb::Frame& f);
  const Stats& stats() const { return stats_; }

 private:
  void ack(uint8_t id, bool positive);
  void reply(uint8_t id, const uint8_t* payload, size_t len);
  void on_status(const rb::Frame& f);
  void on_rec_config(const rb::Frame& f);
  void on_download(const rb::Frame& f);
  void on_erase(const rb::Frame& f);
  void on_gnss_config(const rb::Frame& f);

  Device& dev_;
  Output& out_;
  Stats stats_;
};

}  // namespace cmd
