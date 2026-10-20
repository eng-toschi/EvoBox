# Night log 2026-10-19

## Resumo (5 linhas no máximo)
- F0 ✅: `lib/racebox_protocol` + suíte native verde (conformidade byte a byte com os golden vectors do Racevo).
- F1 ✅ (com ressalva): ⚠️ **o GPS da placa não é NEO-M8N, é u-blox M10** (SPG 5.10) — detectado, tratado como M10 a 10 Hz (D16).
- Em andamento: F2.

## Fases

### F0 — status: ✅
- Golden live frame (campos decodificados → bytes idênticos a `B5 62 FF 01 50 00 … 06 DB`) → **provado** — `test_conformance` (`test_golden_live_frame_built_from_decoded_fields_is_byte_identical`); também pelo caminho NAV-PVT → payload (`test_payload/test_navpvt_path_reproduces_golden_frame`).
- Checksum vectors (todos os 10 do fixture validam; cada byte adulterado é rejeitado) → **provado** — `test_checksum_vectors_*`, `test_live_frame_tampered_is_rejected`.
- Respostas de comando (`0x23` 780, ACK do unlock, preset A, erase 59 %, unlock 123456: parse + re-encode idênticos; `0x22`/`0x26` round-trip) → **provado** — `test_conformance`. Extra: os exemplos em bytes do PDF do vendor para `0x22`, `0x26` e `0x27` (que o fixture não tem) decodificam e re-encodam idênticos; o checksum de cada um prova a transcrição.
- Assembler (todos os pontos de corte, aleatório com seed fixa, concatenação, gotejamento 1 B, ruído, `B5 62` dentro do payload, corrupção, len > 504, overflow limitado, nunca trava) → **provado** — `test_assembler` (15 casos).
- Clamp (gyro 400 °/s → `+32767`, nunca negativo; bytes `FF 7F` no fio) → **provado** — `test_clamp_gyro_400dps_*`, `test_motion_clamp_never_wraps`.
- CI: `.github/workflows/native.yml` roda `pio test -e native` em push/PR → resultado do primeiro run: ver seção "Como reproduzir".
- Fixture conferido byte a byte contra `racevo/docs/fixtures/racebox_test_vectors.json` no commit `033bcab` (idêntico, intocado).

### F1 — status: ✅ (A10 provado no mecanismo; ressalva: o módulo é M10, não M8N)
- Detecção do módulo por `UBX-MON-VER` → **provado em hardware** — console: `found M10 (sw="ROM SPG 5.10 (7b202e)" hw=000A0000 fw="SPG 5.10" mod="" protver=34.10)`. ⚠️ Divergência da SPEC §2.1: a placa foi montada como NEO-M8N, mas o chip é M10. Regra de detecção corrigida (hwVersion `000A0000` é comum a M9 e M10 → decide por `PROTVER`); teste native com as strings reais (`test_mon_ver_m10_as_seen_on_bench`).
- Config UBX (só NAV-PVT, NMEA desligado, dynModel 4, baud 115200, salvo em BBR) → **provado** — log `configured: NAV-PVT only, dynModel=4, GNSS=GPS+BDS+GAL, saved to module`; no reboot seguinte o receptor já responde em 115200 com NAV-PVT.
- **A10** (rate 25 pedido → entrega 10 Hz e loga) → **provado** (mecanismo, neste M10 com teto de 10 Hz) — log `rate 25 Hz requested (code 0) > limit (module 10 Hz, cap 0 Hz): LIMITED to 10 Hz (100 ms)`; `stats`: 150 Δ`iTOW` seguidos = 100 ms, `gaps=0`, `dropped=0`. Não foi possível provar num NEO-M8N verdadeiro (não existe na bancada).
- Outras taxas e teto `maxrate` → **provado** — 5 Hz (200 ms), 1 Hz (1000 ms), `maxrate 5` limita 10 → 5 Hz com log, `maxrate 0` volta a 10 Hz.
- Dado de bancada (sem fix, dentro de casa): com `measRate` 40 ms o M10 entregou 416 NAV-PVT com Δ`iTOW` = 40 ms e zero gaps — não prova 25 Hz com fix; fica `‹VALIDAR›` em campo (D16).
- **D10** (tempo até o 1º NAV-PVT) → **medido**: 53–130 ms com o receptor configurado; 1,96 s no pior caso visto (receptor a 9600/NMEA). Pendente: power-on frio com backup descarregado (precisa desligar a placa da USB).
- Latência NAV-PVT→frame montado: `build_us_max` ≈ 25–230 µs (orçamento SPEC §5.2: ≤ 5 ms).

## Problemas abertos (com o que foi tentado)
- **GPS é M10, não NEO-M8N** (D16). Não é bug do firmware; é a peça. Decisão: teto de 10 Hz até validar 25 Hz com fix em campo.
- Upload a 921600 falhava ("Unable to verify flash chip connection") — medido com esptool: 460800/230400/115200 funcionam. `upload_speed` baixado para 460800.
- Abrir a serial reseta o ESP32 mesmo com DTR/RTS em baixo (CP2102). O `tools/serial_console.py` usa `--boot-wait`.
- Log do core do Arduino no boot: `esp_core_dump_flash: No core dump partition found!` — inofensivo (não há partição de coredump na tabela sem OTA).

## Decisões tomadas (e onde foram registradas na SPEC §13)
- **D14** — `0x25` com 10 B: o campo que falta é o último (auto-shutdown), que mantém o valor atual. SPEC §7.2 + §13.
- **D15** — `0x27` min hAcc em **décimos de metro** (exemplo em bytes do vendor: `0x14` = 2,0 m); default `30`. SPEC §7.6 + §13. ⚠️ diverge do texto antigo da SPEC ("m").
- SPEC §10.1 corrigida: o ACK do unlock é `… 32 3A` (o fixture está certo; `32 32` é typo do PDF).
- **D13** — driver UBX próprio (sem SparkFun): NAV-PVT cru até o builder, comandos/parse testados em native. SPEC §5.4 + §13.
- **D16** — chip M10 detectado: config VALSET, 115200, teto 10 Hz, constelações de fábrica (GPS+GAL+BDS). SPEC §2.1, §2.2 + §13.
- **D10** — medido (ver F1). SPEC §13.
- Min hAcc do `0x27` aplicado no firmware: zera o b0 de Fix Status Flags quando `hAcc` > mínimo (vendor p.10). SPEC §7.6.
- `.gitignore` tinha `*.csv`, que ignorava `partitions.csv` — exceção adicionada.

## O que o Fernando precisa fazer (testes no Racevo, pista, hardware)
- Confirmar a peça GNSS: a etiqueta diz NEO-M8N, o chip responde M10 (SPG 5.10). Decidir se fica (10 Hz) ou se troca por NEO-M9N (D1).
- Em campo, com fix: `rate 0` com o teto do M10 liberado temporariamente para medir se o M10 sustenta 25 Hz (D16).
- D10 a frio: desligar a placa da USB ≥ 1 h (backup do módulo descarregado) e medir o 1º NAV-PVT no console.

## Como reproduzir (comandos)
```bash
pio test -e native        # test_conformance, test_assembler, test_payload, test_ublox
pio run -e esp32dev -t upload
python3 tools/serial_console.py --seconds 8                              # boot + detecção do GNSS
python3 tools/serial_console.py --boot-wait 3 -c "gnss info" -c "stats reset" --wait 0.3 --wait 15 -c stats
```
Observação: a placa apareceu em `/dev/cu.usbserial-110` (não `-220` como na SPEC/CLAUDE.md); o PlatformIO detecta sozinho.
