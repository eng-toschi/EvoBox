# Night log 2026-10-19

## Resumo (5 linhas no máximo)
- F0–F6 implementadas; 107 testes native verdes. F0, F1, F2, F4, F5 provadas em bancada (com as ressalvas abaixo); F6 provada em bancada no modo simulação (5 voltas de 31,416 s, live + gravação + download).
- ⚠️ **Hardware:** (1) o GPS "NEO-M8N" é um **u-blox M10** (SPG 5.10) → teto 10 Hz (D16); (2) o **MPU6050 não amostra** (responde no I²C, mas acelerômetro/giro/temperatura = 0) → **F3 bloqueada em hardware** (lógica provada em native).
- ⚠️ **Incidente:** um RaceBox Mini S real (serial **2254302541**) estava no alcance e o `rb_client` mandou **uma vez** o preset A (`0x25` Track Day) + `0x22` para ele por engano. Ver "O que o Fernando precisa fazer". Ferramenta corrigida.
- Faltam o celular com o Racevo (A1, A7/A9/A11–A14 no app) e a pista (F7).

## Fases

### F0 — status: ✅
- Golden live frame (campos decodificados → bytes idênticos a `B5 62 FF 01 50 00 … 06 DB`) → **provado** — `test_conformance` (`test_golden_live_frame_built_from_decoded_fields_is_byte_identical`); também pelo caminho NAV-PVT → payload (`test_payload/test_navpvt_path_reproduces_golden_frame`).
- Checksum vectors (os 10 do fixture validam; cada byte adulterado é rejeitado) → **provado** — `test_checksum_vectors_*`, `test_live_frame_tampered_is_rejected`.
- Respostas de comando (`0x23` 780, ACK do unlock, preset A, erase 59 %, unlock 123456: parse + re-encode idênticos; `0x22`/`0x26` round-trip) → **provado** — `test_conformance`. Extra: os exemplos em bytes do PDF do vendor para `0x22`, `0x26` e `0x27` (que o fixture não tem) decodificam e re-encodam idênticos; o checksum de cada um prova a transcrição.
- Assembler (todos os pontos de corte, aleatório com seed fixa, concatenação, gotejamento 1 B, ruído, `B5 62` dentro do payload, corrupção, len > 504, overflow limitado, nunca trava) → **provado** — `test_assembler` (15 casos).
- Clamp (gyro 400 °/s → `+32767`, nunca negativo; bytes `FF 7F` no fio) → **provado** — `test_clamp_gyro_400dps_*`, `test_motion_clamp_never_wraps`.
- CI (`.github/workflows/native.yml`, `pio test -e native` em push/PR) → **provado** — runs verdes no GitHub Actions na branch.
- Fixture conferido byte a byte contra `racevo/docs/fixtures/racebox_test_vectors.json` no commit `033bcab` (idêntico, intocado).

### F1 — status: ✅ (ressalva: o módulo é M10, não M8N)
- Detecção do módulo por `UBX-MON-VER` → **provado em hardware** — `found M10 (sw="ROM SPG 5.10 (7b202e)" hw=000A0000 fw="SPG 5.10" mod="" protver=34.10)`. ⚠️ Divergência da SPEC §2.1 (placa montada como NEO-M8N; o chip é M10). Regra de detecção corrigida (hwVersion `000A0000` é comum a M9 e M10 → decide por `PROTVER`); teste native com as strings reais (`test_mon_ver_m10_as_seen_on_bench`).
- Config UBX (só NAV-PVT, NMEA desligado, dynModel 4, 115200, salvo em BBR) → **provado** — log `configured: NAV-PVT only…saved to module`; no reboot seguinte o receptor já responde em 115200.
- **A10** (rate 25 pedido → entrega 10 Hz e loga) → **provado** (mecanismo, neste M10 com teto de 10 Hz) — log `rate 25 Hz requested (code 0) > limit (module 10 Hz, cap 0 Hz): LIMITED to 10 Hz (100 ms)`; 150 Δ`iTOW` seguidos = 100 ms. Não há NEO-M8N verdadeiro na bancada.
- Outras taxas e `maxrate` → **provado** — 5 Hz, 1 Hz, `maxrate 5` limita 10 → 5 Hz com log, `maxrate 0` volta.
- Dado de bancada (sem fix): com `measRate` 40 ms o M10 entregou 416 NAV-PVT com Δ`iTOW` = 40 ms e zero gaps — **não** prova 25 Hz com fix (D16, `‹VALIDAR›` em campo).
- **D10** → **medido**: 1º NAV-PVT a 53–130 ms do boot (receptor configurado); 1,96 s no pior caso visto (receptor a 9600/NMEA).
- Latência NAV-PVT→frame: `build_us_max` 25–2700 µs, `latency_ms_max` ≤ 5 ms (orçamento SPEC §5.2: ≤ 5 ms). O pico de ~2,7–4,7 ms aparece com gravação ativa (escrita na flash pausa o cache dos dois cores).

### F2 — status: 🟡 (A2–A5 provados com o bleak no Mac; A1 precisa do celular)
- Identidade: nome `RaceBox Mini S 0418890644` no advertising, UUID do UART no scan response → **provado** — `rb_client scan`.
- DIS exato → **provado** — `rb_client info`: `'RaceBox Mini S'`, `'0418890644'`, `'3.3'`, `'GA-ESP32-M10'`, `'GoAwake'` → classificação Racevo `miniS`. Bug achado e corrigido em bancada: o `setValue(const T&)` templado do NimBLE 1.4 gravava os bytes do **ponteiro** (`'N\t@?'`) e o `\0` final dos `char[N]`.
- **A1** (Racevo encontra/conecta/classifica Mini S) → **pendente** — precisa do celular com o Racevo. Tudo que o Racevo checa (nome, UUID, DIS exato) foi provado com o bleak.
- **A2** (1º frame < 1 s após subscribe) → **provado** — 49–90 ms (cliente), 6–66 ms (device).
- **A3** (Δ`iTOW` exato, zero gaps em 30 min) → **provado a 10 Hz** — 30 min, 18000 frames, 17997/17999 Δ = 100 ms, **0 gaps, 0 não-monotônicos, 0 erros de assembler**. Ressalva: essa corrida foi com **MTU 23** (o teto do teste A5 ficou em RAM) — condição mais dura (90000 notifies fragmentados). Corrida com MTU 517 + gravação simultânea: ver "Resultado A3 final" abaixo. O único par 99/101 ms é o iTOW cru do receptor sem fix (as estatísticas da UART mostram o mesmo); o firmware não mexe. 25 Hz: só no modo sim (4249/4249 Δ = 40 ms, 0 gaps); 25 Hz real exige M9N (F7).
- **A4** (comandos < 500 ms) → **provado** — 27–74 ms em todos os comandos (`0x22`, `0x25`, `0x27`, `0x30`, `0x23`, `0x24`).
- **A5** (MTU 23 sem perda) → **provado** — `ble mtu 23` no device; 200 frames em 1000 notifies de 20 B, 0 bad, 0 drops; e os 30 min acima.

### F3 — status: ❌ bloqueada em hardware (lógica ✅ em native)
- Calibração (rotação sintética conhecida → R_dv recuperada, montagem invertida, sinal certo por construção, bias em parada, rejeições) → **provado em native** — `test_calibration` (7 casos, 5 montagens diferentes, erro < 1,5°).
- FIFO 1 kHz / média por época / **A6** → **falhou** — o MPU6050 (WHO_AM_I `0x68`) aceita escrita/leitura de registros, mas **nenhum ADC roda**: acel, giro e temperatura = 0, `DATA_RDY` nunca sobe, FIFO vazio. Ver "Problemas abertos". O firmware cai no comportamento da SPEC (janela vazia → repete o último valor, `imu_stall`, log a cada 5 s); sem nenhuma amostra o "último valor" é o padrão da F2 (0, 0, +1000 mg).

### F4 — status: 🟡 (comandos provados em bancada com o bleak; walkthrough no Racevo pendente)
- `0x22`/`0x25` ler/escrever (12 B e 10 B)/`0x27`/`0x30`/`0x23`/`0x24`, NACK para id desconhecido, UBX cru ignorado → **provado em native** — `test_commands` (14 casos, entradas dos golden vectors) — e **em bancada** (respostas e latências acima).
- **Idempotência do `0x25` (R11/R13)** → **provado em bancada** — preset A por 5 conexões seguidas: um único `recording started`, nenhuma reconfiguração de taxa no log, download com só `state=1`/`state=0` (1 run).
- Preset B com a gravação ativa (mesma taxa) → **provado** — `recording reconfigured`, sem marcador.
- Erase com gravação ativa → NACK **provado**. `0x27` com o exemplo do vendor (`08 01 14`) → ACK e leitura `8/1/2,0 m`; dyn 9 → NACK; restaurado para `04 00 1E`.
- **A9 parcial** (erase com progresso + cancel) → **provado** — 33 passos 0→100 % + ACK em 1,2–3 s; cancel em 54 % → ACK, índice reflete o que sobrou.
- **A11 parcial** (reconexões sem `0x26` novo) → **provado** com o bleak (4 reconexões); com o Racevo **pendente**.

### F5 — status: 🟡 (bancada ✅; A7/A8 completos no Racevo/cabo pendentes)
- Log append-only (NOR, setor nunca atravessado, escrita rasgada, cheio sem sobrescrever, erase de cima para baixo com cancel) → **provado em native** — `test_flash_log` (12 casos).
- Filtros (wait-for-fix, stationary, no-fix, auto-shutdown + wait-for-data, memória cheia, reconfigure) → **provado em native** — `test_recorder` (10 casos, presets A e B do Racevo).
- **A7** (gravar → desconectar → reconectar → download com contagem e marcadores) → **provado com o bleak** — `status.stored` = registros baixados (295/295; 4378+1 marcador = expected 4379); marcadores `0x26` nas posições certas; runs como o `splitDownloadIntoRuns` do Racevo. Walkthrough no Racevo **pendente**.
- **A12** (download com gravação ativa) → **provado** — 213 registros + 3 marcadores baixados com a gravação rodando; `stored` continuou subindo (232, depois 295); 4378 registros a 25 Hz no modo sim idem.
- **A8** (queda de energia perde ≤ 2 s) → **provado com reset por EN** (proxy — mesma perda de RAM): último iTOW visto ao vivo 187971860, último gravado 187971160 → **700 ms perdidos**; ao voltar, retoma com `0x26 state=1` novo (SPEC §7.2). Arrancar o cabo de verdade: **pendente** (precisa de mão).
- Retomada da gravação salva no boot → **provado** — `rec: resuming saved recording`; o erase em seguida recebeu NACK (gravando), como manda a SPEC.
- Download cancel → **provado** — cancel após 100 → ACK em 237/376 (o vendor avisa que chegam registros depois do cancel).
- Throughput do download → **28,4 KB/s** no Mac (alvo SPEC ≥ 30). O limite agora é a central (macOS, intervalo 15 ms; `enomem_retries` altos). Medir no celular.
- **A14** (memória cheia: live segue, NACK) → lógica **provada em native**; em bancada **pendente** (encher 25 872 registros leva ~17 min a 25 Hz).

### F6 — status: 🟡 (sim ✅ em bancada; dinâmica só em native; demo no Racevo pendente)
- Modo simulação (pista circular, fix 3D, IMU coerente, numSV = 0, hAcc 2,5 m) → **provado em native** (`test_sim`, inclui o vetor de data do vendor) e **em bancada** — `sim on` + preset A: live a 25 Hz 4249/4249 Δ = 40 ms; **5 voltas de 31,416 s** (esperado 2π·100/20 = 31,4159 s) no live **e** no download (4378 registros, 0 gaps).
- Calibração dinâmica → **provado em native** (`test_calibration`); em hardware **bloqueada** (IMU sem dados) e precisa de movimento real.
- Demo M4 do Racevo em casa ("gravar 5 voltas num gate sintético") → **pendente** (celular). O device está pronto: `sim on` (ou `pio run -e esp32dev-sim -t upload`).

## Problemas abertos (com o que foi tentado)
- **MPU6050 não amostra.** Tentado: (1) leitura de volta dos registros (PWR_MGMT_1=0x01, USER_CTRL=0x40, FIFO_EN=0x78, CONFIG=0x01 — tudo como escrito); (2) `SIGNAL_PATH_RESET` + `PWR_MGMT_2=0` + esperas de 100 ms; (3) verificação de que leituras multi-byte funcionam (14/14 B) — os dados são zero de verdade; (4) troca do clock para o oscilador interno de 8 MHz. Resultado sempre: acel = giro = temperatura = 0, `INT_STATUS` = 0, FIFO = 0 B. Hipóteses: sensor danificado/falsificado ou sem alimentação analógica; precisa de power-cycle (tirar a USB) e, se persistir, trocar a placa do MPU. O autoteste fica no log de boot (`imu: FIFO self-check…`, `raw registers…`).
- **GPS é M10, não NEO-M8N** (D16). Teto 10 Hz até validar 25 Hz com fix em campo.
- Download a 28 KB/s no Mac (alvo 30). Medir com celular antes de mexer mais.
- Upload a 921600 falhava ("Unable to verify flash chip connection") — 460800 funciona; `upload_speed` = 460800.
- Abrir a serial reseta o ESP32 mesmo com DTR/RTS em baixo (CP2102) — `tools/serial_console.py` usa `--boot-wait`.
- `esp_core_dump_flash: No core dump partition found!` no boot — inofensivo.

## Decisões tomadas (e onde foram registradas na SPEC §13)
- **D13** — driver UBX próprio (sem SparkFun). §5.4 + §13.
- **D14** — `0x25` com 10 B: falta o último campo (auto-shutdown), que mantém o valor atual. §7.2 + §13.
- **D15** — `0x27` min hAcc em **décimos de metro** (exemplo em bytes do vendor). §7.6 + §13. Aplicado no firmware: com `hAcc` > mínimo, zera o b0 de Fix Status Flags (vendor p.10).
- **D16** — chip M10: VALSET, 115200, teto 10 Hz, constelações de fábrica. §2.1, §2.2 + §13.
- **D10** — medido (F1). §13.
- **D17** — flush do log a cada ≤ 1 s / ≤ 1 KB (não "por setor"), para A8 valer a 10 Hz. §8.2 + §13.
- **D18** — `0x25` idêntico durante download continua ACK (só escritas que mudam algo levam NACK). §13.
- **D19** — resposta do `0x23` = `0x21 + 0x26` até o snapshot (limite superior real). §7.3 + §13.
- **D20** — reserva síncrona de espaço com a mesma regra do log (gravação assíncrona). §13.
- **D21** — sim com hAcc 2,5 m (abaixo do default 3,0 m do `0x27`, senão o fix sairia inválido e o Racevo não fecharia volta) e numSV = 0 como marca. §9.3 + §13.
- SPEC §10.1: ACK do unlock é `… 32 3A` (typo `32 32` no PDF; o fixture está certo).
- Fila live 16 frames (640 ms a 25 Hz) e NimBLE MSYS 32: zero perdas a 25 Hz no Mac.
- `.gitignore` ignorava `partitions.csv` (`*.csv`) — exceção adicionada.

## O que o Fernando precisa fazer (testes no Racevo, pista, hardware)
- **RaceBox real 2254302541:** às ~01:00 o `rb_client` conectou nele uma vez e mandou `0x25` preset A (Track Day, Enable=1) + `0x22`; ele respondeu `recording=1`, `stored=4320`. Como o Racevo arma exatamente esse preset em toda sessão (R11), é provável que ele já estivesse assim — mas confira no app RaceBox e, se ele não devia estar gravando, pare a gravação por lá. Nada mais foi enviado a ele.
- **MPU6050:** desligar a USB, religar e olhar o log de boot (`imu: raw registers…`). Se continuar tudo zero, trocar o módulo; depois `cal static` parado 10 s e medir A6 com `rb_client live --csv` + `analyze_capture.py … imu`.
- **GPS:** decidir se fica com o M10 (10 Hz) ou troca por NEO-M9N (D1/D16). Em campo com fix, medir se o M10 sustenta 25 Hz.
- **Racevo no celular:** A1 (Mini S na diagnóstico), walkthrough standalone com a gravação ativa (A7/A12), tela de memória (erase + cancel, A9), sessão com backup armado e 3 quedas de BLE (A11), memória cheia (A14), demo M4 com `sim on`.
- **A8 de verdade:** arrancar o cabo com a gravação ativa e baixar.
- **D10 a frio:** placa desligada ≥ 1 h e medir o 1º NAV-PVT.

## Como reproduzir (comandos)
```bash
pio test -e native                                   # 107 casos
pio run -e esp32dev -t upload                        # (esp32dev-sim liga a simulação no boot)
python3 tools/serial_console.py --seconds 8          # boot: GNSS, IMU self-check, log, BLE
python3 tools/rb_client.py --name 0418890644 info
python3 tools/rb_client.py --name 0418890644 live --seconds 60 --csv captures/x.csv
python3 tools/rb_client.py --name 0418890644 cmd status preset-a reccfg-read gnsscfg-read unlock
python3 tools/rb_client.py --name 0418890644 download --csv captures/d.csv
python3 tools/rb_client.py --name 0418890644 erase [--cancel-at 50]
python3 tools/analyze_capture.py captures/x.csv itow|laps|imu
# modo sim pelo console: python3 tools/serial_console.py --boot-wait 3 -c "sim on" --seconds 1
```
**Sempre** `--name 0418890644`: há um RaceBox real por perto. A placa está em `/dev/cu.usbserial-110`.
