# EvoBox

Logger GNSS + IMU em ESP32, compatível com o **RaceBox Mini S** via BLE (protocolo público RaceBox Rev 8). Feito para conectar no app **Racevo** como se fosse um Mini S — live a 10 Hz ou 25 Hz, gravação standalone e download.

- Especificação completa: [`docs/SPEC.md`](docs/SPEC.md)
- Regras de desenvolvimento: [`CLAUDE.md`](CLAUDE.md)

## Hardware

| Bloco | Componente |
|---|---|
| MCU | ESP32-WROOM-32D |
| GNSS | u-blox NEO-M8N (10 Hz) · NEO-M9N recomendado para 25 Hz |
| IMU | MPU6050 |

Ligações e alternativas em SPEC §2.

## Status

Branch `night/2026-10-19`: F0–F6 implementadas (ver `docs/NIGHT_LOG.md` para o que foi provado em bancada e o que está pendente). Fases: SPEC §11.

⚠️ O GPS da placa atual responde como **u-blox M10** (SPG 5.10), não NEO-M8N — tratado como M10 a 10 Hz (SPEC §13 D16).

## Ferramentas de bancada

```bash
python3 tools/serial_console.py --seconds 10                         # console com tempo limite (abrir reseta a placa)
python3 tools/serial_console.py --boot-wait 3 -c status -c stats     # comandos (SPEC §9.2: help lista todos)
python3 tools/rb_client.py info                                      # DIS / classificação Mini S
python3 tools/rb_client.py live --seconds 60 --csv captures/x.csv    # A2/A3: 1º frame, Δ iTOW, gaps
python3 tools/rb_client.py cmd status preset-a reccfg-read           # A4: latência de comando
python3 tools/rb_client.py download | erase | walkthrough            # A7/A9/A12
python3 tools/analyze_capture.py captures/x.csv imu|itow|laps        # A6, A3, voltas no modo sim
```

## Conformidade com o Racevo

`test/fixtures/racebox_test_vectors.json` é cópia verbatim dos golden vectors do Racevo (frames reais do vendor com checksum). A suíte `pio test -e native` prova que os frames que o EvoBox emite são byte-idênticos aos que o Racevo espera. Não edite o fixture — recopie quando o Racevo atualizar o dele.

## Rodar a noite (Claude Code autônomo)

No Mac, com a placa no USB, na pasta do repo:

```bash
git pull
caffeinate -i claude -p "Siga docs/NIGHT_RUN.md à risca." --permission-mode auto 2>&1 | tee night-$(date +%F).log
```

De manhã: leia `docs/NIGHT_LOG.md` na branch `night/<data>`.
