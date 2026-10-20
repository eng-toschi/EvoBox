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

Fase **F0** (estrutura + suíte de conformidade) — ver SPEC §11.

## Conformidade com o Racevo

`test/fixtures/racebox_test_vectors.json` é cópia verbatim dos golden vectors do Racevo (frames reais do vendor com checksum). A suíte `pio test -e native` prova que os frames que o EvoBox emite são byte-idênticos aos que o Racevo espera. Não edite o fixture — recopie quando o Racevo atualizar o dele.

## Rodar a noite (Claude Code autônomo)

No Mac, com a placa no USB, na pasta do repo:

```bash
git pull
caffeinate -i claude -p "Siga docs/NIGHT_RUN.md à risca." --permission-mode auto 2>&1 | tee night-$(date +%F).log
```

De manhã: leia `docs/NIGHT_LOG.md` na branch `night/<data>`.
