# CLAUDE.md — EvoBox

Firmware ESP32 que se apresenta via BLE como um **RaceBox Mini S** e fala o protocolo público RaceBox BLE Rev 8. O cliente principal é o app **Racevo** (`github.com/racevo/Racevo`), que tem de conectar **sem nenhuma mudança de código**.

**`docs/SPEC.md` é o contrato.** Leia inteiro antes de qualquer tarefa. Se a spec não responde, é decisão: proponha a opção sênior, registre em `docs/SPEC.md` §13 e só então implemente. Nada de improvisar em silêncio.

## Como trabalhar

- **Uma fase por vez** (SPEC §11: F0 → F7). Não comece a próxima fase sem os critérios da atual verdes. Ao fechar uma fase, diga quais critérios (§10) foram provados e como.
- Senior-correct, não rápido. Nada de hack temporário, TODO no lugar de fazer, ou teste afrouxado para passar.
- Se o comportamento real divergir da spec (hardware, app, vendor), **pare e reporte** — não "ajuste" o firmware para esconder a divergência.

## Comandos

```bash
pio test -e native                 # suíte de conformidade + unitários (sem hardware) — roda no CI
pio run -e esp32dev                # build
pio run -e esp32dev -t upload      # gravar (porta no Mac do Fernando: /dev/cu.usbserial-220)
pio device monitor -b 115200       # console serial (SPEC §9.2)
pio run -e esp32dev-sim -t upload  # modo simulação (SPEC §9.3)
python3 tools/rb_client.py ...     # cliente BLE de bancada (bleak) — SPEC §10.2
```

## Regras invioláveis

1. **Nenhum fix GNSS é inventado.** Proibido interpolar, duplicar ou extrapolar amostras para atingir uma taxa. Um frame `0xFF 0x01` por época real do receptor. Se o módulo não faz a taxa pedida, limita e loga (SPEC §2.2, §5.3).
2. **Bytes do protocolo vêm do vendor, não da memória.** Offsets, escalas, UUIDs e checksum seguem SPEC §4–§5 e os golden vectors. Mudou algo em `lib/racebox_protocol`? Rode `pio test -e native` antes de commitar.
3. **`test/fixtures/racebox_test_vectors.json` é cópia verbatim do Racevo** (`racevo/docs/fixtures/`, commit `033bcab`). Nunca edite, nunca regenere, nunca "corrija" à mão. Se o Racevo atualizar o dele, recopie e deixe os testes dizerem o que mudou. Vetores que o fixture não tem (ex.: resposta `0x22`, `0x26`) são testados por round-trip, nunca por bytes inventados.
4. **`lib/racebox_protocol` é C++ puro**: sem `Arduino.h`, sem FreeRTOS, sem NimBLE. Tem de compilar e rodar em `native`.
5. **`iTOW` monotônico** desde o primeiro frame emitido (SPEC §5.2, R14).
6. **`0x25` idempotente**: `Enable=1` com o device já gravando e config idêntica = ACK e nada mais — sem reconfigurar o GNSS, sem gap, sem `0x26` (R11/R13). O Racevo reenvia esse comando a cada sessão e a cada reconexão.
7. **Download (`0x23`) funciona com gravação ativa** (snapshot do ponteiro de escrita, R12). NACK só durante erase.
8. **Live data começa assim que o CCCD do TX é habilitado**, com ou sem fix, e nunca bloqueia: a task GNSS não espera BLE nem flash. Filas com timeout 0 + contador de descarte.
9. **Int16 do fio faz clamp, nunca wrap** (G e rotação).
10. **Byte de bateria nunca 0** sem medição real (SPEC §5.5).
11. Identidade BLE exata: nome `RaceBox Mini S <serial 10 dígitos>`, DIS Model `RaceBox Mini S` (sem `\0`, sem espaço), serial determinístico do MAC.

## Arquitetura (SPEC §9)

Tasks FreeRTOS: `gnss`, `imu`, `builder` no core 1; `ble`, `storage`, `power` no core 0. Pinos e build flags só em `src/config.h`. Layout de pastas em SPEC §9.5 — siga-o.

## Hardware atual (SPEC §2)

ESP32-WROOM-32D (4 MB) · NEO-M8N na UART2 (GPIO16 RX / GPIO17 TX, 5 V) · MPU6050 em I²C (GPIO21 SDA / GPIO22 SCL, `0x68`, 3,3 V, montado invertido). **O NEO-M8N não faz 25 Hz** — o firmware detecta o módulo (`UBX-MON-VER`) e suporta NEO-M9N.

## Definição de pronto (por PR)

- [ ] `pio test -e native` verde; `pio run -e esp32dev` sem warnings novos.
- [ ] Lógica nova de protocolo/payload/calibração tem teste em `native`.
- [ ] Critérios da fase (SPEC §10) que dependem de hardware: testados em bancada e descritos no PR, ou listados explicitamente como pendentes para o Fernando.
- [ ] `docs/SPEC.md` atualizado se o comportamento mudou (e a decisão registrada em §13).

## Git

Conventional commits (`feat:`, `fix:`, `test:`, `docs:`, `chore:`), commits pequenos. `main` sempre compila e com `native` verde.
