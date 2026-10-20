# EvoBox — Firmware ESP32 (RaceBox Mini S compatível)

**Especificação técnica de desenvolvimento · v0.2 · 2026-10-19** (v0.2: comportamento do Racevo conferido no código — R11–R16)
Repositório: `github.com/eng-toschi/EvoBox`. Este documento é o contrato do firmware — implemente as seções §4–§8 literalmente; o que estiver marcado `‹VALIDAR›` é decisão pendente ou item a confirmar em hardware (lista consolidada em §13).

---

## 0. Resumo em uma página

Um dispositivo ESP32 + GNSS u-blox + IMU que **se apresenta via BLE como um RaceBox Mini S** e fala o **protocolo público RaceBox BLE Rev 8** (UBX sobre "UART BLE"). Qualquer cliente que fala com um RaceBox — em primeiro lugar o **Racevo** — conecta nele sem nenhuma mudança de código.

- Live data `0xFF 0x01` (frame de 88 bytes) a **10 Hz ou 25 Hz**, selecionável pelo próprio protocolo (`0xFF 0x25`, campo *data rate*).
- Comandos de Mini S: status (`0x22`), download (`0x23` → `0x21`/`0x26`), erase (`0x24`), config de gravação (`0x25`), config GNSS (`0x27`), unlock (`0x30`), ACK/NACK (`0x02`/`0x03`).
- Gravação standalone em flash, com os filtros do vendor (wait-for-fix, stationary, no-fix, auto-shutdown).
- IMU amostrada a 1 kHz e **média por época GNSS**, no frame do veículo (calibração de montagem persistida).
- Nada é inventado: **nenhuma amostra GNSS é interpolada ou fabricada** para atingir uma taxa. Se o receptor não entrega 25 Hz, o firmware entrega o que o receptor entrega e diz isso.

> **Atenção — decisão de hardware antes de começar (§2.2):** o **NEO-M8N atual não faz 25 Hz.** O máximo dele com constelações concorrentes é 10 Hz. Para 25 Hz é preciso trocar o módulo GNSS (recomendado: **NEO-M9N**, mesmo footprint NEO, 25 Hz com 4 constelações). O firmware suporta os dois, detectando o módulo em runtime.

### O que este projeto substitui

O repo anterior (`Open-Source-RaceBox-mini-Emulator`) tinha optado por um **protocolo BLE próprio** (payload compacto ~24–26 bytes). **Esta especificação reverte essa decisão:** o firmware novo fala o protocolo RaceBox byte a byte, porque o Racevo já tem `packages/racebox_protocol`, FrameAssembler, `.rbox`, lap engine e testes de conformidade construídos sobre ele. Protocolo próprio = código novo no app; protocolo RaceBox = zero mudança no app.

---

## 1. Fontes e contratos

| Fonte | O que define | Onde |
|---|---|---|
| **RaceBox BLE Protocol Description rev 8** (PDF do vendor) | Bytes, UUIDs, escalas, comandos — **a lei** | `racevo/docs/RaceBox BLE Protocol Description rev 8.pdf` · <https://www.racebox.pro/products/mini-micro-protocol-documentation> |
| **Racevo `docs/04_RACEBOX_TELEMETRY_SPEC.md`** | Como o Racevo consome o protocolo; §1–§7 transcrevem o vendor; §11 = checklist `‹VERIFY›` | `github.com/racevo/Racevo` |
| **Racevo `app/lib/features/telemetry/data/`** | O comportamento real do cliente (o que ele manda, quanto espera) — ver §3 | idem |
| **`racebox_test_vectors.json`** | Golden vectors oficiais (frames reais do vendor com checksum) | `racevo/docs/fixtures/` e cópia em `goawake-logger/packages/rbox_format/fixtures/` |
| **goawake-logger `docs/goawake-logger-SPEC.md`** | Convenção de eixos (§6.8, §14) e calibração estática+dinâmica (§5) — reaproveitadas aqui | `github.com/eng-toschi/goawake-logger` |
| Página do produto Mini S | Especificação de referência (25 Hz, 4 GNSS, ±8 g, ±320 dps, BT 5.2) | <https://www.racebox.pro/products/racebox-mini-s> |
| Emulador de terceiros | Referência de implementação (não dependência) | <https://github.com/anchit92/ESP32-RaceBox-mini-Emulator> |

**Princípio de conformidade (mesmo do goawake-logger):** não há dependência de código com o Racevo. O que garante que os dois lados falam a mesma língua é a **suíte de conformidade** (§10.1) rodando os mesmos golden vectors. `racebox_test_vectors.json` é copiado **verbatim** para `test/fixtures/` — é fixture, não código; duplicar é o ponto.

---

## 2. Hardware

### 2.1 Placa atual (montada e validada)

| Bloco | Componente | Ligação |
|---|---|---|
| MCU | ESP32-WROOM-32D (HW-394, USB-C, CP2102), 4 MB flash | serial de debug `115200` (porta no Mac: `/dev/cu.usbserial-220`) |
| GNSS | u-blox **NEO-M8N** (breakout GY-GPS V3-NEO), alimentado em 5 V — ⚠️ **divergência medida em 2026-10-20: o chip responde como u-blox M10** (`UBX-MON-VER`: `ROM SPG 5.10 (7b202e)`, hw `000A0000`, `PROTVER=34.10`, sem `MOD=`), ver D16 | **UART2**: GPIO16 = RX2 (← TX do GPS), GPIO17 = TX2 (→ RX do GPS) |
| IMU | MPU6050, alimentado em 3,3 V, AD0 em GND → endereço `0x68` | **I²C**: GPIO21 = SDA, GPIO22 = SCL |

Montagem física: MPU6050 invertido sob o ESP32, GPS por cima. **O MPU invertido é exatamente o caso que a calibração de montagem (§6.4) resolve** — nenhum sinal de eixo é hardcoded.

### 2.2 GNSS: 10 Hz vs 25 Hz — limitação do NEO-M8N

| Módulo | Máx. com GNSS concorrente | 25 Hz? | Observação |
|---|---|---|---|
| NEO-M8N (atual) | **10 Hz** (só GPS chega a ~18 Hz) | **Não** | Config legada (`CFG-RATE`, `CFG-NAV5`, `CFG-GNSS`, `CFG-PRT`) |
| **NEO-M9N** (recomendado) | **25 Hz com 4 constelações** | **Sim** | Mesmo form factor NEO; config por `CFG-VALSET` |
| MAX-M10S | 10 Hz com 4 GNSS (maior com menos constelações) | Parcial `‹VALIDAR›` | Consumo menor; menos folga que o M9N |

Regras do firmware:

1. Detectar o módulo no boot (`UBX-MON-VER`) e definir `gnss_max_rate_hz` (M8N → 10; M9N → 25; M10 → 10, D16). A geração sai de `MOD=` quando existe; senão de `PROTVER` (M8 ≤ 23, M9 27–33, M10 ≥ 34) — o hwVersion `000A0000` é comum a M9 e M10.
2. Uma taxa pedida acima do máximo é **limitada ao máximo**, logada no console e refletida no espaçamento real de `iTOW`. **Proibido** interpolar, duplicar ou extrapolar fixes para "chegar" a 25 Hz — o lap engine do Racevo interpola cruzamento de linha entre fixes reais (±2 ms); um fix fabricado vira tempo de volta falso.
3. Constelações por módulo/taxa (default, `‹VALIDAR›` em campo): M8N @10 Hz = GPS+GLONASS; M9N @25 Hz = GPS+GLONASS+Galileo+BeiDou (se o M9N perder qualidade a 25 Hz com 4 GNSS, cair para GPS+Galileo+GLONASS e documentar).

### 2.3 Ligações opcionais recomendadas (não bloqueiam a v1)

| Sinal | GPIO sugerido | Para quê |
|---|---|---|
| MPU6050 `INT` | GPIO4 | Interrupção de FIFO (evita polling a 1 kHz) |
| GNSS `TIMEPULSE`/PPS | GPIO34 (input-only) | Alinhamento fino IMU↔época GNSS (§6.3) |
| Divisor de bateria (100k/100k) | GPIO35 (ADC1) | Campo de bateria real (§5.5) |
| Botão | GPIO0 (BOOT) | Wake do auto-shutdown; long-press = start/stop de gravação |
| LED de status | GPIO2 | Estados (§9.3) |

---

## 3. O que o Racevo realmente faz (lido do código, não do PDF)

Comportamento do cliente em `racevo/app/lib/features/telemetry/data/` que o firmware **precisa** satisfazer:

| # | Comportamento do Racevo | Consequência para o firmware |
|---|---|---|
| R1 | Scan aceita **nome com prefixo** (`RaceBox Mini S ` testado **antes** de `RaceBox Mini `) **ou** o UUID do serviço UART anunciado | Anunciar os dois (§4.1) |
| R2 | `connect(mtu: 512)` + `ConnectionPriority.high` no Android | Aceitar MTU até 512; aceitar intervalo de conexão curto |
| R3 | `discoverServices()`; **falha se não achar UART RX+TX** | Serviço UART obrigatório e com os UUIDs exatos |
| R4 | Habilita notify no TX e **só escuta** — o live path não manda comando de start | Começar a enviar `0x01` **assim que o CCCD for habilitado**, com ou sem fix |
| R5 | Lê DIS Model/Serial/Firmware; Model **exato** `"RaceBox Mini S"` → `DeviceModel.miniS`; qualquer outra string → `other` | String exata, sem `\0` nem espaço extra |
| R6 | Timeouts: primeiro dado **6 s**, fix **120 s**, resposta a comando **8 s**, stall de transferência **15 s** | Primeiro frame em < 1 s após subscribe; responder comandos em < 500 ms |
| R7 | Walkthrough manda `0x22` (status) e, conforme o estado, `0x30` (unlock) e `0x23` (download) | Implementar Mini S completo desde a v1 — mesmo com memória vazia |
| R8 | Tela de memória manda `0x25` (preset), `0x24` (erase) e cancel de erase (`0x24` + 1 byte) | idem |
| R9 | Presets do Racevo: Track Day = `25 Hz, flags 0x1F, 1389 mm/s, 30 s, 30 s, 300 s`; Standing Start = `25 Hz, flags 0x1D, 0, 0, 30 s, 600 s` | Aceitar os dois exatamente; com M8N, rate 25 → limitado a 10 (§2.2) |
| R10 | Bateria decodificada como Mini: `bit7 = carregando`, `bits0–6 = %` | Nunca mandar 0 % sem bateria real (§5.5) |
| R11 | **Backup automático (G8, `recording_controller.dart`)**: ao iniciar uma gravação ao vivo num Mini S, o Racevo escreve o preset **Track Day** (`0x25`, Enable=1, 25 Hz, `0x1F`) — e **reescreve a cada reconexão** | `0x25` repetido com a mesma config enquanto já grava = **ACK sem efeito**: sem reconfigurar o GNSS, sem gap, sem novo `0x26` (§7.2). Na prática, **toda sessão do Racevo deixa o device gravando a 25 Hz** |
| R12 | O download (walkthrough "standalone") faz `0x22` → [`0x30`] → `0x23` **sem parar a gravação** | Download **tem** de funcionar com gravação ativa (§7.3) — por causa do R11, o device quase sempre estará gravando |
| R13 | `splitDownloadIntoRuns`: `0x26` state=1 abre run, state=0 fecha; registros antes de qualquer marcador viram uma run | Cada `0x26` state=1 espúrio **parte a sessão em duas** no app — só gravar marcador em transição real de estado |
| R14 | O tempo da amostra no `.rbox` é `iTOW − iTOW do primeiro frame`; salto para trás > meia semana é tratado como virada de semana GPS | `iTOW` tem de ser **monotônico** desde o primeiro frame, inclusive antes do fix (§5.2) |
| R15 | Em queda de link, reconecta por **serial** (scan → nome anunciado) a cada 2 s | Voltar a anunciar imediatamente após desconexão, com o mesmo nome/serial |
| R16 | RX: tenta write **com** resposta; cai para sem resposta | RX com as duas propriedades (§4.2) |

---

## 4. Camada BLE

Stack: **NimBLE-Arduino** (menos RAM que Bluedroid; host roda no core 0).

### 4.1 Identidade e advertising

- Nome: `RaceBox Mini S ` + **serial de 10 dígitos** derivado do MAC de fábrica (`esp_efuse_mac_get_default`; últimos 4 bytes como `uint32` em `%010u`). Determinístico: o mesmo hardware sempre tem o mesmo serial — o Racevo indexa o registro do dispositivo por serial.
- O nome completo tem 25 caracteres → `flags (3 B) + complete local name (27 B) = 30 B`: **cabe no advertising mas sem espaço para o UUID de 128 bits**. Colocar o UUID do serviço UART no **scan response**.
- Intervalo de advertising: 100 ms nos primeiros 30 s após boot/desconexão, depois 500 ms.
- **Uma conexão por vez** (igual ao RaceBox real): parar advertising ao conectar; reiniciar ao desconectar.
- Sem pairing/bonding (o RaceBox real não deve ser pareado no Bluetooth do sistema).

### 4.2 GATT

| Serviço / Característica | UUID | Propriedades | Conteúdo |
|---|---|---|---|
| **UART Service** | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` | — | — |
| RX (cliente → device) | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` | Write, Write Without Response | Comandos UBX (podem chegar fragmentados) |
| TX (device → cliente) | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` | Notify | Live data + respostas |
| **Device Information** | `0x180A` | — | — |
| Model | `0x2A24` | Read | `RaceBox Mini S` |
| Serial Number | `0x2A25` | Read | o serial de 10 dígitos |
| Firmware Revision | `0x2A26` | Read | `3.3` (≥ 3.3 libera GNSS config e preset 20 Hz no cliente) |
| Hardware Revision | `0x2A27` | Read | `GA-ESP32-M8N` / `GA-ESP32-M9N` conforme detecção |
| Manufacturer | `0x2A29` | Read | `GoAwake` (configurável; o Racevo não lê) |

**Não** implementar o serviço NMEA (`0x1101`): o vendor desaconselha os dois protocolos simultâneos e o Racevo nunca o assina.

Identidade configurável por build flag (`-DGA_IDENTITY=RACEBOX_MINI_S` default; `GOAWAKE` anuncia `GoAwake Logger <serial>` — o Racevo ainda conecta pelo UUID do serviço, mas classifica como `other`).

### 4.3 MTU, fragmentação e fila de notify

- Payload útil por notify = `MTU − 3`. Frame live = 88 B → cabe inteiro com MTU ≥ 91.
- Se o MTU negociado for menor (23 = default), **fragmentar** o frame em notifies consecutivos (o vendor faz isso e o FrameAssembler do Racevo existe para isso).
- No download (§7.3), **empacotar vários frames por notify** até `MTU − 3` (vendor faz ~244 B por notify).
- Pedir connection interval 7,5–15 ms após conectar (`ble_gap_update_params`); aceitar o que a central der.
- Fila TX com prioridade: respostas de comando > live data. Se o buffer do NimBLE recusar (`BLE_HS_ENOMEM`):
  - live data: **descartar o frame mais antigo** e incrementar `stats.live_dropped` (nunca bloquear a task GNSS);
  - download: **backpressure** (aguardar e reenviar; nunca descartar registro de histórico).

### 4.4 RX: montagem de comandos

Escritas no RX podem chegar fragmentadas ou concatenadas. Usar o **mesmo FrameAssembler** do lado do Racevo (spec 04 §3.3): FIFO de 1 KB, resync em `0xB5 0x62`, espera de completude por `len`, validação Fletcher-8; frame inválido é **descartado silenciosamente** (logado no console). Comprimento de payload > 504 → resync.

---

## 5. Protocolo — o que o firmware emite

### 5.1 Framing

`B5 62 | class | id | len_lo len_hi | payload | CK_A CK_B`, checksum Fletcher-8 sobre `class..fim do payload`. Toda a classe RaceBox é `0xFF`. Inteiros little-endian; nenhum float no fio.

### 5.2 Live Data `0xFF 0x01` — montado a partir do `UBX-NAV-PVT`

O payload RaceBox (80 B) é derivado do NAV-PVT (92 B). **Os primeiros 48 bytes são cópia direta**; o resto é remapeado.

| RaceBox off | Campo | Origem (NAV-PVT off) | Transformação |
|---:|---|---:|---|
| 0–23 | iTOW, data/hora UTC, valid, tAcc, nano, fixType, flags, flags2, numSV | 0–23 | cópia; **fixType**: 2→2, 3→3, 4 (GNSS+DR)→3, demais→0 |
| 24–47 | lon, lat, height (WGS), hMSL, hAcc, vAcc | 24–47 | cópia |
| 48 | Speed (mm/s) | 60 `gSpeed` | se 3D speed habilitado (§7.6): `sqrt(velN²+velE²+velD²)` dos off 48/52/56 |
| 52 | Heading (1e-5°) | 64 `headMot` | cópia |
| 56 | Speed accuracy | 68 `sAcc` | cópia |
| 60 | Heading accuracy | 72 `headAcc` | cópia |
| 64 | PDOP (×100) | 76 `pDOP` | cópia |
| 66 | Lat/Lon flags | 78 `flags3` (byte baixo) | bit0 = `invalidLlh`; bits1–4 = idade da correção diferencial |
| 67 | Bateria | — | §5.5 |
| 68/70/72 | G X/Y/Z (milli-g, int16) | IMU | média da janela da época (§6), **clamp em ±32767, nunca wrap** |
| 74/76/78 | Rot X/Y/Z (centi-°/s, int16) | IMU | idem; com MPU a ±500 dps o fio satura em ±327,67 °/s → clamp |

Regras:

- **Um frame por época GNSS real.** O frame sai assim que o NAV-PVT da época é validado (orçamento: ≤ 5 ms entre o último byte do NAV-PVT na UART e o enqueue no BLE).
- Sem fix, o frame **continua saindo** na taxa configurada com `fixType = 0` e `flags bit0 = 0` (é o que o RaceBox faz e o que o R4/R6 exigem).
- **`iTOW` monotônico (R14).** Antes de resolver a hora, o receptor pode mandar `iTOW` de um relógio interno e depois saltar quando a hora é resolvida; o Racevo usa o primeiro `iTOW` como origem da sessão e leria um salto para trás como virada de semana. Regras: (a) usar o backup do módulo (bateria/supercap do breakout + config salva no BBR) para hot start; (b) enquanto `valid.validTime == 0`, **substituir o `iTOW` por um contador local** ancorado no `millis()` e, quando a hora resolver, calcular o deslocamento `contador − iTOW real` e **aplicá-lo até o próximo boot** (o fio continua monotônico; o salto vira só um registro de log). Se o primeiro NAV-PVT já vier com `validTime = 1` (hot start, o caso normal), nada disso é ativado e o `iTOW` sai cru; (c) `‹VALIDAR›` em cold start com o Racevo gravando desde antes do fix (teste A13).
- Antes do primeiro NAV-PVT (receptor ainda configurando) **não** emitir nada — o primeiro dado tem até 6 s; o boot do GNSS cabe nisso. `‹VALIDAR›` tempo de boot+config do M8N/M9N.
- "Good fix" para os filtros internos (§7.4): `fixType == 3 && flags.bit0 == 1 && hAcc ≤ minHAcc` (min horizontal accuracy da §7.6).

### 5.3 Taxas

Mapeamento do campo *data rate* (vendor §6.2) → `CFG-RATE.measRate`:

| Valor | Taxa | measRate | M8N | M9N |
|---:|---|---:|---|---|
| 0 | 25 Hz | 40 ms | → 10 Hz (limitado) | 25 Hz |
| 4 | 20 Hz | 50 ms | → 10 Hz (limitado) | 20 Hz |
| 1 | 10 Hz | 100 ms | 10 Hz | 10 Hz |
| 2 | 5 Hz | 200 ms | 5 Hz | 5 Hz |
| 3 | 1 Hz | 1000 ms | 1 Hz | 1 Hz |

Comportamento do vendor que deve ser replicado: sem gravação ativa, o live roda na taxa **padrão (25 Hz, ou o máximo do módulo)**; com gravação ativa, o live **segue a taxa da gravação**; desabilitar a gravação volta à padrão. Mudar a taxa reconfigura o receptor — aceitável um gap ≤ 1,5 s (o vendor documenta o mesmo gap).

**Consequência do R11:** com o Racevo, quem escolhe a taxa é o app, e ele sempre arma 25 Hz. Num M9N isso significa 25 Hz em toda sessão. Para forçar 10 Hz com o Racevo existe o **teto de taxa** `max_rate` (NVS, comando de console `maxrate <10|25>`): toda taxa pedida acima do teto é limitada a ele, igual ao limite de hardware do M8N. É um desvio consciente do comportamento do Mini S real e fica visível no `iTOW` e no log.

Para o usuário poder escolher 10 Hz no live **sem** gravar, adicionar a constante de build `GA_DEFAULT_LIVE_RATE` (default `0` = 25 Hz/máx.) e o comando de console `rate <0..4>` (§9.2). Pelo protocolo, o caminho é o mesmo do vendor: `0x25` com Enable=1.

### 5.4 UART com o GNSS

- Boot a 9600 (default de fábrica), reconfigurar para **115200** (M8N) ou **230400** (M9N @25 Hz) e reabrir a UART2.
- Desligar **todas** as sentenças NMEA na UART1 do módulo; habilitar só `UBX-NAV-PVT` (1 por época). Orçamento a 25 Hz: 100 B × 25 = 2,5 KB/s — folgado em 230400.
- Configurar e **salvar** no BBR/flash do módulo (`CFG-CFG` no M8, layers RAM+BBR no M9) para boots rápidos.
- Driver UBX próprio (D13): a task `gnss` lê a UART2, passa os bytes pelo mesmo `FrameAssembler` de `lib/racebox_protocol` e copia o NAV-PVT **cru** (92 B) para a `pvt_q`; nada de BLE na task. Montagem de comandos e parse (`src/gnss/ublox_config.*`) são C++ puro, testados em native.
- M8: compara (poll) `CFG-PRT`/`CFG-MSG`/`CFG-NAV5` antes de escrever e só faz `CFG-CFG` (BBR+flash) se algo persistente mudou — sem desgastar a flash do módulo a cada boot. M9/M10: `CFG-VALSET` em RAM+BBR. Mudança de taxa em runtime: só RAM.
- A última baud em que o receptor respondeu fica em NVS e é a primeira tentativa do autobaud.

### 5.5 Byte de bateria (offset 67)

- Com divisor no GPIO35: mapear tensão→% (curva LiPo 1S, filtro de 10 s, como o vendor), `bit7` = carregando (detecção via pino do carregador, se houver, `‹VALIDAR›`).
- **Sem medição de bateria** (alimentado por USB/12 V): enviar `0x64` (100 %, não carregando). **Nunca** `0` — o vendor recomenda o cliente desconectar em 0 % quando não está gravando.

---

## 6. IMU — aquisição, média por época e eixos

### 6.1 Configuração do MPU6050

- Faixas: acelerômetro **±8 g** (igual ao Mini S), giroscópio **±500 dps** (o Mini S é ±320; o MPU não tem essa faixa — usar a superior e fazer clamp no fio).
- `SMPLRT_DIV` para **1 kHz**, DLPF ~ 188 Hz, **FIFO** habilitado (accel+gyro, 12 B/amostra), leitura em rajada a cada ~10 ms (ou por `INT`).
- I²C a 400 kHz.

### 6.2 Média por época

O RaceBox documenta "IMU a 1 kHz, média" por amostra entregue. Implementação:

- A task IMU acumula somas (6 canais, `int32`) e contagem.
- Quando chega o NAV-PVT da época *n*, o builder **fecha a janela** e usa a média das amostras da época *n* (≈40 amostras a 25 Hz, ≈100 a 10 Hz).
- Janela vazia (I²C travou): repetir os últimos valores, setar `stats.imu_stall` e logar. Nunca bloquear o frame.

### 6.3 Alinhamento temporal

- **v1:** janela = intervalo entre chegadas de NAV-PVT na UART (latência do receptor ~constante, desvio < 1 época).
- **v2 (com PPS ligado):** ancorar as janelas no TIMEPULSE configurado na mesma taxa da navegação → alinhamento < 1 ms.

### 6.4 Eixos e calibração de montagem

**Convenção do fio (a que o Racevo assume — `g_ball.dart`, ver goawake-logger SPEC §14):**

| Situação | gX | gY | gZ |
|---|---|---|---|
| Parado, nivelado | 0 | 0 | ≈ +1 g |
| Acelerando | **+** | 0 | ≈ +1 g |
| Freando | **−** | 0 | ≈ +1 g |
| Curva à **direita** | 0 | **+** | ≈ +1 g |

Giro: X = roll, Y = pitch, Z = yaw, mão direita sobre os eixos do fio `‹VALIDAR›` — é o `‹VERIFY›` nº 2 da spec 04 do Racevo (o sinal do RaceBox real nunca foi confirmado). O firmware isola os seis sinais numa única estrutura (`AxisConvention`), como o goawake-logger.

**Calibração** (porta direta do goawake-logger §5, em C++), resultado = matriz de rotação `R_dv` (device→veículo) em NVS:

1. **Estática** (dispositivo parado ≥ 10 s): média do acelerômetro = vetor gravidade → resolve o eixo vertical; média do giro = bias.
2. **Dinâmica** (automática, em movimento): nas janelas em linha reta (`|yaw| ≤ 0,10 rad/s`, `speed > 5 m/s`, good fix), correlacionar a aceleração horizontal com `d(speed)/dt` do GNSS → resolve o eixo longitudinal **com o sinal certo por construção**. Converge e persiste sozinha; não exige nada do piloto.
3. Até a dinâmica convergir: usar `R_dv` da última calibração salva; sem nenhuma, só a estática (eixos X/Y no frame do dispositivo projetados no plano horizontal) e sinalizar no console.

Disparo: comando de console `cal static` / `cal reset` (§9.2) e, opcionalmente, long-press no botão parado. O bias do giro é reestimado a cada parada > 5 s com good fix e `speed < 0,3 m/s`.

---

## 7. Comandos Mini S

Todas as respostas usam classe `0xFF`. ACK/NACK = id `0x02`/`0x03`, payload de 2 B = `class, id` do comando respondido. Comando com checksum inválido: ignorar. Id desconhecido: NACK `‹VALIDAR›` (o vendor não documenta).

### 7.1 `0x22` — Status da gravação

Query vazia → resposta 12 B: `recording(1) | memory %(1) | security(1)=0x00 | reserved(1) | stored records(u32) | capacity records(u32)`.
`capacity` = registros que cabem **de verdade** na partição (§8) — não os 196 608 do vendor.

### 7.2 `0x25` — Configuração da gravação

- Ler: payload vazio → 12 B com a config atual.
- Escrever: aceitar **10 ou 12 B** (`‹VERIFY›` nº 4 do Racevo — o vendor diz 10 na prosa e manda 12 no exemplo). Com 10 B falta o último campo (auto-shutdown interval, offset 10), que **mantém o valor atual** (D14). ACK em sucesso.
- NACK se: data rate fora de 0–4, memória cheia sem espaço para iniciar, ou download/erase em andamento.
- `Enable=1` com o device **parado**: entra em gravação imediatamente, grava um `0x26` (state=1) e reconfigura a taxa. `Enable=0`: grava `0x26` (state=0), para, volta à taxa padrão.
- **Idempotência (R11/R13):** `Enable=1` com o device **já gravando e config idêntica** → ACK e **nada mais** (sem reconfigurar o receptor, sem gap, sem `0x26`). Com config diferente → aplicar como o vendor (gap ≤ 1,5 s), gravando `0x26` state=0 + state=1 só se a taxa mudou `‹VALIDAR›` contra um Mini S real.
- Memória cheia: `Enable=1` → NACK (o Racevo tolera: o chip de backup só não aparece). Encher a memória **durante** uma gravação: parar de gravar (`0x26` state=0 se couber), **o live continua normal**.
- Config persiste em **NVS** (o Mini S persiste via RAM com bateria). Ao bootar com `Enable=1` salvo, retoma a gravação (com `0x26` state=1 novo).

### 7.3 `0x23` — Download

1. Payload vazio → responder `0x23` com `u32 = registros armazenados` (limite superior para a barra de progresso).
2. **Silenciar o live `0x01`** durante o dump.
3. Transmitir, na ordem de gravação, `0x21` (80 B, layout idêntico ao `0x01`) intercalados com os `0x26` gravados (12 B), empacotando vários frames por notify.
4. Fim → ACK referenciando `0xFF 0x23`, retomar live.
- **Cancel** = `0x23` com 1 byte qualquer → parar de ler a flash, drenar a fila, ACK. (Alguns `0x21` após o cancel são esperados pelo cliente.)
- **Download com gravação ativa é obrigatório (R12).** Ao receber o `0x23`, tirar um *snapshot* do ponteiro de escrita: o dump vai até ali; a gravação continua no buffer/setores seguintes e não entra neste download. O log append-only (§8.2) torna isso seguro (leitura só de setores já fechados + o buffer congelado no snapshot).
- NACK só se erase em andamento.
- Throughput alvo ≥ 30 KB/s (vendor: ~60 KB/s típico). Stall > 15 s derruba o cliente (R6).

### 7.4 Filtros da gravação (flags `0x25` byte 2)

| Bit | Filtro | Implementação |
|---|---|---|
| b0 | Wait for fix | Não gravar até o primeiro good fix |
| b1 | Stationary | `speed < threshold` por `interval` s → pausa (grava `0x26` state=2); retoma no primeiro registro acima do threshold (só com fix válido) |
| b2 | No-Fix | Sem good fix por `interval` s → pausa (`0x26` state=2); retoma ao recuperar fix |
| b3 | Auto-shutdown | Sem dados novos por `interval` s **e** sem cliente BLE → grava `0x26` (state=0, o vendor salva marcador no auto-shutdown), faz flush e entra em **deep sleep** com wake por GPIO0 (timer: D7) |
| b4 | Wait for data before shutdown | Timer de auto-shutdown só arma após ≥ 1 registro gravado |

Regra do vendor a manter: retomada após pausa **não** grava marcador no armazenamento (só é reportada ao vivo).

### 7.5 `0x24` — Erase

Payload vazio → iniciar erase imediatamente; emitir `0x24` com 1 B de progresso 0–100 % a cada setor de 64 KB; ACK ao fim; live silenciado durante. Cancel = `0x24` + 1 byte → parar, ACK (a memória fica parcialmente apagada — o índice deve refletir isso). NACK se gravação/download em andamento.

### 7.6 `0x27` — Config do receptor GNSS

- Ler (vazio) → 3 B: `dynModel | 3D speed (bool) | min horizontal accuracy (décimos de metro)` — o exemplo do vendor (Rev 8 p.10) codifica 2,0 m como `0x14` (D15).
- Escrever 3 B → validar `dynModel ∈ 0..8` (NACK fora) e aplicar: M8 → `CFG-NAV5.dynModel`; M9 → `CFG-NAVSPG-DYNMODEL`. 3D speed afeta só o cálculo do offset 48. Min hAcc afeta o "good fix" (§5.2) e, como diz o vendor (p.10), com `hAcc` pior que o mínimo o frame sai com o **b0 de Fix Status Flags zerado** ("report fix loss"). Aplicado no firmware (não no `pAcc` do receptor, que é em metros inteiros e difere entre M8/M9/M10).
- Default de fábrica: `4` (automotive), ground speed, 3,0 m (byte `30`). Persistir em NVS.
- **Nunca** aceitar UBX cru do cliente — só este formato documentado.

### 7.7 `0x30` — Unlock

Segurança desabilitada (não há comando de cliente para habilitá-la no Rev 8): **sempre ACK**, qualquer código. Implementar o campo de security em `0x22` como `0x00`. Suporte a segurança real fica para depois (ver §13).

---

## 8. Armazenamento

### 8.1 Orçamento no ESP32-WROOM-32D (4 MB)

Tabela de partições custom (`partitions.csv`): `nvs 24K | otadata 8K | app0 1.5M | app1 1.5M | storage ~960K` (com OTA) **ou** `app 1.9M | storage ~2M` (sem OTA, recomendado para a v1).

| Partição de dados | Registros (80 B + 4 B overhead) | @25 Hz | @10 Hz |
|---|---:|---:|---:|
| ~2,0 MB | ~24 000 | ~16 min | ~40 min |

É **muito** menos que o Mini S real (>2 h @25 Hz). Opções para paridade (decisão em §13): microSD via SPI (horas), ou ESP32-S3 N16R8 (16 MB flash → ~2 h @25 Hz).

### 8.2 Formato

Não usar sistema de arquivos para o log (escrita a 25 Hz em LittleFS desgasta e fragmenta). **Log circular append-only direto na partição** via `esp_partition_*`:

- Registro = `tag(1) | len(1) | payload(N) | crc8(1)` — tags `0x21` (80 B) e `0x26` (12 B).
- Escrita em buffer de RAM de 4 KB, flush por setor; em queda de energia perde-se no máximo o setor aberto (≤ ~2 s @25 Hz).
- Boot: varredura do último setor válido para recuperar o ponteiro de escrita e a contagem (sem índice separado que possa divergir).
- Cheio: **parar de gravar** (não sobrescrever o mais antigo — o vendor não sobrescreve) e reportar `memory % = 100`.
- Não-objetivo: o `.rbox` é formato do Racevo, gerado pelo app. O firmware não escreve `.rbox`.

---

## 9. Arquitetura do firmware

### 9.1 Tasks (FreeRTOS)

| Task | Core | Prio | Responsabilidade |
|---|---|---|---|
| `gnss` | 1 | alta | Ler UART2, parser UBX, NAV-PVT → fila `pvt_q` |
| `imu` | 1 | alta | FIFO do MPU a 1 kHz, acumuladores por janela |
| `builder` | 1 | média | Consome `pvt_q`, fecha janela IMU, aplica `R_dv`, monta o payload de 80 B, envia para `ble_q` e `log_q` |
| `ble` | 0 | média | NimBLE: notify (fila com prioridade), RX assembler, despacho de comandos |
| `storage` | 0 | baixa | Append no log, download, erase |
| `power` | 0 | baixa | Bateria, filtros de gravação, auto-shutdown, LED |

Nenhuma task bloqueia outra: todas as filas com `xQueueSend(..., 0)` + contador de descarte.

### 9.2 Console serial (115200)

`status` · `rate <0..4>` · `gnss info` · `cal static` · `cal reset` · `cal show` · `rec start|stop` · `mem` · `erase` · `stats` (drops, jitter, rate real) · `sim on|off` · `log pvt|frames`.

### 9.3 Modo simulação (`-DGA_SIM=1` ou `sim on`)

Gera NAV-PVTs sintéticos de uma pista circular (raio e velocidade configuráveis, 25 Hz, fix 3D) com a IMU coerente (aceleração centrípeta). Serve para testar o Racevo **dentro de casa** — inclusive o demo do M4 dele ("gravar 5 voltas num gate sintético"). Frames em modo sim têm `numSV = 0` e `hAcc` alto `‹VALIDAR›` para nunca serem confundidos com dado real numa sessão de pista.

### 9.4 LED

Piscando lento = advertising sem fix · piscando rápido = advertising com fix · aceso = cliente conectado (igual ao RaceBox) · duplo flash periódico = gravando · flash contínuo = download/erase.

### 9.5 Estrutura do repositório

```
EvoBox/
├── platformio.ini            envs: esp32dev (M8N/M9N detectado), esp32dev-sim, native (testes)
├── partitions.csv
├── src/
│   ├── main.cpp              boot, criação das tasks
│   ├── gnss/                 ublox_config.{h,cpp}, gnss_task.cpp
│   ├── imu/                  mpu6050.{h,cpp}, imu_task.cpp, calibration.{h,cpp}
│   ├── ble/                  ble_server.{h,cpp}, tx_queue.{h,cpp}
│   ├── storage/              flash_log.{h,cpp}
│   ├── app/                  builder.cpp, recorder.cpp (filtros), commands.cpp, power.cpp, console.cpp, sim.cpp
│   └── config.h              pinos, build flags
├── lib/racebox_protocol/     C++ puro, sem Arduino: frame.h, checksum, assembler, payload (NAV-PVT→RB), axis.h
├── test/
│   ├── fixtures/racebox_test_vectors.json   cópia verbatim do Racevo
│   ├── test_conformance/     golden vectors
│   ├── test_assembler/       fuzz/split (espelha racevo 04 §9.3)
│   ├── test_payload/         mapeamento NAV-PVT→RaceBox, clamps, fixType
│   └── test_calibration/     rotação sintética conhecida → R_dv recuperada
├── tools/
│   ├── rb_client.py          bleak: conecta, decodifica, CSV, mede taxa/jitter, manda comandos
│   └── compare_racebox.py    compara CSV do device com export "Custom + Bike Mode" do app RaceBox
└── docs/SPEC.md              este documento
```

`lib/racebox_protocol` **não pode** incluir `Arduino.h` — roda em `pio test -e native` no CI.

---

## 10. Testes e critérios de aceite

### 10.1 Conformidade (CI, `pio test -e native`)

| Teste | Prova |
|---|---|
| Golden live frame | Construir o payload a partir dos campos decodificados do exemplo do vendor → **byte-idêntico** ao frame `B5 62 FF 01 50 00 … 06 DB` |
| Checksum vectors | Fletcher-8 reproduz todos os frames do fixture; frame adulterado é rejeitado |
| Respostas de comando | `0x22`, `0x23` (780 msgs), `0x26`, ACK do unlock (`B5 62 FF 02 02 00 FF 30 32 3A` — o PDF imprime `32 32`, typo do vendor; o fixture traz o checksum correto), preset `B5 62 FF 25 0C 00 01 00 1F 00 6D 05 1E 00 1E 00 2C 01 2B 15` — parse e re-encode byte-idênticos |
| Assembler | Re-chunking em todos os pontos de corte (N pequeno) + aleatório com seed fixa; concatenação; gotejamento 1 B; ruído; `B5 62` dentro do payload não dessincroniza; overflow limitado; **nunca** trava |
| Clamp | gyro 400 °/s → `+32767`, nunca negativo |

### 10.2 Bancada (hardware)

| # | Critério | Medida |
|---|---|---|
| A1 | Racevo encontra, conecta e classifica como **Mini S** | Tela de diagnóstico do Racevo |
| A2 | Primeiro frame < 1 s após subscribe | `rb_client.py` |
| A3 | Taxa exata: Δ`iTOW` = 100 ms (10 Hz) / 40 ms (25 Hz, M9N), **zero gaps em 30 min** com MTU 512 | `rb_client.py stats` |
| A4 | Comandos respondem < 500 ms | idem |
| A5 | Funciona com MTU 23 (fragmentado) sem perda | forçar MTU no cliente de teste |
| A6 | Parado e nivelado: gZ = 1000 ± 30 mg, gX/gY = 0 ± 30 mg, gyro ±0,5 °/s após calibração | 60 s de captura |
| A7 | Gravação → desconectar → reconectar → download completo com contagem e marcadores `0x26` corretos | Walkthrough do Racevo |
| A8 | Queda de energia durante gravação perde ≤ 2 s | arrancar o cabo |
| A9 | Erase com progresso + cancel | Racevo, tela de memória |
| A10 | M8N com rate 25 pedido → entrega 10 Hz e loga a limitação | console |
| A11 | Gravação no Racevo: o preset de backup é aceito (chip "backup" aparece); derrubar o BLE 3× durante a sessão → Racevo reconecta por serial, **sem gap de GNSS** e **sem** `0x26` novo no download posterior (uma run só) | Racevo + download |
| A12 | Download pelo walkthrough **com a gravação ativa** completa e o device continua gravando | Racevo |
| A13 | Cold start (backup do GNSS descarregado) com o Racevo gravando desde antes do fix → `tMs` da sessão monotônico, sem salto | Racevo + `.rbox` |
| A14 | Memória cheia: live segue, `0x25` recebe NACK e o Racevo grava normalmente sem o chip de backup | Racevo |

### 10.3 Pista

| # | Critério |
|---|---|
| P1 | 5+ voltas: tempos do Racevo vs app RaceBox real (montado junto) dentro de **±0,05 s** por volta |
| P2 | Overlay de trajetória e velocidade vs export "Custom + Bike Mode" do app RaceBox (`compare_racebox.py`) |
| P3 | Sinais dos eixos: frenagem forte, aceleração, curva em cada sentido — confirma §6.4 e fecha o `‹VERIFY›` nº 2 do Racevo para este hardware |

---

## 11. Fases

| Fase | Entrega | Critérios |
|---|---|---|
| **F0** | Repo, `platformio.ini`, partições, `lib/racebox_protocol` + suíte de conformidade verde no CI | §10.1 |
| **F1** | GNSS: detecção de módulo, config UBX, NAV-PVT a 10 Hz; console `gnss info`/`stats` | A10 |
| **F2** | BLE: advertising, GATT, DIS, live `0x01` só com GNSS (IMU = 0, gZ = 1000) | A1–A5 |
| **F3** | IMU: FIFO 1 kHz, média por época, calibração estática | A6 |
| **F4** | Comandos Mini S com memória vazia (`0x22`/`0x23`/`0x24`/`0x25` idempotente/`0x27`/`0x30`) — Racevo walkthrough completo e gravação ao vivo com backup armado | A4, A9 parcial, A11 parcial |
| **F5** | Gravação standalone + filtros + download + erase + power loss | A7–A9 |
| **F6** | Modo simulação; calibração dinâmica | demo M4 do Racevo em casa |
| **F7** | Pista + (se aprovado) NEO-M9N a 25 Hz | P1–P3, A3 @25 Hz |

---

## 12. Não-objetivos

- Protocolo NMEA via BLE.
- Escrever `.rbox` no dispositivo (é formato do Racevo, gerado no app).
- Fusão GNSS/IMU (dead reckoning) ou qualquer interpolação de posição.
- Upload Wi-Fi / OTA na v1 (a partição de OTA pode existir, a feature não).
- Compatibilidade com o protocolo próprio do repo anterior.

---

## 13. Decisões em aberto e `‹VALIDAR›`

| # | Item | Proposta |
|---|---|---|
| D1 | **Módulo GNSS para 25 Hz** | Trocar NEO-M8N → **NEO-M9N** (mesmo footprint). Até lá, 10 Hz real. |
| D2 | **Capacidade de memória** (~16 min @25 Hz no WROOM-32D) | Aceitar na v1; microSD ou ESP32-S3 N16R8 na v2 |
| D3 | Bateria (medição + carregador) | v1 reporta 100 % fixo; v2 com divisor + TP4056 |
| D4 | Identidade "RaceBox Mini S" no nome/DIS | Default para compatibilidade total; flag `GOAWAKE` disponível |
| D5 | Sinais de giro (roll/pitch/yaw) | Fechar no teste P3, junto com o `‹VERIFY›` nº 2 do Racevo |
| D6 | Id desconhecido | NACK. (Download durante gravação **não** é mais decisão: é obrigatório, R12) |
| D7 | Auto-shutdown: wake por timer além do botão? | Só botão na v1 |
| D8 | Segurança de memória (`0x30` real) | Fora da v1 (sempre ACK) |
| D9 | Constelações a 25 Hz no M9N | 4 GNSS; reduzir se `hAcc`/numSV degradar |
| D10 | Tempo de boot+config do GNSS vs timeout de 6 s do Racevo | **Medido (2026-10-20, M10 da bancada):** primeiro NAV-PVT a 53–130 ms do boot com o receptor já configurado (BBR) e baud em NVS; 1,96 s no pior caso visto (receptor a 9600/NMEA, autobaud + troca de baud + config). Pendente: power-on frio com o backup do módulo descarregado |
| D11 | Memória de ~16 min + backup armado em toda sessão (R11) → enche rápido e o Racevo não apaga sozinho | Aceitar na v1 (live não é afetado); reforça D2 |
| D12 | Teto de taxa `max_rate` (§5.3) para ter 10 Hz com o Racevo num M9N | Implementar; default = sem teto |
| D13 | Biblioteca GNSS | **Decidido (2026-10-20):** driver UBX próprio em vez da SparkFun u-blox GNSS v3. Motivo: o `FrameAssembler` já valida UBX; o NAV-PVT segue **cru** até o builder (o mapeamento NAV-PVT→RaceBox é testado byte a byte em native); montagem de CFG/VALSET e parse de MON-VER testáveis sem hardware; sem callback de biblioteca em contexto de outra task. Ver §5.4 |
| D14 | `0x25` com 10 B: qual campo falta? | **Decidido (2026-10-19):** o layout do vendor tem 12 B com o auto-shutdown no fim (offset 10); 10 B = os 10 primeiros, e o auto-shutdown **mantém o valor atual**. `‹VALIDAR›` contra um Mini S real (o Racevo manda 12) |
| D15 | Unidade do *min horizontal accuracy* do `0x27` | **Decidido (2026-10-19):** décimos de metro. A prosa do vendor diz "metros", mas o único exemplo em bytes (Rev 8 p.10, `B5 62 FF 27 03 00 08 01 14 46 20`, checksum válido) decodifica `0x14` = 20 como "2.0 m". Byte do vendor > prosa. Default 3,0 m = `30`. O Racevo não usa o `0x27`. `‹VALIDAR›` lendo o `0x27` de um Mini S real (esperado `04 00 1E`) |
| D16 | **Divergência de hardware:** a placa vendida como NEO-M8N tem um chip **u-blox M10** (SPG 5.10, PROTVER 34.10) | **Decidido (2026-10-20):** tratar como M10: config por `CFG-VALSET`, baud 115200, **teto de 10 Hz** (tabela §2.2: "10 Hz com 4 GNSS, ‹VALIDAR›"). Medido em bancada, **sem fix**: com `measRate` 40 ms o M10 entregou 416 NAV-PVT seguidos com Δ`iTOW` = 40 ms e zero gaps — isso **não** prova 25 Hz com fix (épocas sem solução são baratas). Constelações de fábrica deste módulo: GPS+Galileo+BeiDou (sem GLONASS); o firmware não mexe. `‹VALIDAR›` em campo com fix: 25 Hz com GPS+GAL; se Δ`iTOW` ficar em 40 ms com `hAcc`/numSV bons, subir o teto do M10 e documentar. Consequência: com esta placa, o Racevo recebe 10 Hz (igual ao plano do M8N) |

### Itens do checklist `‹VERIFY›` do Racevo (04 §11) que este hardware ajuda a fechar

- **nº 2** sinal dos eixos — P3 (com a ressalva: fecha para *este* hardware; o RaceBox real continua precisando do teste com o g-ball magenta).
- **nº 4** 10 vs 12 B no `0x25` — o firmware aceita os dois; o Racevo manda 12.
- **nº 5** Preset B (`0x1D`) — testável em bancada com o emulador antes do Mini S real.
