# Night log 2026-10-19

## Resumo (5 linhas no máximo)
- F0 ✅: `lib/racebox_protocol` + 44 testes native verdes (conformidade byte a byte com os golden vectors do Racevo).
- Em andamento: F1.

## Fases

### F0 — status: ✅
- Golden live frame (campos decodificados → bytes idênticos a `B5 62 FF 01 50 00 … 06 DB`) → **provado** — `test_conformance` (`test_golden_live_frame_built_from_decoded_fields_is_byte_identical`); também pelo caminho NAV-PVT → payload (`test_payload/test_navpvt_path_reproduces_golden_frame`).
- Checksum vectors (todos os 10 do fixture validam; cada byte adulterado é rejeitado) → **provado** — `test_checksum_vectors_*`, `test_live_frame_tampered_is_rejected`.
- Respostas de comando (`0x23` 780, ACK do unlock, preset A, erase 59 %, unlock 123456: parse + re-encode idênticos; `0x22`/`0x26` round-trip) → **provado** — `test_conformance`. Extra: os exemplos em bytes do PDF do vendor para `0x22`, `0x26` e `0x27` (que o fixture não tem) decodificam e re-encodam idênticos; o checksum de cada um prova a transcrição.
- Assembler (todos os pontos de corte, aleatório com seed fixa, concatenação, gotejamento 1 B, ruído, `B5 62` dentro do payload, corrupção, len > 504, overflow limitado, nunca trava) → **provado** — `test_assembler` (15 casos).
- Clamp (gyro 400 °/s → `+32767`, nunca negativo; bytes `FF 7F` no fio) → **provado** — `test_clamp_gyro_400dps_*`, `test_motion_clamp_never_wraps`.
- CI: `.github/workflows/native.yml` roda `pio test -e native` em push/PR → resultado do primeiro run: ver seção "Como reproduzir".
- Fixture conferido byte a byte contra `racevo/docs/fixtures/racebox_test_vectors.json` no commit `033bcab` (idêntico, intocado).

## Problemas abertos (com o que foi tentado)
- (nenhum até agora)

## Decisões tomadas (e onde foram registradas na SPEC §13)
- **D14** — `0x25` com 10 B: o campo que falta é o último (auto-shutdown), que mantém o valor atual. SPEC §7.2 + §13.
- **D15** — `0x27` min hAcc em **décimos de metro** (exemplo em bytes do vendor: `0x14` = 2,0 m); default `30`. SPEC §7.6 + §13. ⚠️ diverge do texto antigo da SPEC ("m").
- SPEC §10.1 corrigida: o ACK do unlock é `… 32 3A` (o fixture está certo; `32 32` é typo do PDF).
- `.gitignore` tinha `*.csv`, que ignorava `partitions.csv` — exceção adicionada.

## O que o Fernando precisa fazer (testes no Racevo, pista, hardware)
- (preenchido ao longo da noite)

## Como reproduzir (comandos)
```bash
pio test -e native        # 44 casos: test_conformance, test_assembler, test_payload
pio run -e esp32dev
```
Observação: a placa apareceu em `/dev/cu.usbserial-110` (não `-220` como na SPEC/CLAUDE.md); o PlatformIO detecta sozinho.
