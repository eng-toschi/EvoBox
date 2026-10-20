# Execução noturna autônoma

Prompt para o Claude Code rodar sozinho, sem ninguém para responder perguntas. Iniciado com o comando em `README.md` (seção "Rodar a noite").

---

Você vai trabalhar **sozinho a noite toda**. Ninguém vai responder perguntas: nunca pare para perguntar — decida, registre e siga.

## Contexto

- Leia `CLAUDE.md` e `docs/SPEC.md` inteiros antes de começar. São o contrato.
- Você está no Mac do Fernando. A placa EvoBox (ESP32 + NEO-M8N + MPU6050) **deve** estar no USB (`/dev/cu.usbserial-*`). O Bluetooth do Mac está disponível para testes BLE com `bleak`.
- O GPS provavelmente está **dentro de casa, sem fix**. Tudo que depende de fix é testado no **modo simulação** (SPEC §9.3). Não fique esperando fix.

## Objetivo da noite

Avançar o máximo possível nas fases **F0 → F6** (SPEC §11), em ordem, com **testes reais**. F7 (pista, NEO-M9N) e os testes que exigem o app Racevo no celular ficam fora — liste-os no relatório.

## Fluxo

1. Crie a branch `night/<AAAA-MM-DD>` a partir do `main`. **Nunca** commite nem faça push em `main`.
2. Para cada fase:
   1. Implemente seguindo a spec e o layout da SPEC §9.5.
   2. Teste: `pio test -e native` sempre; `pio run -e esp32dev -t upload` + leitura do console serial (leia a serial com um script Python com `pyserial` e tempo limite — o macOS não tem `timeout`, e `pio device monitor` sem limite trava a sessão) para o que roda na placa; `tools/rb_client.py` (bleak) para tudo que é BLE — escreva-o já na F2.
   3. Verifique os critérios da SPEC §10 daquela fase. Para cada um: **provado** (como), **falhou** (o que viu) ou **pendente** (por quê — ex.: precisa do celular, de fix real ou de pista).
   4. Commit pequeno e convencional; `git push -u origin night/<data>` ao fim de cada fase.
   5. Atualize `docs/NIGHT_LOG.md` (ver abaixo) e commite junto.
3. Travou num problema: até **3 tentativas** com diagnóstico de verdade (logs, medições). Sem solução, registre no log com o que tentou e o que viu, e passe para o que **não depende** dele (outra parte da fase ou a próxima fase, se a dependência permitir). Nunca fique em loop.

## Proibido

- Afrouxar, pular ou apagar um teste para ficar verde. Teste vermelho fica vermelho e vai para o log.
- Editar `test/fixtures/racebox_test_vectors.json`.
- Interpolar/fabricar fixes GNSS, ou fingir no log que algo foi testado em hardware quando não foi.
- Mudar comportamento da spec sem registrar a decisão em `docs/SPEC.md` §13.
- Mandar UBX que reconfigure o GNSS de forma irreversível além do previsto na spec (salvar config em BBR/flash do módulo está previsto; reset de fábrica, não).
- `git push` em `main`, `--force`, `reset --hard`, `rm -rf`, `sudo`.

## Se a placa não estiver acessível

Se não houver `/dev/cu.usbserial-*` ou o upload falhar por porta ocupada/ausente: registre no log e faça tudo que for possível sem hardware — F0 completa, toda a lógica testável em `native` (assembler, payload builder, comandos, recorder/filtros com relógio injetado, log circular contra uma flash simulada, calibração com dados sintéticos), e `tools/rb_client.py` pronto.

## `docs/NIGHT_LOG.md` — relatório da manhã

Mantenha sempre atualizado (é o que o Fernando lê ao acordar):

```
# Night log <data>
## Resumo (5 linhas no máximo)
## Fases
### F0 — status: ✅/🟡/❌
- critério → provado/falhou/pendente — evidência
## Problemas abertos (com o que foi tentado)
## Decisões tomadas (e onde foram registradas na SPEC §13)
## O que o Fernando precisa fazer (testes no Racevo, pista, hardware)
## Como reproduzir (comandos)
```

Ao terminar (ou ao esgotar o que dá para fazer), faça o push final e encerre com o resumo.
