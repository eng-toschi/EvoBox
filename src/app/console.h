// Console serial (SPEC §9.2).
#pragma once

namespace console {

// Comando extra registrado por um módulo. `args` = resto da linha (pode ser "").
// Retorna false se não reconheceu os argumentos (o console imprime o uso).
using Handler = bool (*)(const char* args);

void add(const char* name, const char* usage, Handler h);
// Processa bytes pendentes do Serial; chamar periodicamente.
void poll();

}  // namespace console
