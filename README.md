# Servico1 — Projeto Arduino (Simulado e Físico)

## Descrição

Este repositório contém dois sketches para o projeto "Servico1":

- `Fisico/redes4.ino` — Sketch destinado a execução em hardware (placa física).
- `Simulado/Servico1.ino` — Sketch para desenvolvimento e testes em ambiente simulado.

O README foi atualizado para incluir um resumo do funcionamento documentado em `Servico1.pdf` e instruções para instalar a biblioteca Bluepad32 (usada para integração com gamepads via Bluetooth), que está disponível em: https://gitlab.com/ricardoquesada/bluepad32

## Resumo do funcionamento (conforme `Servico1.pdf`)

Observação: o PDF `Servico1.pdf` descreve o objetivo, arquitetura e fluxo de execução do projeto. Abaixo há um resumo em alto nível — consulte o PDF para diagramas, detalhes de algoritmos e casos de teste.

- Objetivo: implementar o serviço "Servico1" que realiza [comunicação/controle/processamento] entre módulos de simulação e hardware (veja o PDF para o contexto e requisitos específicos).
- Fluxo básico:
  - Inicialização: o sketch configura a serial, interfaces e variáveis de estado.
  - Comunicação: o sistema troca mensagens (via Serial/Bluetooth/Wi‑Fi — veja o sketch e o PDF) para receber comandos e enviar status.
  - Lógica principal: o loop principal processa eventos/entradas e aciona saídas (pinos, atuadores, resposta em rede).
  - Testes/exemplos: o PDF inclui cenários de teste e exemplos de uso para as versões `Simulado` e `Fisico`.

Notas sobre o conteúdo do PDF:
- Para detalhes de requisitos, diagrama de blocos, algoritmos e instruções de montagem (hardware), abra `Servico1.pdf`.
- O PDF não descreve a integração com Bluepad32 — essa integração foi adicionada separadamente e está documentada abaixo.

## Bluepad32 — uso e instalação

O projeto usa (ou pode usar) a biblioteca Bluepad32 para integrar gamepads Bluetooth (ex.: DualShock, Xbox, Switch Pro) com placas baseadas em ESP32. O repositório oficial está aqui:

https://gitlab.com/ricardoquesada/bluepad32

No repositório do Bluepad32 há exemplos e um vídeo demonstrando o funcionamento — consulte o README do projeto para ver o vídeo e exemplos práticos.

Como instalar a Bluepad32 (opções, Windows / PowerShell):

Opção A — Instalar via Arduino IDE (arquivo ZIP)
1. Abra o repositório no navegador: https://gitlab.com/ricardoquesada/bluepad32
2. Clique em `Download` -> `Download repository` para obter o ZIP.
3. No Arduino IDE: `Sketch > Include Library > Add .ZIP Library...` e selecione o ZIP baixado.

Opção B — Clonar diretamente na pasta de bibliotecas do Arduino (recomendado para desenvolvimento):

Abra o PowerShell e execute:

```powershell
# crie a pasta libraries se não existir
mkdir "$env:USERPROFILE\Documents\Arduino\libraries" -ErrorAction SilentlyContinue
cd "$env:USERPROFILE\Documents\Arduino\libraries"
git clone https://gitlab.com/ricardoquesada/bluepad32.git
```

Depois de clonado, o Arduino IDE deve enxergar a biblioteca automaticamente (feche e reabra o IDE se necessário).

Opção C — PlatformIO
- No `platformio.ini` do seu projeto, adicione (exemplo):

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = https://gitlab.com/ricardoquesada/bluepad32.git
```

Observações importantes
- Bluepad32 depende do core ESP32 (instale o suporte a placas ESP32 no Arduino IDE: `Tools > Board > Boards Manager` e instale `esp32` por Espressif).
- Veja a pasta `examples` do repositório Bluepad32 para sketches demonstrativos — copie/adapte-os para entender como integrar com seu sketch `Servico1`.

## Como integrar Bluepad32 ao `Servico1`

1. Instale a Bluepad32 conforme uma das opções acima.
2. Abra o sketch alvo (`Fisico/redes4.ino` ou `Simulado/Servico1.ino`) no Arduino IDE.
3. No topo do sketch, inclua as headers necessárias (ex.: `#include <Bluepad32.h>`). Se o seu sketch já usa `#include` para Bluepad32, verifique se a biblioteca foi instalada corretamente.
4. Adapte os callbacks/exemplos do Bluepad32 (ex.: leitura de botões/joystick) para enviar comandos ao fluxo principal do `Servico1` (por Serial, rede ou lógica interna).

Exemplo conceitual (pseudo):

```cpp
// no setup()
Bluepad32.begin();

// no loop() — ler estado do gamepad e transformar em comando
if (gamepad.connected()) {
  auto x = gamepad.axisX();
  auto y = gamepad.axisY();
  // converter para comandos do Servico1
}
```

Consulte os exemplos da Bluepad32 para detalhes de API e callbacks.

## Como abrir, compilar e enviar

1. Abra o Arduino IDE.
2. Selecione `File > Open...` e escolha `Fisico\\redes4.ino` ou `Simulado\\Servico1.ino`.
3. Se usar Bluepad32, confirme que a biblioteca foi instalada e que você selecionou uma placa ESP32 apropriada em `Tools > Board`.
4. Compile e envie.

Exemplo rápido com Arduino CLI (PowerShell):

```powershell
# exemplo, ajuste --fqbn e porta
arduino-cli compile --fqbn esp32:esp32:esp32 "c:\Users\lickc\Documents\Arduino\Servico1\Simulado\Servico1.ino"
arduino-cli upload -p COM3 --fqbn esp32:esp32:esp32 "c:\Users\lickc\Documents\Arduino\Servico1\Simulado\Servico1.ino"
```

## Testes e debug

- Abra o Serial Monitor (`Tools > Serial Monitor`) para ver logs e mensagens de debug.
- Se usar Bluepad32, veja as mensagens de inicialização do Bluepad32 e logs de conexão do gamepad.
- Se não conseguir compilar: verifique se o core ESP32 está instalado e se a biblioteca Bluepad32 foi colocada em `Documents\\Arduino\\libraries`.

## Contribuindo

- Documente pinos, diagrama de conexão e dependências (bibliotecas) diretamente nos sketches.
- Adicione um `docs/hardware.md` com fotos/diagramas das ligações do hardware real.

## Licença

Adicione um arquivo `LICENSE` com a licença que preferir (ex.: MIT). Atualmente não há licença adicionada a este repositório.

---
Arquivo atualizado: `README.md` — verifique e ajuste os trechos do resumo do PDF caso queira mais detalhes extraídos diretamente do `Servico1.pdf`.
