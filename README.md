# Sistema de Aquisição Trifásico - STM32G474

Sistema embarcado para aquisição e análise de sinais trifásicos em tempo real, desenvolvido para STM32G474 com capacidade de medição de tensões e correntes AC.

## 📋 Visão Geral

Este projeto implementa um sistema completo de aquisição de dados para redes elétricas trifásicas, com as seguintes características:

- **Aquisição simultânea** de 7 canais analógicos (3 tensões + 3 correntes + neutro)
- **Taxa de amostragem** de 7.68 kHz sincronizada por hardware
- **Processamento em tempo real** com PLLs para sincronização de fase
- **Transmissão via UART** dos dados coletados
- **Detecção automática de falhas** e recuperação de ADCs

## 🔧 Especificações Técnicas

### Hardware
- **Microcontrolador**: STM32G474CEUx
- **Clock do sistema**: 170 MHz (modo boost)
- **ADCs**: 4 unidades (ADC1-4) com resolução de 12 bits
- **Timer**: TIM1 configurado para trigger preciso a 7.68 kHz
- **Comunicação**: UART4 a 1.382.400 baud
- **CORDIC**: Para cálculos trigonométricos otimizados

### Canais de Aquisição
| Canal | Sinal | ADC | Pino |
|-------|-------|-----|------|
| CH_VA | Tensão Fase A | ADC1_CH1 | PA0 |
| CH_IA | Corrente Fase A | ADC1_CH2 | PA1 |
| CH_VB | Tensão Fase B | ADC2_CH3 | PA2 |
| CH_IB | Corrente Fase B | ADC2_CH4 | PA3 |
| CH_VC | Tensão Fase C | ADC3_CH1 | PB1 |
| CH_IC | Corrente Fase C | ADC3_CH5 | PB13 |
| CH_IN | Corrente Neutro | ADC4_CH3 | PB12 |

### Algoritmos Implementados
- **PLL (Phase-Locked Loop)** para cada fase com sincronização adaptativa
- **Janelas de dados**: 1 ciclo (128 amostras) e 12 ciclos (1536 amostras)
- **Compactação de dados**: 2 amostras de 12 bits em 3 bytes
- **CRC16-CCITT** para verificação de integridade

## 📂 Estrutura do Projeto

```
IC_Trifasico/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32g4xx_hal_conf.h
│   │   └── stm32g4xx_it.h
│   └── Src/
│       ├── main.c                 # Código principal otimizado
│       ├── stm32g4xx_hal_msp.c    # Configuração MSP
│       ├── stm32g4xx_it.c         # Handlers de interrupção
│       └── system_stm32g4xx.c     # Configuração do sistema
├── Drivers/                       # Drivers HAL da ST
├── cmake/                         # Configurações do CMake
├── CMakeLists.txt                 # Build system
├── CMakePresets.json              # Presets de configuração
└── *.ioc                         # Configuração do STM32CubeMX
```

## 🚀 Como Compilar

### Pré-requisitos
- **STM32CubeCLT** 1.17.0 ou superior
- **CMake** 3.28 ou superior
- **Ninja** build system
- **ARM GCC Toolchain** (incluído no STM32CubeCLT)

### Instruções

1. **Clone o repositório**:
```bash
git clone https://github.com/K2sot/IC_Trifasico.git
cd IC_Trifasico
```

## 📊 Protocolo de Comunicação

### Formato dos Frames (250 bytes)
- **Header** (2 bytes): Identificação do tipo de dados
- **Sample Number** (4 bytes): Número da amostra global
- **Overflow Flag** (1 byte): Indicador de overflow do contador
- **Timestamp** (4 bytes): Timestamp de sincronização
- **Dados** (192 bytes): 128 amostras compactadas (2-em-3)
- **Frequência** (4 bytes): Frequência instantânea (IEEE754)
- **Fase** (4 bytes): Informação de fase (IEEE754)
- **Preenchimento** (até 244 bytes): Espaço reservado
- **Sequência** (2 bytes): Número de sequência
- **Canal ID** (1 byte): Identificador do canal
- **Reservado** (1 byte)
- **CRC16** (2 bytes): Verificação de integridade

### Headers por Tipo de Dados
| Tipo | Header | Descrição |
|------|--------|-----------|
| Tensão A | 0xAA 0x01 | Dados de 1 ciclo - Tensão Fase A |
| Tensão B | 0xAA 0x02 | Dados de 1 ciclo - Tensão Fase B |
| Tensão C | 0xAA 0x03 | Dados de 1 ciclo - Tensão Fase C |
| Corrente A | 0xAA 0x11 | Dados de 1 ciclo - Corrente Fase A |
| Corrente B | 0xAA 0x12 | Dados de 1 ciclo - Corrente Fase B |
| Corrente C | 0xAA 0x13 | Dados de 1 ciclo - Corrente Fase C |
| Neutro | 0xAA 0x1F | Dados de 1 ciclo - Corrente Neutro |
| Info Freq | 0xAA 0xF0 | Informações de frequência e timing |
| Debug | 0xAA 0x00 | Mensagens de debug/status |

## ⚙️ Configuração dos PLLs

Cada fase possui um PLL independente com os seguintes parâmetros:

```c
// Parâmetros padrão do PLL
const float KP = 0.023137f;      // Ganho proporcional
const float KI = 0.00026773f;   // Ganho integral  
const float BW = 20.0f;         // Largura de banda do filtro (Hz)
const float FPCT = 0.08f;       // Excursão de frequência ±8%
```

### Características do PLL
- **Frequência nominal**: 60 Hz
- **Faixa de operação**: 55.2 Hz a 64.8 Hz (±8%)
- **Detector de fase**: Quadratura (coseno)
- **Filtro passa-baixas**: IIR de primeira ordem
- **Anti-windup**: Limitação do integrador

## 🔍 Monitoramento e Debug

### Mensagens de Status
O sistema envia periodicamente informações de diagnóstico:
- **Contadores de callback** dos ADCs
- **Status de travamento** dos conversores
- **Recuperação automática** de falhas
- **Informações de frequência** das três fases

### LEDs e Indicadores
- **Sistema funcionando**: Heartbeat via UART
- **Falhas de ADC**: Mensagens de erro automáticas
- **Recuperação**: Log de reinicializações

## 🛠️ Desenvolvimento

### Estrutura de Pastas Ignoradas
O arquivo `.gitignore` está configurado para ignorar:
- Arquivos de build do CMake
- Cache de configuração
- Diretórios Debug/Release
- Arquivos temporários

### Configuração do VS Code
O projeto inclui tasks configuradas para:
- **Build limpo**: CMake clean rebuild
- **Flash via SWD**: Programação direta
- **Listagem de interfaces**: Debug de comunicação

## 📝 Licença

Este projeto está licenciado sob os termos da STMicroelectronics. Consulte o arquivo LICENSE para mais detalhes.

## 👥 Contribuição

Desenvolvido como parte do Trabalho de Conclusão de Curso (TCC) na UTFPR.

### Autor
- **Kevin** - Desenvolvimento principal

### Instituição
- **UTFPR** - Universidade Tecnológica Federal do Paraná
- **Departamento**: Iniciação Científica (IC)

## 📞 Suporte

Para questões técnicas ou sugestões, abra uma issue no repositório do GitHub.

---
*Sistema desenvolvido para aplicações de monitoramento de qualidade de energia elétrica em redes trifásicas.*
