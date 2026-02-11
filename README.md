# Sistema de Identificação de Movimento

Este projeto implementa um **Sistema de Identificação de Movimento** utilizando um microcontrolador **Raspberry Pi Pico / Pico W**, sensores inerciais (IMU) e **Machine Learning embarcado** com **TensorFlow Lite for Microcontrollers**.

O sistema é capaz de:
- Coletar dados de movimento via sensor IMU
- Armazenar e organizar amostras para treinamento
- Treinar um modelo de Machine Learning em ambiente externo (Google Colab)
- Converter o modelo treinado para **.tflite**
- Integrar o modelo ao firmware embarcado para inferência em tempo real

---

## 🧠 Visão Geral do Funcionamento

O projeto é dividido em **duas etapas principais**:

### 1️⃣ Coleta de Dados (`collect_data.c`)
Responsável por:
- Inicializar o sensor IMU via **I2C**
- Realizar leituras de aceleração e giroscópio
- Enviar os dados via **Serial (USB)** para coleta externa
- Gerar dados rotulados para treinamento do modelo

Esses dados são posteriormente salvos em arquivos `.csv` e utilizados no treinamento.

---

### 2️⃣ Inferência em Tempo Real (`main.c`)
Responsável por:
- Inicializar periféricos (IMU, LEDs, USB Serial)
- Carregar o modelo TensorFlow Lite embarcado (`model.h`)
- Executar inferência com dados do sensor
- Identificar padrões de movimento
- Indicar o movimento reconhecido (ex: LEDs ou mensagens no terminal)

---

## 🧪 Treinamento do Modelo (Google Colab)

O treinamento é realizado externamente utilizando **Python + TensorFlow**, seguindo o fluxo:

1. Importação dos dados coletados (`.csv`)
2. Pré-processamento (normalização / janelas de tempo)
3. Treinamento do modelo de classificação
4. Conversão para **TensorFlow Lite (.tflite)**
5. Geração do arquivo `model.h` com `xxd` ou script auxiliar

O arquivo `model.h` é então incluído diretamente no firmware.

---

## ⚙️ Tecnologias Utilizadas

- **Raspberry Pi Pico / Pico W**
- **C / Pico SDK**
- **I2C**
- **Sensor IMU (ex: MPU6050)**
- **TensorFlow Lite for Microcontrollers**
- **Python (Treinamento)**
- **Google Colab**

---

## 🚀 Como Compilar e Executar

### Pré-requisitos
- Pico SDK configurado
- CMake
- Toolchain ARM GCC

### Passos básicos
```bash
mkdir build
cd build
cmake ..
make
```

Após a gravação:
- Use `collect_data.c` para gerar os dados
- Treine o modelo no Colab
- Gere o `model.h`
- Compile novamente usando `main.c`

---

## 📊 Aplicações Possíveis

- Monitoramento de movimentos
- Sistemas embarcados inteligentes
- Interfaces homem-máquina (HMI)
- Projetos educacionais de TinyML

---

## 📌 Observações Importantes

- O modelo deve ser pequeno o suficiente para caber na RAM/Flash do Pico
- A taxa de amostragem do sensor impacta diretamente a acurácia
- LEDs e mensagens seriais são usados para debug e validação
