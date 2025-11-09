# OMNICONTROLLER_INTERFACE



![Language](https://img.shields.io/badge/Python-100%25-blue)
![ROS2](https://img.shields.io/badge/ROS2-Humble-%2300add8)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

## 📑 Table of Contents

- [Overview](#-overview)
- [Getting Started](#-getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
- [Usage](#-usage)
  - [Parameters](#parameters)
  - [Topics (ROS ↔ MQTT)](#topics-ros--mqtt)
  - [Controller Mappings](#controller-mappings)
- [Project Structure](#-project-structure)
- [Roadmap](#-roadmap)
- [License](#-license)

---

## 🧭 Overview

`omnicontroller_interface` é um pacote ROS 2 que implementa uma **interface bidirecional entre ROS 2 e MQTT**, desenvolvida para o **OmniCare Robot**.  
Ele permite que um microcontrolador (como o ESP32) envie comandos e receba estados via MQTT, enquanto o robô publica dados de status e bateria em tópicos ROS 2.  

### Principais funções:
- **Bridge ROS ↔ MQTT** (publica e recebe mensagens entre sistemas).
- **Leitura de bateria via INA219** e publicação como porcentagem.
- **Teleop leve** via mensagens MQTT.
- **Execução remota de nodes/launches** através de botões do controle.

> 🧩 Pacote principal: `ros2_mqtt`  
> 📄 Launch file: `launch/ros2_mqtt.launch.py`  
> ⚙️ Configurações: `config/params.yaml`

---

## ⚙️ Getting Started

### Prerequisites

- Ubuntu 22.04 com **ROS 2 Humble**
- Python ≥ 3.10
- Dependências:
  ```bash
  sudo apt install python3-paho-mqtt python3-smbus
  ```
- Bibliotecas ROS 2 usadas:
  - `rclpy`
  - `std_msgs`
  - `geometry_msgs`
  - `sensor_msgs`

> Hardware opcional: **INA219** conectado ao barramento I²C para leitura de bateria.

---

### Installation

```bash
# Dentro do seu workspace ROS 2:
cd ~/ros2_ws/src
git clone https://github.com/LucasLagoeiro/omnicontroller_interface.git
cd ..
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Usage

1. Edite os parâmetros MQTT e mapeamentos de controle no arquivo:
   ```bash
   config/params.yaml
   ```
2. Execute o launch file:
   ```bash
   ros2 launch ros2_mqtt ros2_mqtt.launch.py
   ```

### Parameters

Trecho do `config/params.yaml`:

```yaml
mqtt:
  broker_ip_address: "192.168.15.115"
  mqtt_state_pub_topic: "/esp32/navigation/state"
  mqtt_battery_pub_topic: "/esp32/status/battery"
  mqtt_sub_topic: "/esp32/omnicontroller"
  state_sub_topic: "/omnicare/navigation/state"
  battery_sub_topic: "/omnicare/status/battery"

controller:
  button:
    a: ""
    b: "run.omnicare_behavior.behavior_manager"
    start: "launch.omnicare_bringup.load_real_robot"
  teleop:
    right: -0.5
    left: 0.5
    up: 0.5
    down: -0.5
```

**Mapeamentos:**
- `run.<pkg>.<entrypoint>` → executa um nó Python.
- `launch.<pkg>.<launch_file>` → executa um launch file.

---

### Topics (ROS ↔ MQTT)

| Direção | ROS 2 Tópico | MQTT Tópico | Tipo | Descrição |
|----------|---------------|--------------|-------|------------|
| ROS → MQTT | `/omnicare/navigation/state` | `/esp32/navigation/state` | `std_msgs/String` | Estado atual do robô |
| ROS → MQTT | `/omnicare/status/battery` | `/esp32/status/battery` | `std_msgs/String` | Tensão e porcentagem |
| MQTT → ROS | `/esp32/omnicontroller` | `/omnicare/cmd_vel` (interno) | `geometry_msgs/Twist` | Comandos de movimento |

> Loop principal a **10 Hz**, leitura de bateria a **1 Hz**.

---

### Controller Mappings

| Botão | Ação | Tipo |
|-------|------|------|
| A | - | Nenhuma |
| B | `run.omnicare_behavior.behavior_manager` | Executa nó Python |
| X | - | Nenhuma |
| Y | - | Nenhuma |
| START | `launch.omnicare_bringup.load_real_robot` | Inicia launch |

---

## 🧪 Testing

Testes manuais:
```bash
ros2 topic pub -1 /omnicare/navigation/state std_msgs/String "{data: 'READY'}"
```
Verifique no MQTT (`mqtt_state_pub_topic`) se a mensagem é retransmitida corretamente. Exemplo: 
```bash
mosquitto_sub -h 192.168.0.129 -t /esp32/navigation/state
```


---

## 📂 Project Structure

```
omnicontroller_interface/
└── ros2_mqtt/
    ├── launch/
    │   └── ros2_mqtt.launch.py
    ├── config/
    │   └── params.yaml
    ├── ros2_mqtt/
    │   ├── relay_ros2_mqtt.py
    │   └── utils/
    │       ├── handlers.py
    │       ├── ina219.py
    │       └── launch_process.py
    ├── test/
    ├── package.xml
    ├── setup.py
    └── setup.cfg
```

## 🧭 Roadmap

- [ ] Passar para C++ o código.  
- [ ] Publicar pacote no ROS Index.

## 📜 License

Este projeto está licenciado sob os termos da **MIT License**.  
Consulte o arquivo [LICENSE](LICENSE) para mais detalhes.

---

**Autor:** [Lucas Lagoeiro](https://github.com/LucasLagoeiro)  
