# 🤖 Robot Obstacle Detection Simulator

Simulador de robô autônomo com detecção de obstáculos usando PyBullet e controle remoto via MQTT.

## 🎯 Objetivo
Desenvolver um robô que detecta obstáculos e para automaticamente, com controle remoto via MQTT.

## 🔧 Tecnologias
- **PyBullet** - Simulação física 3D
- **MQTT** - Comunicação IoT
- **NumPy** - Cálculos matemáticos
- **Threading** - Processamento paralelo

## 🏗️ Arquitetura
- **Ambiente**: Sala 16x16 com obstáculos aleatórios
- **Sensores**: Ray casting
- **Movimento**: Controle diferencial (4 rodas)
- **Comunicação**: MQTT assíncrono

## 🚀 Funcionalidades
- ✅ Detecção de obstáculos em tempo real
- ✅ Parada automática antes de colisão
- ✅ Controle remoto (frente/trás/esquerda/direita)
- ✅ Comunicação via MQTT
- ❌ Controle de trajetória (não implementado)

## 📦 Instalação
```bash
pip install -r requirements.txt
```

## 🎮 Uso
```bash
python main.py
```

**Comandos MQTT:**
- `front` - Mover para frente
- `back` - Mover para trás  
- `left/right` - Girar
- `stop` - Parar

**Topicos MQTT:**
- `robot/command` - Movimentação do robo
- `robot/status` - Status indicando colisão
- `robot/action` - Ultima ação executada
- 
## 📊 Configuração MQTT
```python
MQTT_BROKER = 'localhost'
MQTT_PORT = 1883
MQTT_USER = 'edu'
MQTT_PASSWORD = 'eduardo'
```

## 🔮 Limitações
- Sem planejamento de trajetória
- Sem algoritmo de desvio
- Dependente de comandos externos

---
*Desenvolvido para apresentação acadêmica - Detecção de obstáculos implementada com sucesso*