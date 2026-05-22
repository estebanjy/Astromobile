# lerobot_on_track

Robot móvil diferencial con brazo manipulador SO101, navegación autónoma y teleoperación remota.

## Hardware

| Componente | Detalle |
|---|---|
| Placa de control | Jetson Nano 2GB |
| Microcontrolador | Raspberry Pi Pico (RP2040) |
| Base móvil | 4 motores DC + driver BTS7960 |
| Brazo | SO101 follower (Feetech via USB) |
| Cámaras | CSI + USB |
| Comunicación | ZMQ sobre WiFi |

## Estructura del proyecto

```
lerobot_on_track/
├── robot/
│   └── control_server.py       # Servidor principal — corre en la Jetson Nano
│                               # Integra: brazo SO101, base móvil, cámaras, ZMQ
├── operator/
│   ├── gcs_app.py              # Ground Control Station — corre en Mac (PyQt6)
│   └── so101_remote_teleop.py  # Teleoperación del brazo SO101 vía ZMQ
├── firmware/
│   ├── main.py                 # Firmware MicroPython para el Pico
│   └── pico_firmware.ino       # Alternativa en C++ / Arduino
├── sim/
│   ├── docker-compose.sim.yml  # Levanta la simulación completa
│   ├── docker/
│   │   └── Dockerfile.sim      # ROS 2 Humble + Gazebo + Nav2
│   └── robot_ws/               # Paquete ROS 2 (robot_sim)
│       └── src/robot_sim/
│           ├── launch/simulation.launch.py
│           ├── urdf/robot.urdf.xacro
│           ├── worlds/outdoor.world
│           └── config/
│               ├── nav2_params.yaml
│               └── ekf.yaml
└── src/lerobot/                # Biblioteca LeRobot (dependencia del brazo)
```

## Comunicación ZMQ

| Puerto | Dirección | Contenido |
|---|---|---|
| 5555 | GCS → Robot (PUSH/PULL) | Comandos JSON: `arm` + `base` |
| 5556 | Robot → GCS (PUSH/PULL) | Observaciones: `arm`, `odom`, `camera_csi`, `camera_usb` |

```json
// Comando (GCS → Robot)
{
  "arm":  { "shoulder_pan": 1.57, "..." : "..." },
  "base": { "vx": 0.5, "omega": 0.0 }
}

// Observación (Robot → GCS)
{
  "arm":  { "shoulder_pan": 1.57, "...": "..." },
  "odom": { "x": 0.0, "y": 0.0, "theta": 0.0 },
  "mode": "RC"
}
```

## Uso

### 1. Robot (Jetson Nano)

```bash
python robot/control_server.py \
    --robot.port=/dev/ttyACM0 \
    --robot.id=follower
```

### 2. Operador (Mac)

```bash
conda activate lerobot
PYTHONPATH=src python operator/gcs_app.py \
    --remote_ip=<IP_JETSON> \
    --leader_port=/dev/tty.usbmodem...
```

Controles de teclado (base móvil):
- `W / S` — adelante / atrás
- `A / D` — giro izquierda / derecha
- `Espacio` — parar

### 3. Simulación (Mac con Docker)

```bash
# Requisitos: Docker Desktop + XQuartz
xhost + 127.0.0.1

docker compose -f sim/docker-compose.sim.yml up
```

Acceder a RViz / Gazebo vía noVNC: `http://localhost:6080`

## Setup

Clonar el repo y correr el script de setup según el componente:

```bash
git clone <repo>
cd lerobot_on_track
```

### Mac (operador)

```bash
conda activate lerobot
./setup.sh operator
```

Instala: `pyzmq`, `pyqt6`, `numpy`, `opencv-python` y LeRobot desde `src/`.

### Jetson Nano

```bash
./setup.sh robot
```

Instala: `pyzmq`, `opencv-python`, `pyserial`, `draccus` y LeRobot desde `src/`.

### Simulación

```bash
./setup.sh sim   # verifica que Docker esté instalado
```

Requiere además en Mac:
```bash
brew install --cask xquartz
```

### Firmware (Pico)

En el IDE de Arduino:
1. Instalar board: **Raspberry Pi Pico/RP2040** by Earle Philhower
2. Instalar librería: **ArduinoJson >= 7.x** (Library Manager)
3. Abrir y flashear `firmware/pico_firmware.ino`
