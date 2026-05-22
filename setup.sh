#!/usr/bin/env bash
# Setup del proyecto — instala dependencias segun el componente
#
# Uso:
#   ./setup.sh operator   # Mac (GCS + teleop brazo)
#   ./setup.sh robot      # Jetson Nano (control server)
#   ./setup.sh sim        # Simulacion (solo verifica Docker)

set -e

COMPONENT=${1:-""}

if [ -z "$COMPONENT" ]; then
    echo "Uso: ./setup.sh [operator|robot|sim]"
    echo ""
    echo "  operator  — Mac: instala dependencias de GCS y brazo leader"
    echo "  robot     — Jetson Nano: instala dependencias del servidor de control"
    echo "  sim       — Simulacion: verifica que Docker este instalado"
    exit 1
fi

# ─── OPERATOR (Mac) ────────────────────────────────────────────────────────────
if [ "$COMPONENT" = "operator" ]; then
    echo ">>> Instalando dependencias del operador (Mac)..."
    pip install -r operator/requirements.txt

    echo ">>> Instalando LeRobot (brazo SO101 leader)..."
    pip install -e src/

    echo ""
    echo "Listo. Para correr la GCS:"
    echo "  PYTHONPATH=src python operator/gcs_app.py --remote_ip=<IP_JETSON>"
fi

# ─── ROBOT (Jetson Nano) ───────────────────────────────────────────────────────
if [ "$COMPONENT" = "robot" ]; then
    echo ">>> Instalando dependencias del robot (Jetson Nano)..."
    pip install -r robot/requirements.txt

    echo ">>> Instalando LeRobot (brazo SO101 follower)..."
    pip install -e src/

    echo ""
    echo "Listo. Para correr el servidor:"
    echo "  PYTHONPATH=src python robot/control_server.py --robot.port=/dev/ttyACM0 --pico.port=/dev/ttyACM1"
fi

# ─── SIM (Docker) ──────────────────────────────────────────────────────────────
if [ "$COMPONENT" = "sim" ]; then
    echo ">>> Verificando Docker..."
    if ! command -v docker &> /dev/null; then
        echo "ERROR: Docker no esta instalado."
        echo "Instalar desde: https://www.docker.com/products/docker-desktop"
        exit 1
    fi

    echo "Docker: OK ($(docker --version))"
    echo ""
    echo "Para correr la simulacion:"
    echo "  docker compose -f sim/docker-compose.sim.yml up"
    echo ""
    echo "Nota: en Mac tambien requiere XQuartz:"
    echo "  brew install --cask xquartz"
    echo "  xhost + 127.0.0.1"
fi
