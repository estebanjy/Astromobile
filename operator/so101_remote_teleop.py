#!/usr/bin/env python
"""
SO101 Teleoperación Remota - corre en el Mac.
Lee el brazo leader SO101 y envía comandos al follower en la Jetson por ZMQ.

Uso en el Mac:
    python so101_remote_teleop.py \
        --leader_port=/dev/tty.usbmodem... \
        --remote_ip=192.168.1.17

Opciones:
    --leader_port   Puerto USB del brazo leader en el Mac
    --leader_id     ID del leader (default: leader)
    --remote_ip     IP de la Jetson
    --port_cmd      Puerto ZMQ de comandos (default: 5555)
    --port_obs      Puerto ZMQ de observaciones (default: 5556)
    --fps           Frecuencia de teleoperación (default: 30)
"""

import argparse
import json
import logging
import time

import zmq

from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderTeleopConfig
from lerobot.teleoperators.so_leader.so_leader import SOLeader

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main():
    parser = argparse.ArgumentParser(description="SO101 teleoperación remota vía ZMQ")
    parser.add_argument(
        "--leader_port", required=True, help="Puerto USB del leader en el Mac"
    )
    parser.add_argument("--leader_id", default="leader", help="ID del leader")
    parser.add_argument("--remote_ip", required=True, help="IP de la Jetson")
    parser.add_argument(
        "--port_cmd", type=int, default=5555, help="Puerto ZMQ comandos"
    )
    parser.add_argument(
        "--port_obs", type=int, default=5556, help="Puerto ZMQ observaciones"
    )
    parser.add_argument(
        "--fps", type=int, default=30, help="Frecuencia de teleoperación"
    )
    args = parser.parse_args()

    # Conectar leader
    logger.info(f"Conectando leader en {args.leader_port}...")
    config = SOLeaderTeleopConfig(port=args.leader_port, id=args.leader_id)
    leader = SOLeader(config)
    leader.connect()
    logger.info("Leader conectado.")

    # Conectar ZMQ al host en la Jetson
    context = zmq.Context()

    cmd_socket = context.socket(zmq.PUSH)
    cmd_socket.setsockopt(zmq.CONFLATE, 1)
    cmd_socket.connect(f"tcp://{args.remote_ip}:{args.port_cmd}")

    obs_socket = context.socket(zmq.PULL)
    obs_socket.setsockopt(zmq.CONFLATE, 1)
    obs_socket.connect(f"tcp://{args.remote_ip}:{args.port_obs}")

    logger.info(
        f"Conectado al follower en {args.remote_ip}. Iniciando teleoperación..."
    )
    logger.info("Presiona Ctrl+C para detener.")

    try:
        while True:
            loop_start = time.time()

            # Leer posición del leader
            action = leader.get_action()

            # Enviar al follower en la Jetson
            try:
                cmd_socket.send_string(json.dumps({"arm": action}), flags=zmq.NOBLOCK)
            except zmq.Again:
                pass

            # Control de frecuencia
            elapsed = time.time() - loop_start
            time.sleep(max(1 / args.fps - elapsed, 0))

    except KeyboardInterrupt:
        logger.info("Deteniendo teleoperación...")
    finally:
        leader.disconnect()
        cmd_socket.close()
        obs_socket.close()
        context.term()
        logger.info("Teleoperación finalizada.")


if __name__ == "__main__":
    main()
