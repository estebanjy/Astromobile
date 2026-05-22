#!/usr/bin/env python
"""
SO101 Remote Host - corre en la Jetson Nano.
Conecta el brazo follower SO101 y lo expone por ZMQ para teleoperación remota.

Uso en la Jetson:
    python -m lerobot.robots.so_follower.so101_host \
        --robot.port=/dev/ttyACM0 \
        --robot.id=follower
"""

import base64
import json
import logging
import time
from dataclasses import dataclass, field

import cv2
import draccus
import zmq

from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class SO101HostConfig:
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556
    watchdog_timeout_ms: int = 500
    max_loop_freq_hz: int = 30
    connection_time_s: int = 3600


@dataclass
class SO101ServerConfig:
    robot: SOFollowerRobotConfig = field(
        default_factory=lambda: SOFollowerRobotConfig(port="/dev/ttyACM0")
    )
    host: SO101HostConfig = field(default_factory=SO101HostConfig)


@draccus.wrap()
def main(cfg: SO101ServerConfig):
    logger.info("Conectando brazo follower SO101...")
    robot = SOFollower(cfg.robot)
    robot.connect()

    logger.info("Iniciando servidor ZMQ...")
    context = zmq.Context()

    cmd_socket = context.socket(zmq.PULL)
    cmd_socket.setsockopt(zmq.CONFLATE, 1)
    cmd_socket.bind(f"tcp://*:{cfg.host.port_zmq_cmd}")

    obs_socket = context.socket(zmq.PUSH)
    obs_socket.setsockopt(zmq.CONFLATE, 1)
    obs_socket.bind(f"tcp://*:{cfg.host.port_zmq_observations}")

    last_cmd_time = time.time()
    logger.info(
        f"Listo. Escuchando en puertos {cfg.host.port_zmq_cmd} (cmd) "
        f"y {cfg.host.port_zmq_observations} (obs)"
    )

    try:
        start = time.perf_counter()
        while time.perf_counter() - start < cfg.host.connection_time_s:
            loop_start = time.time()

            # Recibir comando del Mac
            try:
                msg = cmd_socket.recv_string(zmq.NOBLOCK)
                action = json.loads(msg)
                robot.send_action(action)
                last_cmd_time = time.time()
            except zmq.Again:
                pass
            except Exception as e:
                logger.error(f"Error procesando comando: {e}")

            # Watchdog: si no hay comandos por más de watchdog_timeout_ms, el brazo
            # simplemente mantiene su posición actual (control de posición).
            if time.time() - last_cmd_time > cfg.host.watchdog_timeout_ms / 1000:
                pass

            # Leer observación y enviarla al Mac
            try:
                obs = robot.get_observation()
                obs_serializable = {}
                for key, val in obs.items():
                    if hasattr(val, "shape"):  # numpy array = imagen de cámara
                        ret, buf = cv2.imencode(
                            ".jpg", val, [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                        )
                        obs_serializable[key] = (
                            base64.b64encode(buf).decode("utf-8") if ret else ""
                        )
                    else:
                        obs_serializable[key] = float(val)

                obs_socket.send_string(json.dumps(obs_serializable), flags=zmq.NOBLOCK)
            except zmq.Again:
                pass
            except Exception as e:
                logger.error(f"Error enviando observación: {e}")

            # Control de frecuencia
            elapsed = time.time() - loop_start
            time.sleep(max(1 / cfg.host.max_loop_freq_hz - elapsed, 0))

    except KeyboardInterrupt:
        logger.info("Interrupción por teclado. Apagando...")
    finally:
        robot.disconnect()
        obs_socket.close()
        cmd_socket.close()
        context.term()
        logger.info("SO101 host apagado correctamente.")


if __name__ == "__main__":
    main()
