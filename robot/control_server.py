#!/usr/bin/env python
"""
Control Server - corre en la Jetson Nano.
Integra:
  - Brazo SO101 follower (Feetech via USB)
  - Base movil (4 motores DC via Pico por serial USB)
  - Camara CSI (cv2.VideoCapture, index configurable)
  - Camara USB (cv2.VideoCapture, index configurable)
  - Servidor ZMQ para GCS (Mac)

Uso:
    python control_server.py \
        --robot.port=/dev/ttyACM0 \
        --robot.id=follower

Puertos ZMQ:
  5555 PULL: recibe comandos del GCS (brazo + base)
  5556 PUSH: envia observaciones al GCS (brazo + odometria + camaras)

Protocolo comando (JSON):
  {
    "arm": {"shoulder_pan": 1.57, ...},   # opcional
    "base": {"vx": 0.5, "omega": 0.0}    # opcional
  }

Protocolo observacion (JSON):
  {
    "arm": {"shoulder_pan": 1.57, ...},
    "odom": {"x": 0.0, "y": 0.0, "theta": 0.0},
    "mode": "RC",
    "camera_csi": "<base64_jpeg_or_empty>",
    "camera_usb": "<base64_jpeg_or_empty>"
  }
"""

import base64
import json
import logging
import math
import threading
import time
from dataclasses import dataclass, field

import cv2
import draccus
import serial
import zmq

from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Configuracion
# ---------------------------------------------------------------------------
@dataclass
class CameraConfig:
    csi_index: int = 0  # indice cv2 para camara CSI
    usb_index: int = 1  # indice cv2 para camara USB
    width: int = 640
    height: int = 480
    fps: int = 30
    jpeg_quality: int = 80


@dataclass
class PicoConfig:
    port: str = "/dev/ttyACM1"  # Puerto serial del Pico (ACM0 suele ser el brazo)
    baudrate: int = 115200
    timeout: float = 0.1


@dataclass
class ServerConfig:
    port_zmq_cmd: int = 5555
    port_zmq_obs: int = 5556
    max_loop_hz: int = 30
    connection_time_s: int = 86400  # 24 horas


@dataclass
class ControlServerConfig:
    robot: SOFollowerRobotConfig = field(
        default_factory=lambda: SOFollowerRobotConfig(port="/dev/ttyACM0")
    )
    pico: PicoConfig = field(default_factory=PicoConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    server: ServerConfig = field(default_factory=ServerConfig)


# ---------------------------------------------------------------------------
# Camara
# ---------------------------------------------------------------------------
class CameraStream:
    """Captura de camara en hilo separado. Tolerante a camara ausente."""

    def __init__(self, index: int, width: int, height: int, fps: int, name: str):
        self.name = name
        self.index = index
        self.width = width
        self.height = height
        self.fps = fps
        self._frame = None
        self._lock = threading.Lock()
        self._running = False
        self._cap = None
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        logger.info(f"Abriendo camara {self.name} (index={self.index})...")
        cap = cv2.VideoCapture(self.index)
        if not cap.isOpened():
            logger.warning(
                f"Camara {self.name} no disponible (index={self.index}). Continuando sin ella."
            )
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        logger.info(f"Camara {self.name} abierta.")

        while self._running:
            ret, frame = cap.read()
            if ret:
                with self._lock:
                    self._frame = frame
            else:
                time.sleep(0.033)

        cap.release()

    def get_jpeg_b64(self, quality: int = 80) -> str:
        """Retorna la ultima imagen como JPEG base64, o cadena vacia si no hay."""
        with self._lock:
            frame = self._frame
        if frame is None:
            return ""
        ret, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ret:
            return ""
        return base64.b64encode(buf).decode("utf-8")

    def stop(self):
        self._running = False


# ---------------------------------------------------------------------------
# Pico (base movil)
# ---------------------------------------------------------------------------
class PicoInterface:
    """Comunicacion serial con el Pico para control de base y lectura de encoders."""

    # Parametros fisicos del robot (ajustar segun hardware)
    WHEEL_RADIUS_M = 0.065  # radio de rueda en metros
    WHEEL_BASE_M = 0.30  # distancia entre ruedas izquierda-derecha
    TICKS_PER_REV = (
        13440  # 64 CPR * 210 (reduccion) * 4 (X4) = pulsos/vuelta en eje salida
    )

    def __init__(self, cfg: PicoConfig):
        self.cfg = cfg
        self._serial: serial.Serial | None = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._last_enc = [0, 0, 0, 0]
        self._last_time = time.time()
        # Odometria acumulada
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.mode = "RC"

    def start(self):
        try:
            self._serial = serial.Serial(
                self.cfg.port,
                self.cfg.baudrate,
                timeout=self.cfg.timeout,
            )
            logger.info(f"Pico conectado en {self.cfg.port}")
        except Exception as e:
            logger.warning(
                f"No se pudo conectar al Pico ({self.cfg.port}): {e}. Base movil deshabilitada."
            )
            self._serial = None
            return

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        """Lee lineas JSON del Pico y actualiza odometria."""
        buf = ""
        while self._running:
            try:
                if self._serial and self._serial.in_waiting:
                    ch = self._serial.read(1).decode("utf-8", errors="ignore")
                    if ch == "\n":
                        line = buf.strip()
                        buf = ""
                        if line:
                            self._process_telemetry(line)
                    else:
                        buf += ch
                else:
                    time.sleep(0.005)
            except Exception as e:
                logger.debug(f"Error leyendo Pico: {e}")
                time.sleep(0.1)

    def _process_telemetry(self, line: str):
        try:
            data = json.loads(line)
        except Exception:
            return

        enc = data.get("enc", [0, 0, 0, 0])
        self.mode = data.get("mode", "RC")

        now = time.time()
        dt = now - self._last_time
        self._last_time = now

        # Diferencia de ticks
        d_enc = [enc[i] - self._last_enc[i] for i in range(4)]
        self._last_enc = list(enc)

        # Ticks a distancia: promedio izq (0,2) y der (1,3)
        left_ticks = (d_enc[0] + d_enc[2]) / 2.0
        right_ticks = (d_enc[1] + d_enc[3]) / 2.0

        dist_per_tick = (2 * math.pi * self.WHEEL_RADIUS_M) / self.TICKS_PER_REV
        d_left = left_ticks * dist_per_tick
        d_right = right_ticks * dist_per_tick

        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.WHEEL_BASE_M

        # Actualizar pose
        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

    def send_drive(self, vx: float, omega: float):
        if self._serial is None:
            return
        cmd = json.dumps({"cmd": "drive", "vx": vx, "omega": omega}) + "\n"
        with self._lock:
            try:
                self._serial.write(cmd.encode())
            except Exception as e:
                logger.debug(f"Error enviando drive al Pico: {e}")

    def send_stop(self):
        if self._serial is None:
            return
        cmd = json.dumps({"cmd": "stop"}) + "\n"
        with self._lock:
            try:
                self._serial.write(cmd.encode())
            except Exception as e:
                logger.debug(f"Error enviando stop al Pico: {e}")

    def get_odometry(self) -> dict:
        return {
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "theta": round(self.theta, 4),
        }

    def stop(self):
        self._running = False
        self.send_stop()


# ---------------------------------------------------------------------------
# Servidor principal
# ---------------------------------------------------------------------------
@draccus.wrap()
def main(cfg: ControlServerConfig):
    # --- Brazo follower ---
    logger.info("Conectando brazo SO101 follower...")
    robot = SOFollower(cfg.robot)
    robot.connect()
    logger.info("Brazo conectado.")

    # --- Base movil (Pico) ---
    pico = PicoInterface(cfg.pico)
    pico.start()

    # --- Camaras ---
    cam_csi = CameraStream(
        cfg.camera.csi_index,
        cfg.camera.width,
        cfg.camera.height,
        cfg.camera.fps,
        name="CSI",
    )
    cam_usb = CameraStream(
        cfg.camera.usb_index,
        cfg.camera.width,
        cfg.camera.height,
        cfg.camera.fps,
        name="USB",
    )
    cam_csi.start()
    cam_usb.start()

    # --- ZMQ ---
    context = zmq.Context()

    cmd_socket = context.socket(zmq.PULL)
    cmd_socket.setsockopt(zmq.CONFLATE, 1)
    cmd_socket.setsockopt(zmq.RCVTIMEO, 0)
    cmd_socket.bind(f"tcp://*:{cfg.server.port_zmq_cmd}")

    obs_socket = context.socket(zmq.PUSH)
    obs_socket.setsockopt(zmq.CONFLATE, 1)
    obs_socket.bind(f"tcp://*:{cfg.server.port_zmq_obs}")

    logger.info(
        f"Servidor listo. ZMQ cmd={cfg.server.port_zmq_cmd}, obs={cfg.server.port_zmq_obs}"
    )

    dt = 1.0 / cfg.server.max_loop_hz

    try:
        start = time.perf_counter()
        while time.perf_counter() - start < cfg.server.connection_time_s:
            loop_start = time.perf_counter()

            # --- Recibir comando del GCS ---
            try:
                msg = cmd_socket.recv_string(zmq.NOBLOCK)
                cmd = json.loads(msg)

                # Comando de brazo
                arm_cmd = cmd.get("arm")
                if arm_cmd:
                    try:
                        robot.send_action(arm_cmd)
                    except Exception as e:
                        logger.error(f"Error enviando accion al brazo: {e}")

                # Comando de base
                base_cmd = cmd.get("base")
                if base_cmd:
                    vx = float(base_cmd.get("vx", 0.0))
                    omega = float(base_cmd.get("omega", 0.0))
                    pico.send_drive(vx, omega)

            except zmq.Again:
                pass
            except Exception as e:
                logger.error(f"Error procesando comando: {e}")

            # --- Leer observacion ---
            try:
                arm_obs = robot.get_observation()
                arm_serializable = {
                    k: float(v) for k, v in arm_obs.items() if not hasattr(v, "shape")
                }
            except Exception as e:
                import traceback
                logger.error(f"Error leyendo brazo: {e}\n{traceback.format_exc()}")
                arm_serializable = {}

            # --- Construir y enviar observacion ---
            obs = {
                "arm": arm_serializable,
                "odom": pico.get_odometry(),
                "mode": pico.mode,
                "camera_csi": cam_csi.get_jpeg_b64(cfg.camera.jpeg_quality),
                "camera_usb": cam_usb.get_jpeg_b64(cfg.camera.jpeg_quality),
            }

            try:
                obs_socket.send_string(json.dumps(obs), zmq.NOBLOCK)
            except zmq.Again:
                pass

            # --- Frecuencia ---
            elapsed = time.perf_counter() - loop_start
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        logger.info("Interrupcion. Apagando...")
    finally:
        pico.stop()
        cam_csi.stop()
        cam_usb.stop()
        robot.disconnect()
        cmd_socket.close()
        obs_socket.close()
        context.term()
        logger.info("Servidor apagado.")


if __name__ == "__main__":
    main()
