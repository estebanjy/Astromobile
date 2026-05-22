#!/usr/bin/env python
"""
Ground Control Station (GCS) - corre en el Mac.
Interfaz PyQt6 para teleoperacion del robot movil con brazo SO101.

Funciones:
  - 2 feeds de video FPV (CSI y USB), tolerante a camara ausente
  - Panel de joints del brazo SO101 (observacion en tiempo real)
  - Display de odometria (x, y, theta)
  - Indicador de modo RC / GCS
  - Control del brazo leader SO101 (hilo separado)
  - Control de base movil via teclado (WASD) o sliders

Uso:
    conda activate lerobot
    PYTHONPATH=src python gcs_app.py \
        --remote_ip=192.168.1.17 \
        --leader_port=/dev/tty.usbmodem5A680129461

Dependencias Mac:
    pip install pyqt6 pyzmq

Controles de teclado (base movil, solo cuando GCS tiene foco):
    W / S : adelante / atras
    A / D : giro izquierda / derecha
    Espacio: parar
"""

import argparse
import base64
import json
import logging
import math
import threading
import time

import numpy as np
import zmq
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt6.QtGui import QImage, QPixmap, QFont, QKeyEvent, QColor
from PyQt6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QProgressBar,
    QPushButton,
    QSizePolicy,
    QSlider,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


# ---------------------------------------------------------------------------
# Hilo ZMQ: recibe observaciones y envia comandos
# ---------------------------------------------------------------------------
class ZMQWorker(QObject):
    observation_received = pyqtSignal(dict)
    connection_lost = pyqtSignal()

    def __init__(self, remote_ip: str, port_cmd: int = 5555, port_obs: int = 5556):
        super().__init__()
        self.remote_ip = remote_ip
        self.port_cmd = port_cmd
        self.port_obs = port_obs
        self._running = False
        self._lock = threading.Lock()
        self._pending_cmd: dict | None = None
        self._thread = None
        self._cmd_socket = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        context = zmq.Context()

        cmd_socket = context.socket(zmq.PUSH)
        cmd_socket.setsockopt(zmq.CONFLATE, 1)
        cmd_socket.connect(f"tcp://{self.remote_ip}:{self.port_cmd}")
        self._cmd_socket = cmd_socket

        obs_socket = context.socket(zmq.PULL)
        obs_socket.setsockopt(zmq.CONFLATE, 1)
        obs_socket.setsockopt(zmq.RCVTIMEO, 200)
        obs_socket.connect(f"tcp://{self.remote_ip}:{self.port_obs}")

        last_obs_time = time.time()

        while self._running:
            # Enviar comando pendiente
            with self._lock:
                cmd = self._pending_cmd
                self._pending_cmd = None

            if cmd is not None:
                try:
                    cmd_socket.send_string(json.dumps(cmd), zmq.NOBLOCK)
                except zmq.Again:
                    pass

            # Recibir observacion
            try:
                msg = obs_socket.recv_string()
                obs = json.loads(msg)
                last_obs_time = time.time()
                self.observation_received.emit(obs)
            except zmq.Again:
                if time.time() - last_obs_time > 3.0:
                    self.connection_lost.emit()
            except Exception as e:
                logger.debug(f"Error ZMQ obs: {e}")

        cmd_socket.close()
        obs_socket.close()
        context.term()

    def send_command(self, cmd: dict):
        with self._lock:
            self._pending_cmd = cmd

    def stop(self):
        self._running = False


# ---------------------------------------------------------------------------
# Hilo leader SO101 (brazo)
# ---------------------------------------------------------------------------
class LeaderWorker(QObject):
    action_ready = pyqtSignal(dict)

    def __init__(self, leader_port: str, leader_id: str, fps: int = 30):
        super().__init__()
        self.leader_port = leader_port
        self.leader_id = leader_id
        self.fps = fps
        self._running = False
        self._enabled = True
        self._thread = None
        self._leader = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        try:
            from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderTeleopConfig
            from lerobot.teleoperators.so_leader.so_leader import SOLeader
        except ImportError:
            logger.error("No se pudo importar SOLeader. Teleoperacion del brazo deshabilitada.")
            return

        try:
            config = SOLeaderTeleopConfig(port=self.leader_port, id=self.leader_id)
            self._leader = SOLeader(config)
            self._leader.connect()
            logger.info("Leader arm conectado.")
        except Exception as e:
            logger.error(f"Error conectando leader: {e}")
            return

        dt = 1.0 / self.fps
        while self._running:
            t0 = time.perf_counter()
            if self._enabled:
                try:
                    action = self._leader.get_action()
                    self.action_ready.emit(action)
                except Exception as e:
                    logger.debug(f"Error leyendo leader: {e}")
            elapsed = time.perf_counter() - t0
            time.sleep(max(0, dt - elapsed))

    def set_enabled(self, enabled: bool):
        self._enabled = enabled

    def stop(self):
        self._running = False
        if self._leader:
            try:
                self._leader.disconnect()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Widget de video
# ---------------------------------------------------------------------------
class VideoWidget(QLabel):
    """Muestra un frame de video escalado al widget."""

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setStyleSheet("background-color: #1a1a2e; border: 1px solid #444; border-radius: 4px;")
        self._title = title
        self._show_no_signal()

    def _show_no_signal(self):
        self.setText(f"[{self._title}]\nSin señal")
        self.setStyleSheet(
            "background-color: #1a1a2e; color: #888; border: 1px solid #444; "
            "border-radius: 4px; font-size: 14px;"
        )

    def update_frame(self, b64_jpeg: str):
        if not b64_jpeg:
            self._show_no_signal()
            return

        try:
            data = base64.b64decode(b64_jpeg)
            arr = np.frombuffer(data, dtype=np.uint8)
            import cv2
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                self._show_no_signal()
                return
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame_rgb.shape
            qimg = QImage(frame_rgb.data, w, h, ch * w, QImage.Format.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg).scaled(
                self.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
            self.setPixmap(pixmap)
            self.setStyleSheet("background-color: #000; border: 1px solid #444; border-radius: 4px;")
        except Exception as e:
            logger.debug(f"Error decodificando frame: {e}")
            self._show_no_signal()


# ---------------------------------------------------------------------------
# Panel de joints
# ---------------------------------------------------------------------------
class JointPanel(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("Brazo SO101 — Joints", parent)
        self.setStyleSheet(PANEL_STYLE)
        layout = QGridLayout(self)
        layout.setSpacing(4)

        self._bars: dict[str, QProgressBar] = {}
        self._labels: dict[str, QLabel] = {}

        for row, name in enumerate(JOINT_NAMES):
            lbl = QLabel(name.replace("_", " ").title())
            lbl.setMinimumWidth(110)
            lbl.setStyleSheet("color: #ccc; font-size: 12px;")

            bar = QProgressBar()
            bar.setRange(-314, 314)   # -pi a pi en centi-rad
            bar.setValue(0)
            bar.setTextVisible(False)
            bar.setStyleSheet("""
                QProgressBar { background: #2d2d2d; border-radius: 3px; height: 14px; }
                QProgressBar::chunk { background: #4fc3f7; border-radius: 3px; }
            """)

            val_lbl = QLabel("0.00")
            val_lbl.setFixedWidth(55)
            val_lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            val_lbl.setStyleSheet("color: #4fc3f7; font-family: monospace; font-size: 12px;")

            layout.addWidget(lbl, row, 0)
            layout.addWidget(bar, row, 1)
            layout.addWidget(val_lbl, row, 2)

            self._bars[name] = bar
            self._labels[name] = val_lbl

    def update_joints(self, arm: dict):
        for name in JOINT_NAMES:
            val = arm.get(name, 0.0)
            centi = int(val * 100)
            bar = self._bars.get(name)
            lbl = self._labels.get(name)
            if bar:
                bar.setValue(max(-314, min(314, centi)))
            if lbl:
                lbl.setText(f"{val:.2f}")


# ---------------------------------------------------------------------------
# Panel de odometria
# ---------------------------------------------------------------------------
class OdometryPanel(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("Odometria", parent)
        self.setStyleSheet(PANEL_STYLE)
        layout = QGridLayout(self)

        self._fields: dict[str, QLabel] = {}
        for row, (key, label) in enumerate([
            ("x", "X (m)"),
            ("y", "Y (m)"),
            ("theta", "Theta (rad)"),
        ]):
            lbl = QLabel(label)
            lbl.setStyleSheet("color: #ccc; font-size: 12px;")
            val = QLabel("0.0000")
            val.setStyleSheet("color: #a5d6a7; font-family: monospace; font-size: 13px; font-weight: bold;")
            val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            layout.addWidget(lbl, row, 0)
            layout.addWidget(val, row, 1)
            self._fields[key] = val

    def update_odom(self, odom: dict):
        for key, lbl in self._fields.items():
            val = odom.get(key, 0.0)
            lbl.setText(f"{val:.4f}")


# ---------------------------------------------------------------------------
# Panel de modo (RC / GCS)
# ---------------------------------------------------------------------------
class ModeIndicator(QLabel):
    def __init__(self, parent=None):
        super().__init__("  GCS  ", parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setFixedHeight(36)
        self.setFont(QFont("monospace", 14, QFont.Weight.Bold))
        self._set_gcs()

    def _set_rc(self):
        self.setText("  RC — Control por radio  ")
        self.setStyleSheet(
            "background: #e65100; color: white; border-radius: 6px; padding: 4px 12px;"
        )

    def _set_gcs(self):
        self.setText("  GCS — Control por software  ")
        self.setStyleSheet(
            "background: #1565c0; color: white; border-radius: 6px; padding: 4px 12px;"
        )

    def update_mode(self, mode: str):
        if mode == "RC":
            self._set_rc()
        else:
            self._set_gcs()


# ---------------------------------------------------------------------------
# Panel de control de base (teclado)
# ---------------------------------------------------------------------------
PANEL_STYLE = """
    QGroupBox {
        color: #e0e0e0;
        font-size: 13px;
        font-weight: bold;
        border: 1px solid #444;
        border-radius: 6px;
        margin-top: 8px;
        padding-top: 8px;
        background: #12121f;
    }
    QGroupBox::title {
        subcontrol-origin: margin;
        left: 10px;
        padding: 0 4px;
    }
"""


class BaseControlPanel(QGroupBox):
    drive_changed = pyqtSignal(float, float)  # vx, omega

    def __init__(self, parent=None):
        super().__init__("Base movil — Teclado (W/S/A/D) o sliders", parent)
        self.setStyleSheet(PANEL_STYLE)

        self._vx = 0.0
        self._omega = 0.0

        layout = QVBoxLayout(self)

        # Sliders
        sl_layout = QGridLayout()

        lbl_vx = QLabel("Velocidad lineal")
        lbl_vx.setStyleSheet("color: #ccc;")
        self._slider_vx = QSlider(Qt.Orientation.Horizontal)
        self._slider_vx.setRange(-100, 100)
        self._slider_vx.setValue(0)
        self._slider_vx.valueChanged.connect(self._on_vx_changed)
        self._lbl_vx_val = QLabel("0.00")
        self._lbl_vx_val.setStyleSheet("color: #4fc3f7; font-family: monospace;")

        lbl_omega = QLabel("Velocidad angular")
        lbl_omega.setStyleSheet("color: #ccc;")
        self._slider_omega = QSlider(Qt.Orientation.Horizontal)
        self._slider_omega.setRange(-100, 100)
        self._slider_omega.setValue(0)
        self._slider_omega.valueChanged.connect(self._on_omega_changed)
        self._lbl_omega_val = QLabel("0.00")
        self._lbl_omega_val.setStyleSheet("color: #4fc3f7; font-family: monospace;")

        sl_layout.addWidget(lbl_vx, 0, 0)
        sl_layout.addWidget(self._slider_vx, 0, 1)
        sl_layout.addWidget(self._lbl_vx_val, 0, 2)
        sl_layout.addWidget(lbl_omega, 1, 0)
        sl_layout.addWidget(self._slider_omega, 1, 1)
        sl_layout.addWidget(self._lbl_omega_val, 1, 2)

        btn_stop = QPushButton("PARAR (Espacio)")
        btn_stop.setStyleSheet(
            "background: #c62828; color: white; font-weight: bold; "
            "border-radius: 4px; padding: 6px;"
        )
        btn_stop.clicked.connect(self.stop)

        layout.addLayout(sl_layout)
        layout.addWidget(btn_stop)

    def _on_vx_changed(self, val: int):
        self._vx = val / 100.0
        self._lbl_vx_val.setText(f"{self._vx:.2f}")
        self.drive_changed.emit(self._vx, self._omega)

    def _on_omega_changed(self, val: int):
        self._omega = val / 100.0
        self._lbl_omega_val.setText(f"{self._omega:.2f}")
        self.drive_changed.emit(self._vx, self._omega)

    def stop(self):
        self._slider_vx.setValue(0)
        self._slider_omega.setValue(0)

    def apply_key(self, key: Qt.Key, pressed: bool):
        speed = 60  # 60% velocidad con teclado
        if not pressed:
            self.stop()
            return
        if key == Qt.Key.Key_W:
            self._slider_vx.setValue(speed)
        elif key == Qt.Key.Key_S:
            self._slider_vx.setValue(-speed)
        elif key == Qt.Key.Key_A:
            self._slider_omega.setValue(-speed)
        elif key == Qt.Key.Key_D:
            self._slider_omega.setValue(speed)
        elif key == Qt.Key.Key_Space:
            self.stop()


# ---------------------------------------------------------------------------
# Ventana principal
# ---------------------------------------------------------------------------
class GCSWindow(QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.setWindowTitle("GCS — Robot Movil SO101")
        self.setMinimumSize(1100, 700)
        self.setStyleSheet("background-color: #0d0d1a; color: #e0e0e0;")

        self._base_vx = 0.0
        self._base_omega = 0.0
        self._last_obs_time = 0.0  # fuerza banner "sin conexion" al arrancar

        self._build_ui()
        self._setup_zmq()
        self._setup_leader()
        self._setup_timer()

    # --- UI ---
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(6)
        root.setContentsMargins(8, 8, 8, 8)

        # Banner de conexion — ocupa todo el ancho, visible cuando no hay datos
        self._conn_banner = QLabel()
        self._conn_banner.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._conn_banner.setFixedHeight(36)
        self._conn_banner.setFont(QFont("monospace", 12, QFont.Weight.Bold))
        self._update_conn_banner(connected=False)

        # Columna izquierda: videos
        left = QVBoxLayout()
        left.setSpacing(6)

        lbl_cameras = QLabel("FPV")
        lbl_cameras.setStyleSheet("color: #888; font-size: 11px; letter-spacing: 2px;")

        self._cam_csi = VideoWidget("CSI")
        self._cam_usb = VideoWidget("USB")

        left.addWidget(lbl_cameras)
        left.addWidget(self._cam_csi, stretch=1)
        left.addWidget(self._cam_usb, stretch=1)

        # Columna derecha: telemetria + control
        right = QVBoxLayout()
        right.setSpacing(8)
        right.setContentsMargins(0, 0, 0, 0)

        self._mode_indicator = ModeIndicator()
        right.addWidget(self._mode_indicator)

        self._joint_panel = JointPanel()
        right.addWidget(self._joint_panel)

        self._odom_panel = OdometryPanel()
        right.addWidget(self._odom_panel)

        self._base_panel = BaseControlPanel()
        self._base_panel.drive_changed.connect(self._on_base_drive)
        right.addWidget(self._base_panel)

        # Toggle brazo
        self._btn_arm = QPushButton("Brazo: ACTIVO")
        self._btn_arm.setCheckable(True)
        self._btn_arm.setChecked(True)
        self._btn_arm.setStyleSheet(
            "QPushButton { background: #1b5e20; color: white; border-radius: 4px; padding: 6px; font-weight: bold; }"
            "QPushButton:checked { background: #1b5e20; }"
            "QPushButton:!checked { background: #4a4a4a; }"
        )
        self._btn_arm.toggled.connect(self._on_arm_toggled)
        right.addWidget(self._btn_arm)

        right.addStretch()

        # Layout de columnas
        columns = QHBoxLayout()
        columns.setSpacing(8)

        columns.addLayout(left, stretch=3)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.VLine)
        sep.setStyleSheet("color: #333;")
        columns.addWidget(sep)

        columns.addLayout(right, stretch=2)

        # Layout raiz: banner arriba + columnas abajo
        root.addWidget(self._conn_banner)
        root.addLayout(columns, stretch=1)

        # Status bar
        self._status = QStatusBar()
        self._status.setStyleSheet("color: #888; font-size: 11px;")
        self.setStatusBar(self._status)
        self._status.showMessage("Conectando...")

    # --- ZMQ ---
    def _setup_zmq(self):
        self._zmq = ZMQWorker(
            self.args.remote_ip,
            self.args.port_cmd,
            self.args.port_obs,
        )
        self._zmq.observation_received.connect(self._on_observation)
        self._zmq.connection_lost.connect(self._on_connection_lost)
        self._zmq.start()

    # --- Leader arm ---
    def _setup_leader(self):
        self._leader = None
        if self.args.leader_port:
            self._leader = LeaderWorker(
                self.args.leader_port,
                self.args.leader_id,
                fps=self.args.fps,
            )
            self._leader.action_ready.connect(self._on_arm_action)
            self._leader.start()

    # --- Timer de actualizacion de UI ---
    def _setup_timer(self):
        self._timer = QTimer()
        self._timer.setInterval(33)  # ~30fps UI
        self._timer.timeout.connect(self._tick)
        self._timer.start()

    def _update_conn_banner(self, connected: bool):
        if connected:
            self._conn_banner.setText(f"  Conectado — {self.args.remote_ip}  ")
            self._conn_banner.setStyleSheet(
                "background: #1b5e20; color: white; border-radius: 4px; padding: 4px 12px;"
            )
        else:
            ip = getattr(self.args, "remote_ip", "???")
            self._conn_banner.setText(f"  SIN CONEXION — Jetson no responde ({ip})  ")
            self._conn_banner.setStyleSheet(
                "background: #b71c1c; color: white; border-radius: 4px; padding: 4px 12px;"
            )

    def _tick(self):
        age = time.time() - self._last_obs_time
        connected = age < 3.0
        self._update_conn_banner(connected)
        if connected:
            self._status.showMessage(f"Conectado | {self.args.remote_ip}")
        else:
            self._status.showMessage(f"Sin datos ({age:.1f}s) | {self.args.remote_ip}")

    # --- Slots ---
    def _on_observation(self, obs: dict):
        self._last_obs_time = time.time()

        arm = obs.get("arm", {})
        if arm:
            self._joint_panel.update_joints(arm)

        odom = obs.get("odom", {})
        if odom:
            self._odom_panel.update_odom(odom)

        mode = obs.get("mode", "GCS")
        self._mode_indicator.update_mode(mode)

        self._cam_csi.update_frame(obs.get("camera_csi", ""))
        self._cam_usb.update_frame(obs.get("camera_usb", ""))

    def _on_connection_lost(self):
        self._status.showMessage("CONEXION PERDIDA — reintentando...")

    def _on_arm_action(self, action: dict):
        """Envia accion del brazo leader al servidor via ZMQ."""
        cmd = {"arm": action, "base": {"vx": self._base_vx, "omega": self._base_omega}}
        self._zmq.send_command(cmd)

    def _on_base_drive(self, vx: float, omega: float):
        self._base_vx = vx
        self._base_omega = omega
        # Si no hay leader activo, enviar solo comando de base
        if not self._leader or not self._leader._enabled:
            self._zmq.send_command({"base": {"vx": vx, "omega": omega}})

    def _on_arm_toggled(self, checked: bool):
        self._btn_arm.setText("Brazo: ACTIVO" if checked else "Brazo: PAUSADO")
        if self._leader:
            self._leader.set_enabled(checked)

    # --- Teclado ---
    def keyPressEvent(self, event: QKeyEvent):
        key = event.key()
        if key in (Qt.Key.Key_W, Qt.Key.Key_S, Qt.Key.Key_A, Qt.Key.Key_D, Qt.Key.Key_Space):
            self._base_panel.apply_key(key, pressed=True)
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QKeyEvent):
        key = event.key()
        if key in (Qt.Key.Key_W, Qt.Key.Key_S, Qt.Key.Key_A, Qt.Key.Key_D):
            self._base_panel.apply_key(key, pressed=False)
        else:
            super().keyReleaseEvent(event)

    # --- Cierre ---
    def closeEvent(self, event):
        self._timer.stop()
        if self._leader:
            self._leader.stop()
        self._zmq.stop()
        event.accept()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(description="GCS — Estacion de control terrestre")
    parser.add_argument("--remote_ip", default="192.168.1.17", help="IP de la Jetson")
    parser.add_argument("--port_cmd", type=int, default=5555)
    parser.add_argument("--port_obs", type=int, default=5556)
    parser.add_argument("--leader_port", default=None,
                        help="Puerto USB del brazo leader (omitir para solo video)")
    parser.add_argument("--leader_id", default="my_awesome_leader_arm")
    parser.add_argument("--fps", type=int, default=30)
    return parser.parse_args()


def main():
    import sys
    args = parse_args()
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    win = GCSWindow(args)
    win.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
