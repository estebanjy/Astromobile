"""
autonomy_bridge.py — corre en la Jetson Orin Nano (robot real, no simulacion)

Conecta ROS 2 con control_server.py via ZMQ:
  /cmd_vel  (nav2)  →  ZMQ PUSH  →  control_server → Pico → motores
  ZMQ PULL  (obs)   →  /odom/filtered, /joint_states

Uso:
    ros2 run robot_sim autonomy_bridge --ros-args \
        -p remote_ip:=localhost \
        -p port_cmd:=5555 \
        -p port_obs:=5556
"""

import json
import threading

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import zmq


class AutonomyBridge(Node):

    def __init__(self):
        super().__init__("autonomy_bridge")

        # Parametros
        self.declare_parameter("remote_ip", "localhost")
        self.declare_parameter("port_cmd",  5555)
        self.declare_parameter("port_obs",  5556)

        ip       = self.get_parameter("remote_ip").value
        port_cmd = self.get_parameter("port_cmd").value
        port_obs = self.get_parameter("port_obs").value

        # ZMQ
        self._ctx = zmq.Context()

        self._cmd_sock = self._ctx.socket(zmq.PUSH)
        self._cmd_sock.setsockopt(zmq.CONFLATE, 1)
        self._cmd_sock.connect(f"tcp://{ip}:{port_cmd}")

        self._obs_sock = self._ctx.socket(zmq.PULL)
        self._obs_sock.setsockopt(zmq.CONFLATE, 1)
        self._obs_sock.setsockopt(zmq.RCVTIMEO, 200)
        self._obs_sock.connect(f"tcp://{ip}:{port_obs}")

        # Publishers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._odom_pub = self.create_publisher(Odometry, "/odom", qos)

        # Subscriber /cmd_vel de Nav2
        self._cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, 10
        )

        # Hilo que lee observaciones de ZMQ y las publica en ROS 2
        self._running = True
        self._obs_thread = threading.Thread(target=self._obs_loop, daemon=True)
        self._obs_thread.start()

        self.get_logger().info(
            f"autonomy_bridge conectado a tcp://{ip}  cmd:{port_cmd}  obs:{port_obs}"
        )

    def _on_cmd_vel(self, msg: Twist):
        """Recibe /cmd_vel de Nav2 y lo envía al control_server via ZMQ."""
        # Normalizar a [-1, 1] (el control_server espera vx/omega en esa escala)
        max_linear  = 0.5   # m/s
        max_angular = 1.0   # rad/s
        vx    = max(-1.0, min(1.0, msg.linear.x  / max_linear))
        omega = max(-1.0, min(1.0, msg.angular.z / max_angular))

        cmd = {"base": {"vx": vx, "omega": omega}}
        try:
            self._cmd_sock.send_string(json.dumps(cmd), zmq.NOBLOCK)
        except zmq.Again:
            pass

    def _obs_loop(self):
        """Lee observaciones del control_server y las publica como topics ROS 2."""
        while self._running:
            try:
                msg = self._obs_sock.recv_string()
                obs = json.loads(msg)
            except zmq.Again:
                continue
            except Exception as e:
                self.get_logger().debug(f"Error leyendo obs ZMQ: {e}")
                continue

            # Publicar odometria
            odom_data = obs.get("odom", {})
            if odom_data:
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id  = "base_footprint"
                odom_msg.pose.pose.position.x = odom_data.get("x", 0.0)
                odom_msg.pose.pose.position.y = odom_data.get("y", 0.0)

                import math
                theta = odom_data.get("theta", 0.0)
                odom_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
                odom_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
                self._odom_pub.publish(odom_msg)

    def destroy_node(self):
        self._running = False
        self._cmd_sock.close()
        self._obs_sock.close()
        self._ctx.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
