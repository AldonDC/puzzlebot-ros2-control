"""
odometry_node.py
========================================

📍 Robot Localization Node (Terminal Estética + Fluidez PID)

Subscribes:
- /VelocityEncL
- /VelocityEncR

Publishes:
- /odom (nav_msgs/Odometry)
"""

import rclpy
import math
import signal
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry


class RobotLocalization(Node):
    def __init__(self):
        super().__init__('localization_node')

        # Publisher de odometría
        self.odometry_publisher = self.create_publisher(Odometry, 'odom', qos.qos_profile_sensor_data)

        # Subscripciones a velocidades de encoder
        self.left_velocity_subscriber = self.create_subscription(Float32, 'VelocityEncL', self.update_wl, qos.qos_profile_sensor_data)
        self.right_velocity_subscriber = self.create_subscription(Float32, 'VelocityEncR', self.update_wr, qos.qos_profile_sensor_data)

        # Inicialización de velocidades y pose
        self.wl = 0.0
        self.wr = 0.0
        self.X = 0.0
        self.Y = 0.0
        self.Theta = 0.0

        # Parámetros del robot
        self.declare_parameter('wheel_distance', 0.19)
        self.declare_parameter('wheel_radius', 0.05)
        self.wheel_distance = self.get_parameter('wheel_distance').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # Tiempos
        self.declare_parameter('sample_time', 0.01)
        self.declare_parameter('rate', 200.0)
        self.sample_time = self.get_parameter('sample_time').value
        self.rate = self.get_parameter('rate').value

        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.curr_time = self.start_time
        self.last_time = self.start_time

        # Timer de odometría
        self.timer = self.create_timer(1.0 / self.rate, self.calc_odometry)

        self.get_logger().info("🟢 Localization Node iniciado correctamente.\n" + "═" * 60)

    def update_wl(self, msg):
        self.wl = msg.data

    def update_wr(self, msg):
        self.wr = msg.data

    def calc_odometry(self):
        self.curr_time = self.get_clock().now().nanoseconds * 1e-9
        dt = self.curr_time - self.last_time

        if dt < 0.001 or dt > 0.1:
            self.last_time = self.curr_time
            return
        
        def normalize_angle(angle):
            while angle > math.pi:
                angle -= 2*math.pi
            while angle < -math.pi:
                angle += 2*math.pi
            return angle
        
        if dt > self.sample_time:
            # Velocidades lineales
            vl = self.wl * self.wheel_radius
            vr = self.wr * self.wheel_radius
            V = (vr + vl) / 2.0
            W = (vr - vl) / self.wheel_distance

            self.X += V * math.cos(self.Theta) * dt
            self.Y += V * math.sin(self.Theta) * dt
            self.Theta += W * dt
            self.Theta = normalize_angle(self.Theta)

            # Crear mensaje Odometry
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"

            odom.pose.pose.position.x = self.X
            odom.pose.pose.position.y = self.Y
            odom.pose.pose.orientation.z = math.sin(self.Theta / 2.0)
            odom.pose.pose.orientation.w = math.cos(self.Theta / 2.0)

            odom.twist.twist.linear.x = V
            odom.twist.twist.angular.z = W

            self.odometry_publisher.publish(odom)

            # Terminal Output bonito
            self.get_logger().info(
                "\n📡 Odom Published\n"
                + "┌" + "─" * 48 + "┐\n"
                + f"│ 🧭 Posición       →  X: {self.X:6.2f}   Y: {self.Y:6.2f}       │\n"
                + f"│ 🔄 Orientación    →  θ: {self.Theta:6.2f} rad               │\n"
                + f"│ 🚗 Velocidad      →  V: {V:6.3f} m/s   W: {W:6.3f} rad/s │\n"
                + "└" + "─" * 48 + "┘"
            )

            self.last_time = self.curr_time

    def stop_handler(self, signum, frame):
        self.get_logger().info("⛔ SIGINT recibido. Apagando nodo de localización...")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RobotLocalization()
    signal.signal(signal.SIGINT, node.stop_handler)

    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info("🔁 SystemExit recibido. Apagado limpio.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
