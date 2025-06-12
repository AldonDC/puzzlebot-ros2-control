"""
PID Controller Node for Point-to-Point Navigation (with proper state transitions)

Subscribes:
- /odom (nav_msgs/Odometry)
- /setpoint (geometry_msgs/Vector3)

Publishes:
- /cmd_vel (geometry_msgs/Twist)
"""

import math
import rclpy
from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry


class PIDController(Node):
    def __init__(self):
        super().__init__("position_controller")

        # -------------------------
        # ROS 2 Communication Setup
        # -------------------------
        self.create_subscription(Odometry, 'odom', self.odometry_callback, qos.qos_profile_sensor_data)
        self.create_subscription(Vector3, 'setpoint', self.setpoint_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('MAX_V', 0.15)
        self.declare_parameter('MAX_W', 1.0)

        self.declare_parameter('KpPos', 1.0)
        self.declare_parameter('KiPos', 0.2)
        self.declare_parameter('KdPos', 0.01)

        self.declare_parameter('KpHead', 1.0)
        self.declare_parameter('KiHead', 0.0)
        self.declare_parameter('KdHead', 0.0)
        
        self.declare_parameter('ANGLE_ALIGNMENT_THRESHOLD', 0.2)

        # Load values
        self.MAX_V = self.get_parameter('MAX_V').value
        self.MAX_W = self.get_parameter('MAX_W').value
        self.kp_pos = self.get_parameter('KpPos').value
        self.kd_pos = self.get_parameter('KdPos').value
        self.ki_pos = self.get_parameter('KiPos').value
        self.kp_head = self.get_parameter('KpHead').value
        self.ki_head = self.get_parameter('KiHead').value
        self.kd_head = self.get_parameter('KdHead').value
        self.angle_threshold = self.get_parameter('ANGLE_ALIGNMENT_THRESHOLD').value

        # -------------------------
        # State and Control Variables
        # -------------------------
        self.prev_error_pos = 0.0
        self.integral_pos = 0.0
        self.prev_error_ang = 0.0
        self.integral_ang = 0.0
        self.prev_sample_time = self.get_clock().now().nanoseconds * 1e-9

        self.setpoint_x = 0.0
        self.setpoint_y = 0.0

        # Finite State Machine: ALIGNING → MOVING → STOPPED
        self.state = "ALIGNING"

        self.get_logger().info("✅ PID Controller Node started\n" + "=" * 60)

    # -------------------------
    # Odometry Feedback Handler
    # -------------------------
    def odometry_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.normalize_angle(self.quaternion_to_yaw(msg.pose.pose.orientation))

        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = max(curr_time - self.prev_sample_time, 1e-6)
        self.prev_sample_time = curr_time

        # Compute errors
        error_x = self.setpoint_x - x
        error_y = self.setpoint_y - y
        error_pos = math.hypot(error_x, error_y)
        target_ang = math.atan2(error_y, error_x)
        error_ang = self.normalize_angle(target_ang - yaw)

        error_d_pos = (error_pos - self.prev_error_pos) / dt
        error_d_ang = (error_ang - self.prev_error_ang) / dt
        self.integral_pos = max(min(self.integral_pos + error_pos * dt, 2.0), -2.0)
        self.integral_ang = max(min(self.integral_ang + error_ang * dt, 2.0), -2.0)

        POS_TOL = 0.03
        ANG_TOL = 0.05
        cmd = Twist()

        # -------------------------
        # State Machine Transitions
        # -------------------------
        if self.state == "ALIGNING":
            cmd.linear.x = 0.0
            cmd.angular.z = self.kp_head * error_ang + self.kd_head * error_d_ang + self.ki_head * self.integral_ang
            if abs(error_ang) < ANG_TOL:
                self.state = "MOVING"
                self.integral_ang = 0.0

        elif self.state == "MOVING":
            cmd.linear.x = self.kp_pos * error_pos + self.kd_pos * error_d_pos + self.ki_pos * self.integral_pos
            cmd.angular.z = self.kp_head * error_ang + self.kd_head * error_d_ang + self.ki_head * self.integral_ang
            if abs(error_ang) > self.angle_threshold:
                cmd.linear.x *= 0.4
            if error_pos < POS_TOL and abs(error_ang) < ANG_TOL:
                self.state = "STOPPED"
                self.integral_pos = 0.0
                self.integral_ang = 0.0
                self.get_logger().info(f"🎯 Objetivo alcanzado: ({x:.2f}, {y:.2f})\n" + "-" * 60)

        elif self.state == "STOPPED":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Publish command
        cmd.linear.x = max(min(cmd.linear.x, self.MAX_V), -self.MAX_V)
        cmd.angular.z = max(min(cmd.angular.z, self.MAX_W), -self.MAX_W)
        self.cmd_vel_publisher.publish(cmd)

        # Log current state
        self.get_logger().info(
            f"📍 Pos: ({x:.2f}, {y:.2f}) → Setpoint: ({self.setpoint_x:.2f}, {self.setpoint_y:.2f})\n"
            f"🛠️ e_pos={error_pos:.2f} | v={cmd.linear.x:.2f} m/s\n"
            f"🔄 e_ang={error_ang:.2f} | w={cmd.angular.z:.2f} rad/s\n"
            + "-" * 60
        )

        self.prev_error_pos = error_pos
        self.prev_error_ang = error_ang

    # -------------------------
    # Setpoint Handler
    # -------------------------
    def setpoint_callback(self, msg):
        self.setpoint_x = msg.x
        self.setpoint_y = msg.y
        self.state = "ALIGNING"
        self.get_logger().info(f"📌 Nuevo setpoint recibido: ({msg.x:.2f}, {msg.y:.2f})")

    # -------------------------
    # Utilities
    # -------------------------
    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        return math.atan2(siny_cosp, cosy_cosp)


# -------------------------
# Node Entry Point
# -------------------------
def main(args=None):
    rclpy.init(args=args)
    controller = PIDController()
    try:
        rclpy.spin(controller)
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
