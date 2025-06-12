"""
🟦 Path Generator Node with Multiple Trajectories

Publishes:
- /setpoint (geometry_msgs/Vector3)

This node generates different trajectories (square, line, circle, triangle) 
based on a parameter and publishes the waypoints one by one.
"""

import math
import rclpy
from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')

        # Parameters
        self.declare_parameter('trajectory_type', 'square')  # square, line, circle, triangle
        self.declare_parameter('side_length', 1.0)
        self.declare_parameter('points_per_side', 5)
        self.declare_parameter('goal_tolerance', 0.05)

        self.trajectory_type = self.get_parameter('trajectory_type').value
        self.side_length = self.get_parameter('side_length').value
        self.points_per_side = int(self.get_parameter('points_per_side').value)  # 🔵 Corregido aquí
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # Publisher
        self.setpoint_pub = self.create_publisher(Vector3, 'setpoint', 10)

        # Subscriber
        self.create_subscription(Odometry, 'odom', self.odom_callback, qos.qos_profile_sensor_data)

        # Waypoints
        self.waypoints = self.generate_trajectory(self.trajectory_type)
        self.current_index = 0

        if self.waypoints:
            self.goal = self.waypoints[self.current_index]
            self.publish_goal()
            self.get_logger().info(f"🚀 Path Generator Node started with trajectory: {self.trajectory_type}")
        else:
            self.get_logger().warn("⚠️ No se generaron waypoints.")

    def generate_trajectory(self, trajectory_type):
        if trajectory_type == 'square':
            return self.generate_square(self.side_length, self.points_per_side)
        elif trajectory_type == 'line':
            return self.generate_line(self.side_length, self.points_per_side)
        elif trajectory_type == 'circle':
            return self.generate_circle(self.side_length / 2, self.points_per_side * 4)  # 🔵 Ajuste radius
        elif trajectory_type == 'triangle':
            return self.generate_triangle(self.side_length, self.points_per_side)
        else:
            self.get_logger().warn(f"⚠️ Trajectory '{trajectory_type}' not recognized. Defaulting to square.")
            return self.generate_square(self.side_length, self.points_per_side)

    def generate_square(self, side, points):
        corners = [
            (0.0, 0.0),
            (side, 0.0),
            (side, side),
            (0.0, side),
            (0.0, 0.0)
        ]
        waypoints = []
        for i in range(len(corners) - 1):
            x0, y0 = corners[i]
            x1, y1 = corners[i + 1]
            for j in range(points):
                t = j / (points - 1)
                x = x0 + t * (x1 - x0)
                y = y0 + t * (y1 - y0)
                waypoints.append((x, y))
        return waypoints

    def generate_line(self, length, points):
        return [(length * (i / (points - 1)), 0.0) for i in range(points)]

    def generate_circle(self, radius, points):
        waypoints = []
        for i in range(points):
            angle = (2 * math.pi * i) / points
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            waypoints.append((x, y))
        return waypoints

    def generate_triangle(self, side, points):
        corners = [
            (0.0, 0.0),
            (side, 0.0),
            (side / 2, math.sqrt(3) / 2 * side),
            (0.0, 0.0)
        ]
        waypoints = []
        for i in range(len(corners) - 1):
            x0, y0 = corners[i]
            x1, y1 = corners[i + 1]
            for j in range(points):
                t = j / (points - 1)
                x = x0 + t * (x1 - x0)
                y = y0 + t * (y1 - y0)
                waypoints.append((x, y))
        return waypoints

    def publish_goal(self):
        vec = Vector3()
        vec.x, vec.y, vec.z = self.goal[0], self.goal[1], 0.0
        self.setpoint_pub.publish(vec)
        self.get_logger().info(f"📌 Nuevo goal publicado: ({vec.x:.2f}, {vec.y:.2f})")

    def odom_callback(self, msg):
        if self.current_index >= len(self.waypoints):
            return  # 🔵 Ya terminó todos los waypoints

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        dx = x - self.goal[0]
        dy = y - self.goal[1]
        dist = math.hypot(dx, dy)

        if dist <= self.goal_tolerance:
            self.get_logger().info(f"✅ Waypoint alcanzado: ({self.goal[0]:.2f}, {self.goal[1]:.2f})")
            self.current_index += 1
            if self.current_index < len(self.waypoints):
                self.goal = self.waypoints[self.current_index]
                self.publish_goal()
            else:
                self.get_logger().info("🏁 Todos los waypoints completados.")

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
