#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
import time

class TurnExecutorNode(Node):
    def __init__(self):
        super().__init__('turn_executor_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.turn_cmd_sub = self.create_subscription(String, '/turn_command', self.turn_callback, 10)
        self.turn_complete_pub = self.create_publisher(Bool, '/turn_complete', 10)

        self.turning = False
        self.last_turn_time = 0
        self.turn_cooldown = 5  # seconds

        # Motion parameters
        self.linear_speed = 0.14         # m/s
        self.advance_distance = 0.40     # meters
        self.additional_distance = 0.25  # meters
        self.angular_speed = 0.5         # rad/s
        self.turn_duration = 3.2         # seconds for 90° turn

        self.get_logger().info("🔄 Turn Executor Node (FSM-driven) ready")

    def turn_callback(self, msg):
        command = msg.data.strip().lower()
        now = time.time()

        if self.turning or (now - self.last_turn_time < self.turn_cooldown):
            return

        if command in ["right", "left", "forward"]:
            self.get_logger().info(f"📥 Turn command received from FSM: {command.upper()}")
            self.turning = True
            self.last_turn_time = now
            self.execute_turn(command)

    def execute_turn(self, direction):
        # Step 1: Advance 35 cm
        advance_time = self.advance_distance / self.linear_speed
        advance_msg = Twist()
        advance_msg.linear.x = self.linear_speed
        advance_msg.angular.z = 0.0

        self.get_logger().info(f"🚗 Advancing {self.advance_distance*100:.0f} cm for {advance_time:.2f} s...")
        start_time = time.time()
        while time.time() - start_time < advance_time:
            self.cmd_vel_pub.publish(advance_msg)
            time.sleep(0.05)

        self.stop_motion()

        # Step 2: Turn only if it's right or left
        if direction in ["right", "left"]:
            turn_msg = Twist()
            turn_msg.linear.x = 0.0
            turn_msg.angular.z = -self.angular_speed if direction == "right" else self.angular_speed

            self.get_logger().info(f"↪️ Executing {direction.upper()} turn for {self.turn_duration:.2f} s...")
            start_time = time.time()
            while time.time() - start_time < self.turn_duration:
                self.cmd_vel_pub.publish(turn_msg)
                time.sleep(0.05)

            self.stop_motion()

        # Step 3: Additional advance 10 cm
        additional_time = self.additional_distance / self.linear_speed
        self.get_logger().info(f"🚗 Advancing additional {self.additional_distance*100:.0f} cm for {additional_time:.2f} s...")
        start_time = time.time()
        while time.time() - start_time < additional_time:
            self.cmd_vel_pub.publish(advance_msg)
            time.sleep(0.05)

        self.stop_motion()
        self.turning = False

        # Publish completion signal
        complete_msg = Bool()
        complete_msg.data = True
        self.turn_complete_pub.publish(complete_msg)
        self.get_logger().info("✅ Turn/Forward motion completed. Signal sent.")

    def stop_motion(self):
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = TurnExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
