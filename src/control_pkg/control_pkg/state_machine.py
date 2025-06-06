#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
import time
from enum import Enum

class RobotState(Enum):
    STOP = 0
    SLOW = 1
    GO = 2

class PuzzlebotVelocityController(Node):
    def __init__(self):
        super().__init__('puzzlebot_velocity_controller')

        # Subscriptions
        self.traffic_light_sub = self.create_subscription(String, '/traffic_light_color', self.traffic_light_callback, 10)
        self.intersection_sub = self.create_subscription(String, '/intersection', self.intersection_callback, 10)
        self.traffic_sign_sub = self.create_subscription(String, '/detected_traffic_sign', self.traffic_sign_callback, 10)
        self.turn_complete_sub = self.create_subscription(Bool, '/turn_complete', self.turn_complete_callback, 10)

        # Publishers
        self.max_vel_pub = self.create_publisher(Float32, '/max_vel', 10)
        self.turn_pub = self.create_publisher(String, '/turn_command', 10)
        self.navigation_enable_pub = self.create_publisher(Bool, '/navigation_enable', 10)

        # Internal state
        self.traffic_light_color = "none"
        self.intersection_detected = False
        self.waiting_for_turn = False
        self.traffic_sign = None
        self.intersection_time = None
        self.waiting_for_green = False
        self.turn_in_progress = False
        self.last_turn_command = None
        
        # New timing variables
        self.give_way_start_time = None
        self.construction_start_time = None
        self.stop_sign_start_time = None
        self.yellow_light_start_time = None
        self.min_give_way_duration = 5.0  # seconds
        self.min_construction_duration = 5.0  # seconds
        self.min_stop_sign_duration = 10.0  # seconds
        self.min_yellow_duration = 7.0  # seconds

        self.current_state = RobotState.GO
        self.current_max_vel = 0.16

        self.evaluate_state_and_publish()

    def turn_complete_callback(self, msg):
        if msg.data and self.turn_in_progress:
            self.turn_in_progress = False
            self.waiting_for_turn = False
            self.intersection_detected = False
            self.intersection_time = None
            self.traffic_sign = None
            self.current_state = RobotState.GO
            self.current_max_vel = 0.16

            self.publish_navigation_enable(True)
            self.get_logger().info("✅ Turn completed, reactivating navigation")

    def traffic_light_callback(self, msg):
        current_time = time.time()
        new_color = msg.data

        # Handle red light - complete stop until green
        if new_color == "red":
            self.yellow_light_start_time = None  # Reset yellow timer
            self.current_state = RobotState.STOP
            self.current_max_vel = 0.0
            self.publish_navigation_enable(False)
            self.get_logger().info("🔴 Red light detected — Complete stop until green")
            self.traffic_light_color = new_color
            self.publish_max_vel()
            return

        # Handle yellow light timing
        if new_color == "yellow":
            if self.yellow_light_start_time is None:
                self.yellow_light_start_time = current_time
                self.current_state = RobotState.SLOW
                self.current_max_vel = 0.07
                self.get_logger().info("🟡 Yellow light detected — Slowing down for 7 seconds")
        elif self.yellow_light_start_time is not None:
            if current_time - self.yellow_light_start_time < self.min_yellow_duration:
                # Maintain slow speed for full 7 seconds
                self.current_state = RobotState.SLOW
                self.current_max_vel = 0.07
            else:
                self.yellow_light_start_time = None
                if new_color == "green":
                    self.current_state = RobotState.GO
                    self.current_max_vel = 0.16
                    self.publish_navigation_enable(True)
                    self.get_logger().info("🟢 Yellow duration completed, green light detected — Maximum speed")

        # Handle green light
        if new_color == "green":
            if self.yellow_light_start_time is None:  # Only if not in yellow timing
                self.current_state = RobotState.GO
                self.current_max_vel = 0.16
                self.publish_navigation_enable(True)
                self.get_logger().info("🟢 Green light detected — Maximum speed and navigation enabled")

        self.traffic_light_color = new_color

        if self.waiting_for_green and new_color == "green":
            self.waiting_for_green = False
            self.publish_navigation_enable(True)
            self.current_state = RobotState.GO
            self.current_max_vel = 0.16

        self.evaluate_state_and_publish()

    def intersection_callback(self, msg):
        if msg.data == "intersection":
            self.intersection_detected = True
            self.waiting_for_turn = True
            self.intersection_time = time.time()

            self.publish_navigation_enable(False)
            self.current_state = RobotState.STOP
            self.current_max_vel = 0.0

            self.evaluate_state_and_publish()

    def traffic_sign_callback(self, msg):
        current_time = time.time()
        new_sign = msg.data

        # Handle Stop sign timing
        if new_sign == "Stop":
            if self.stop_sign_start_time is None:
                self.stop_sign_start_time = current_time
                self.current_state = RobotState.GO
                self.current_max_vel = 0.16
                self.publish_navigation_enable(True)
                self.get_logger().info("🛑 Stop sign detected — Continuing navigation for 6 seconds")
            elif current_time - self.stop_sign_start_time >= self.min_stop_sign_duration:
                self.current_state = RobotState.STOP
                self.current_max_vel = 0.0
                self.publish_navigation_enable(False)
                self.get_logger().info("🛑 Stop sign duration completed — Robot stopped")
        else:
            self.stop_sign_start_time = None

        # Handle Give Way sign timing
        if new_sign == "GiveWay":
            if self.give_way_start_time is None:
                self.give_way_start_time = current_time
                self.current_state = RobotState.SLOW
                self.current_max_vel = 0.07
                self.get_logger().info("⚠️ Give Way sign detected — Slowing down for 5 seconds")
        elif self.give_way_start_time is not None:
            if current_time - self.give_way_start_time < self.min_give_way_duration:
                self.current_state = RobotState.SLOW
                self.current_max_vel = 0.07
            else:
                self.give_way_start_time = None

        # Handle Construction sign timing
        if new_sign == "Construction":
            if self.construction_start_time is None:
                self.construction_start_time = current_time
                self.current_state = RobotState.SLOW
                self.current_max_vel = 0.07
                self.get_logger().info("🚧 Construction sign detected — Slowing down for 5 seconds")
        elif self.construction_start_time is not None:
            if current_time - self.construction_start_time < self.min_construction_duration:
                self.current_state = RobotState.SLOW
                self.current_max_vel = 0.07
            else:
                self.construction_start_time = None

        # Handle intersection logic
        if self.waiting_for_turn and not self.turn_in_progress:
            time_since_intersection = time.time() - self.intersection_time if self.intersection_time else float('inf')

            if time_since_intersection < 10.0:
                if new_sign in ["Right", "left", "Forward"]:
                    self.turn_in_progress = True
                    self.last_turn_command = new_sign
                    self.send_turn_command(new_sign)
                    self.get_logger().info(f"➡️ Handling turn at intersection: {new_sign}")
                elif self.traffic_light_color == "red":
                    self.waiting_for_green = True
                    self.current_state = RobotState.STOP
                    self.current_max_vel = 0.0

        self.publish_max_vel()
        self.evaluate_state_and_publish()

    def send_turn_command(self, direction):
        msg = String()
        msg.data = direction
        self.turn_pub.publish(msg)
        self.get_logger().info(f"📨 Sent turn command: {direction}")

    def publish_max_vel(self):
        vel_msg = Float32()
        vel_msg.data = self.current_max_vel
        self.max_vel_pub.publish(vel_msg)

    def publish_navigation_enable(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.navigation_enable_pub.publish(msg)
        self.get_logger().info(f"🔁 Navigation enable = {enable}")

    def evaluate_state_and_publish(self):
        if self.intersection_detected:
            return

        if self.traffic_light_color == "red":
            self.current_state = RobotState.STOP
            self.current_max_vel = 0.0
        elif self.traffic_light_color == "yellow":
            self.current_state = RobotState.SLOW
            self.current_max_vel = 0.07
        elif self.traffic_light_color in ["green", "none"]:
            if self.traffic_sign == "Stop":
                self.current_state = RobotState.STOP
                self.current_max_vel = 0.0
            elif self.traffic_sign in ["GiveWay", "Construction"]:
                self.current_state = RobotState.SLOW
                self.current_max_vel = 0.07
            else:
                self.current_state = RobotState.GO
                self.current_max_vel = 0.16

        self.publish_max_vel()

def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
