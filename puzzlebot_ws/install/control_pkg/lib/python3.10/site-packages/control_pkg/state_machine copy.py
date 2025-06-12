#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from enum import Enum
import time

class RobotState(Enum):
    STOP = 0
    SLOW = 1
    GO = 2

class PuzzlebotVelocityController(Node):
    def __init__(self):
        super().__init__('puzzlebot_state_machine')

        # Subscribers
        self.create_subscription(String, '/traffic_light_color', self.traffic_light_callback, 10)
        self.create_subscription(String, '/intersection', self.intersection_callback, 10)
        self.create_subscription(String, '/detected_traffic_sign', self.traffic_sign_callback, 10)
        self.create_subscription(Bool, '/turn_complete', self.turn_complete_callback, 10)

        # Publishers
        self.max_vel_pub = self.create_publisher(Float32, '/max_vel', 10)
        self.turn_pub = self.create_publisher(String, '/turn_command', 10)
        self.nav_enable_pub = self.create_publisher(Bool, '/navigation_enable', 10)

        # State variables
        self.traffic_light_color = "none"
        self.detected_traffic_signs = set()
        self.intersection_detected = False
        self.direction_sign = "none"
        self.direction_sign_timestamp = None
        self.velocity_sign = "none"
        self.turning = False

        # Velocity constants
        self.VEL_GO = 0.14
        self.VEL_SLOW = 0.05
        self.VEL_STOP = 0.0

        # Timers
        self.stop_detected_time = None
        self.yellow_timer = None
        self.give_way_timer = None
        self.construction_timer = None
        self.sign_detection_time = None  # Para trackear cuándo se detectó la última señal

        # Timing constants
        self.MIN_STOP_TIME = 15.0  
        self.MIN_YELLOW_TIME = 12.0
        self.MIN_GIVE_WAY = 7.0
        self.MIN_CONSTRUCTION = 7.0
        self.SIGN_TIMEOUT = 3.0  # Tiempo para considerar que una señal ya no está presente
        self.DIRECTION_SIGN_VALID_TIME = 30.0  # Tiempo que una señal direccional sigue siendo válida

        # Current state
        self.current_state = RobotState.GO
        self.current_max_vel = self.VEL_GO

        # For restoring state after red light
        self.previous_state = RobotState.GO
        self.previous_max_vel = self.VEL_GO
        self.red_light_active = False

        # Priority tracking
        self.priority_source = "none"  # "traffic_light", "stop_sign", "intersection", etc.

        # Timer para ejecutar la máquina de estados periódicamente (cada 0.1 seg)
        self.timer = self.create_timer(0.1, self.state_machine)

    def traffic_light_callback(self, msg):
        self.traffic_light_color = msg.data.strip().lower()

    def intersection_callback(self, msg):
        if msg.data.strip().lower() == "intersection":
            self.intersection_detected = True
        else:
            self.intersection_detected = False

    def traffic_sign_callback(self, msg):
        sign = msg.data.strip().lower()
        
        # Solo agregar la nueva señal sin limpiar las existentes
        self.detected_traffic_signs.add(sign)
        self.sign_detection_time = time.time()

        # Actualizar señales según prioridad
        if sign == "stop":
            self.velocity_sign = "stop"
        elif sign in ["giveway", "construction"] and self.velocity_sign != "stop":
            self.velocity_sign = sign
        elif sign in ["left", "right", "forward"]:
            # Las señales direccionales se conservan con timestamp
            if self.direction_sign == "none" or self.is_direction_sign_expired():
                self.direction_sign = sign
                self.direction_sign_timestamp = time.time()
                self.get_logger().info(f"🧭 Direction sign detected: {sign.upper()}")
            else:
                self.get_logger().info(f"🧭 Direction sign {sign.upper()} ignored - {self.direction_sign.upper()} still valid")
        else:
            self.get_logger().info(f"Ignoring Traffic Sign: {sign}")

    def turn_complete_callback(self, msg):
        if msg.data:
            self.get_logger().info("Turn/Forward motion completed. Signal received.")
            self.turning = False
            # Reactivar navegación al completar giro
            self.nav_enable_pub.publish(Bool(data=True))
            self.intersection_detected = False
            # Resetear señal direccional después de usarla
            self.direction_sign = "none"
            self.direction_sign_timestamp = None
        else:
            self.get_logger().warn("Turn/Forward motion not completed. Waiting for completion signal.")
            self.turning = True
            # Desactivar navegación mientras gira
            self.nav_enable_pub.publish(Bool(data=False))

    def is_direction_sign_expired(self):
        """Verifica si la señal direccional ha expirado"""
        if self.direction_sign_timestamp is None:
            return True
        return time.time() - self.direction_sign_timestamp > self.DIRECTION_SIGN_VALID_TIME

    def reset_sign_timers(self):
        """Resetea todos los timers de señales de tráfico"""
        self.stop_detected_time = None
        self.give_way_timer = None
        self.construction_timer = None

    def update_velocities(self):
        if self.current_state == RobotState.STOP:
            self.current_max_vel = self.VEL_STOP
            self.max_vel_pub.publish(Float32(data=self.current_max_vel))
            self.get_logger().info(f"🚦 State: STOP (Priority: {self.priority_source})")
        elif self.current_state == RobotState.SLOW:
            self.current_max_vel = self.VEL_SLOW
            self.max_vel_pub.publish(Float32(data=self.current_max_vel))
            self.get_logger().info(f"🚦 State: SLOW (Priority: {self.priority_source})")
        elif self.current_state == RobotState.GO:
            self.current_max_vel = self.VEL_GO
            self.max_vel_pub.publish(Float32(data=self.current_max_vel))
            self.get_logger().info(f"🚦 State: GO (Priority: {self.priority_source})")
            
    def check_traffic_light(self):
        """Manejo de semáforos con máxima prioridad"""
        if self.traffic_light_color == "red":
            if not self.red_light_active:
                self.red_light_active = True
                self.previous_state = self.current_state
                self.current_state = RobotState.STOP
                self.priority_source = "red_light"
                self.update_velocities()
                self.get_logger().info("🚦 Red light detected. Stopping robot.")
            return True  # Red light has priority, skip other checks
            
        elif self.traffic_light_color == "yellow":
            if not self.yellow_timer:
                if not self.red_light_active:  # Solo si no hay luz roja activa
                    # Solo cambiar a SLOW si no hay intersección activa
                    if not self.intersection_detected or self.turning:
                        self.current_state = RobotState.SLOW
                        self.priority_source = "yellow_light"
                        self.update_velocities()
                self.yellow_timer = time.time()
            elif time.time() - self.yellow_timer > self.MIN_YELLOW_TIME:
                self.yellow_timer = None
                if not self.red_light_active:
                    self.current_state = RobotState.GO
                    self.priority_source = "none"
                    self.update_velocities()
                    self.get_logger().info("🚦 Yellow light duration exceeded. Resuming Normal Functions.")
                
            # Durante amarillo, permitir procesamiento de intersecciones
            return False
            
        elif self.traffic_light_color == "green":
            if self.red_light_active:
                self.red_light_active = False
                if self.yellow_timer:  # Solo resetear si no hay amarillo activo
                    self.yellow_timer = None
                self.current_state = RobotState.GO
                self.priority_source = "none"
                self.update_velocities()
                self.get_logger().info("🚦 Green light detected. Resuming Functions.")
        
        return self.red_light_active

    def check_intersection(self):
        """Manejo de intersecciones - segunda prioridad"""
        if self.intersection_detected:
            if not self.turning:
                self.current_state = RobotState.STOP
                self.priority_source = "intersection"
                self.update_velocities()
                self.get_logger().info("🚦 Intersection detected. Stopping.")
                
                # Verificar si tenemos una señal direccional válida
                if self.direction_sign in ["left", "right", "forward"] and not self.is_direction_sign_expired():
                    self.turning = True
                    self.turn_pub.publish(String(data=self.direction_sign))
                    self.nav_enable_pub.publish(Bool(data=False))
                    self.get_logger().info(f"📥 Turn command sent: {self.direction_sign.upper()}")
                else:
                    if self.is_direction_sign_expired():
                        self.get_logger().warn(f"🚦 Direction sign {self.direction_sign.upper()} expired. Waiting for new direction...")
                        self.direction_sign = "none"
                        self.direction_sign_timestamp = None
                    else:
                        self.get_logger().warn("🚦 Intersection detected but no valid direction sign available. Waiting...")
            return True  # Intersection active
        else:
            if not self.turning:
                self.nav_enable_pub.publish(Bool(data=True))
            return False

    def check_sign_timeout(self):
        """Verifica si las señales han desaparecido por timeout"""
        if (self.sign_detection_time and 
            time.time() - self.sign_detection_time > self.SIGN_TIMEOUT):
            
            # Reset todas las señales y timers
            self.detected_traffic_signs.clear()
            self.velocity_sign = "none"
            self.reset_sign_timers()
            self.sign_detection_time = None
            self.get_logger().info("🚦 Traffic signs timeout. Clearing all signs.")

    def check_traffic_signs(self):
        """Manejo de señales de tráfico - tercera prioridad"""
        self.check_sign_timeout()
        
        if self.velocity_sign == "stop":
            if not self.stop_detected_time:
                self.stop_detected_time = time.time()
                self.get_logger().info("🚦 Stop sign detected. Stopping robot.")
            else:
                # Verificar si han pasado 15 segundos desde la detección de stop
                if time.time() - self.stop_detected_time >= self.MIN_STOP_TIME:
                    self.get_logger().info("🚦 Stop sign stop time elapsed. Robot cannot move again.")
                    self.current_state = RobotState.STOP
                    self.update_velocities()
                    self.stop_detected_time = None  # Reset the stop timer
                    self.velocity_sign = "none"  # Reset the s
                    self.nav_enable_pub.publish(Bool(data=False))  # Desactivar navegación
            return True  # Stop sign active
            
        elif self.velocity_sign == "giveway":
            if not self.give_way_timer:
                self.give_way_timer = time.time()
                self.current_state = RobotState.SLOW
                self.priority_source = "giveway_sign"
                self.update_velocities()
                self.get_logger().info("🚦 Give Way sign detected. Slowing down.")
            elif time.time() - self.give_way_timer > self.MIN_GIVE_WAY:
                self.give_way_timer = None
                self.velocity_sign = "none"  # Reset the sign
                self.get_logger().info("🚦 Give Way sign time elapsed. Resuming normal speed.")
                return False
            return True
            
        elif self.velocity_sign == "construction":
            if not self.construction_timer:
                self.construction_timer = time.time()
                self.current_state = RobotState.SLOW
                self.priority_source = "construction_sign"
                self.update_velocities()
                self.get_logger().info("🚧 Construction sign detected. Slowing down.")
            elif time.time() - self.construction_timer > self.MIN_CONSTRUCTION:
                self.construction_timer = None
                self.velocity_sign = "none"  # Reset the sign
                self.get_logger().info("🚦 Construction sign time elapsed. Resuming normal speed.")
                return False
            return True
        
        return False  # No active traffic signs

    def state_machine(self):
        """Máquina de estados con prioridades claras"""
        # Prioridad 1: Semáforos (máxima prioridad)
        if self.check_traffic_light():
            return
        
        # Prioridad 2: Intersecciones
        if self.check_intersection():
            return
        
        # Prioridad 3: Señales de tráfico
        if self.check_traffic_signs():
            return
        
        # Estado por defecto: GO si no hay otras condiciones activas
        if self.current_state != RobotState.GO and not self.turning:
            self.current_state = RobotState.GO
            self.priority_source = "default"
            self.update_velocities()

def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotVelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()