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
        self.stop_navigation_time = None  # Para controlar navegación después de stop
        self.yellow_timer = None
        self.give_way_timer = None
        self.construction_timer = None
        self.sign_detection_time = None
        self.last_traffic_light_time = None  # Para timeout del semáforo

        # Timing constants
        self.MIN_STOP_TIME = 1.0  
        self.STOP_NAVIGATION_TIME = 10.0  # 10 segundos de navegación después de stop
        self.MIN_YELLOW_TIME = 12.0
        self.MIN_GIVE_WAY = 7.0
        self.MIN_CONSTRUCTION = 7.0
        self.SIGN_TIMEOUT = 3.0
        self.DIRECTION_SIGN_VALID_TIME = 30.0
        self.TRAFFIC_LIGHT_TIMEOUT = 5.0  # Timeout para semáforo

        # Current state
        self.current_state = RobotState.GO
        self.current_max_vel = self.VEL_GO

        # State tracking
        self.red_light_active = False
        self.yellow_light_active = False
        self.green_light_active = False
        self.stop_sign_completed = False
        self.intersection_waiting_for_green = False

        # NUEVA VARIABLE: Control de prioridad absoluta del stop
        self.stop_sign_active = False  # Flag para controlar prioridad absoluta

        # Priority tracking
        self.priority_source = "none"

        # Timer para ejecutar la máquina de estados periódicamente
        self.timer = self.create_timer(0.1, self.state_machine)

    def traffic_light_callback(self, msg):
        new_color = msg.data.strip().lower()
        if new_color != self.traffic_light_color:
            self.get_logger().info(f"🚦 Traffic light changed: {self.traffic_light_color} -> {new_color}")
        
        self.traffic_light_color = new_color
        self.last_traffic_light_time = time.time()

    def intersection_callback(self, msg):
        intersection_status = msg.data.strip().lower() == "intersection"
        if intersection_status != self.intersection_detected:
            self.get_logger().info(f"🚦 Intersection status: {intersection_status}")
        self.intersection_detected = intersection_status

    def traffic_sign_callback(self, msg):
        sign = msg.data.strip().lower()
        
        # Actualizar el timestamp cada vez que se detecta una señal
        self.sign_detection_time = time.time()
        
        # Solo agregar la nueva señal
        if sign not in self.detected_traffic_signs:
            self.detected_traffic_signs.add(sign)
            self.get_logger().info(f"🚧 New traffic sign detected: {sign.upper()}")

        # Actualizar señales según prioridad
        if sign == "stop":
            if self.velocity_sign != "stop":
                self.velocity_sign = "stop"
                self.stop_sign_completed = False
                self.stop_sign_active = True  # ACTIVAR PRIORIDAD ABSOLUTA
                self.get_logger().info("🛑 STOP SIGN PRIORITY ACTIVATED - OVERRIDING ALL OTHER SIGNALS")
        elif sign in ["giveway", "construction"] and self.velocity_sign not in ["stop"]:
            # Solo actualizar si no hay stop activo
            if not self.stop_sign_active:
                self.velocity_sign = sign
        elif sign in ["left", "right", "forward"]:
            if self.direction_sign == "none" or self.is_direction_sign_expired():
                self.direction_sign = sign
                self.direction_sign_timestamp = time.time()
                self.get_logger().info(f"🧭 Direction sign detected: {sign.upper()}")

    def turn_complete_callback(self, msg):
        if msg.data:
            self.get_logger().info("✅ Turn/Forward motion completed.")
            self.turning = False
            self.nav_enable_pub.publish(Bool(data=True))
            self.intersection_detected = False
            self.intersection_waiting_for_green = False
            self.direction_sign = "none"
            self.direction_sign_timestamp = None
        else:
            self.turning = True
            self.nav_enable_pub.publish(Bool(data=False))

    def is_direction_sign_expired(self):
        if self.direction_sign_timestamp is None:
            return True
        return time.time() - self.direction_sign_timestamp > self.DIRECTION_SIGN_VALID_TIME

    def is_traffic_light_active(self):
        """Verifica si el semáforo está activo (recibiendo señales)"""
        if self.last_traffic_light_time is None:
            return False
        return time.time() - self.last_traffic_light_time <= self.TRAFFIC_LIGHT_TIMEOUT

    def reset_sign_timers(self):
        self.stop_detected_time = None
        self.stop_navigation_time = None
        self.give_way_timer = None
        self.construction_timer = None

    def update_velocities(self):
        if self.current_state == RobotState.STOP:
            self.current_max_vel = self.VEL_STOP
        elif self.current_state == RobotState.SLOW:
            self.current_max_vel = self.VEL_SLOW
        elif self.current_state == RobotState.GO:
            self.current_max_vel = self.VEL_GO
        
        self.max_vel_pub.publish(Float32(data=self.current_max_vel))
        self.get_logger().info(f"🚦 State: {self.current_state.name} | Vel: {self.current_max_vel:.3f} | Priority: {self.priority_source}")

    def check_traffic_light(self):
        """Manejo de semáforos - SOLO SI NO HAY STOP ACTIVO"""
        # SI HAY STOP ACTIVO, IGNORAR COMPLETAMENTE EL SEMÁFORO
        if self.stop_sign_active:
            return False
            
        if not self.is_traffic_light_active():
            # Resetear estados del semáforo si no hay señal
            if self.red_light_active or self.yellow_light_active or self.green_light_active:
                self.get_logger().info("🚦 Traffic light signal lost - resetting states")
                self.red_light_active = False
                self.yellow_light_active = False
                self.green_light_active = False
                self.intersection_waiting_for_green = False
            return False

        # ROJO - Máxima prioridad, siempre para el robot
        if self.traffic_light_color == "red":
            if not self.red_light_active:
                self.red_light_active = True
                self.yellow_light_active = False
                self.green_light_active = False
                self.get_logger().info("🔴 RED LIGHT - Robot stopped")
            
            self.nav_enable_pub.publish(Bool(data=False))  # Desactivar navegación
            self.current_state = RobotState.STOP
            self.priority_source = "red_light"
            self.update_velocities()
            return True
            
        # AMARILLO - Permite intersecciones pero reduce velocidad cuando no hay intersección
        elif self.traffic_light_color == "yellow":
            if not self.yellow_light_active:
                self.yellow_light_active = True
                self.red_light_active = False
                self.green_light_active = False
                self.get_logger().info("🟡 YELLOW LIGHT - Intersections allowed, reduced speed otherwise")
            
            # Si hay intersección detectada, permitir que se procese
            if self.intersection_detected:
                self.get_logger().info("🟡 Yellow light + intersection detected - allowing intersection processing")
                return False  # Permitir que check_intersection() tome control
            
            # Si no hay intersección, aplicar velocidad lenta
            if not self.turning:
                self.current_state = RobotState.SLOW
                self.priority_source = "yellow_light"
                self.update_velocities()
            
            return False  # No bloquear otras funciones
            
        # VERDE - Permite movimiento normal
        elif self.traffic_light_color == "green":
            was_red = self.red_light_active
            
            if not self.green_light_active:
                self.green_light_active = True
                self.red_light_active = False
                self.yellow_light_active = False
                self.get_logger().info("🟢 GREEN LIGHT - Movement allowed")
            
            # Si estábamos esperando verde en intersección, ahora podemos proceder
            if self.intersection_waiting_for_green:
                self.intersection_waiting_for_green = False
                self.get_logger().info("🟢 Green light received - intersection can proceed")
            
            # Permitir que otras funciones (como intersecciones) tomen control
            return False
        
        return False

    def check_intersection(self):
        """Manejo de intersecciones - FUNCIONA CON VERDE Y AMARILLO"""
        # SI HAY STOP ACTIVO, IGNORAR COMPLETAMENTE LAS INTERSECCIONES
        if self.stop_sign_active:
            return False
            
        if not self.intersection_detected:
            if not self.turning:
                self.nav_enable_pub.publish(Bool(data=True))
            return False

        # Solo bloquear intersección si hay luz ROJA (no amarilla)
        if self.red_light_active:
            if not self.intersection_waiting_for_green:
                self.intersection_waiting_for_green = True
                self.get_logger().info("🚦 Intersection detected but red light active - waiting for green")
            return False  # No manejar intersección hasta que esté verde

        # PERMITIR INTERSECCIÓN CON VERDE O AMARILLO
        if self.traffic_light_color in ["green", "yellow"] or not self.is_traffic_light_active():
            if not self.turning:
                self.current_state = RobotState.STOP
                self.priority_source = "intersection"
                self.update_velocities()
                
                if self.direction_sign in ["left", "right", "forward"] and not self.is_direction_sign_expired():
                    self.turning = True
                    self.turn_pub.publish(String(data=self.direction_sign))
                    self.nav_enable_pub.publish(Bool(data=False))
                    self.get_logger().info(f"🔄 Executing turn: {self.direction_sign.upper()} (Traffic light: {self.traffic_light_color})")
                else:
                    self.get_logger().warn("🚦 Intersection detected but no valid direction sign available")
            
            return True
        
        return False

    def check_sign_timeout(self):
        """Verifica si las señales han desaparecido por timeout"""
        if (self.sign_detection_time and 
            time.time() - self.sign_detection_time > self.SIGN_TIMEOUT):
            
            if self.detected_traffic_signs:
                self.get_logger().info("🚧 Traffic signs timeout - clearing all signs")
            
            self.detected_traffic_signs.clear()
            self.velocity_sign = "none"
            self.reset_sign_timers()
            self.sign_detection_time = None
            self.stop_sign_completed = False
            
            # RESETEAR TAMBIÉN LA PRIORIDAD ABSOLUTA DEL STOP
            if self.stop_sign_active:
                self.stop_sign_active = False
                self.get_logger().info("🛑 STOP SIGN PRIORITY DEACTIVATED - Normal operation resumed")

    def check_traffic_signs(self):
        """Manejo de señales de tráfico - STOP tiene prioridad absoluta"""
        self.check_sign_timeout()
        
        # MANEJO DEL STOP CON PRIORIDAD ABSOLUTA
        if self.velocity_sign == "stop" and not self.stop_sign_completed:
            if not self.stop_detected_time:
                # Iniciar período de avance por 12 segundos
                self.stop_detected_time = time.time()
                self.current_state = RobotState.GO  # Mantener avanzando
                self.priority_source = "stop_sign_advance"
                self.update_velocities()
                self.nav_enable_pub.publish(Bool(data=True))  # Mantener navegación activa
                self.get_logger().info("🛑 STOP SIGN ACTIVE - Advancing for 12 seconds (IGNORING ALL OTHER SIGNALS)")
                
            elif time.time() - self.stop_detected_time >= 12.0:  # 12 segundos de avance
                # 12 segundos completados, parar completamente
                self.current_state = RobotState.STOP
                self.priority_source = "stop_final"
                self.update_velocities()
                self.nav_enable_pub.publish(Bool(data=False))  # Desactivar navegación
                self.stop_sign_completed = False
                self.get_logger().info("🛑 STOP SIGN ADVANCE COMPLETED - FINAL STOP (IGNORING ALL OTHER SIGNALS)")
                return True

        # Una vez completado el stop, ignorar todo hasta que se reinicie manualmente
        elif self.stop_sign_completed:
            # Mantener el estado de parada final
            self.current_state = RobotState.STOP
            self.nav_enable_pub.publish(Bool(data=False))
            self.get_logger().info("🛑 STOP COMPLETED - ROBOT STOPPED (IGNORING ALL SIGNALS)")
            return True
        
        # OTRAS SEÑALES - SOLO SI NO HAY STOP ACTIVO
        elif not self.stop_sign_active:
            if self.velocity_sign == "giveway":
                if not self.give_way_timer:
                    self.give_way_timer = time.time()
                    self.current_state = RobotState.SLOW
                    self.priority_source = "giveway_sign"
                    self.update_velocities()
                    self.get_logger().info("⚠️ Give Way sign detected - slowing down")
                elif time.time() - self.give_way_timer > self.MIN_GIVE_WAY:
                    self.give_way_timer = None
                    self.velocity_sign = "none"
                    self.get_logger().info("⚠️ Give Way completed")
                    return False
                return True
                
            elif self.velocity_sign == "construction":
                if not self.construction_timer:
                    self.construction_timer = time.time()
                    self.current_state = RobotState.SLOW
                    self.priority_source = "construction_sign"
                    self.update_velocities()
                    self.get_logger().info("🚧 Construction sign detected - slowing down")
                elif time.time() - self.construction_timer > self.MIN_CONSTRUCTION:
                    self.construction_timer = None
                    self.velocity_sign = "none"
                    self.get_logger().info("🚧 Construction completed")
                    return False
                return True
        
        return False

    def state_machine(self):
        """Máquina de estados con STOP como prioridad absoluta"""
        
        # PRIORIDAD ABSOLUTA: Señales de tráfico (específicamente STOP)
        if self.check_traffic_signs():
            return
        
        # Solo procesar otras prioridades si NO hay stop activo
        if not self.stop_sign_active:
            # Prioridad 1: Semáforos (máxima prioridad después de stop)
            if self.check_traffic_light():
                return
            
            # Prioridad 2: Intersecciones (solo si no hay luz roja)
            if self.check_intersection():
                return
        
        # Estado por defecto: GO si no hay otras condiciones activas
        if (self.current_state != RobotState.GO and not self.turning and 
            not self.stop_sign_active and not self.stop_sign_completed):
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