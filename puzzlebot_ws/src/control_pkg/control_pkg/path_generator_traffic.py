#!/usr/bin/env python3
"""
🚦 Path Generator con Control de Tráfico Optimizado para CUADRADOS PERFECTOS 🚦
=========================================================================

Nodo optimizado específicamente para trazar cuadrados perfectos, con pausas en las esquinas
y comportamiento específico para cada segmento del recorrido.

Subscribes:
- /odom (nav_msgs/Odometry): Posición actual del robot
- /traffic_light_color (std_msgs/String): Color detectado del semáforo

Publishes:
- /setpoint (geometry_msgs/Vector3): Waypoints de la trayectoria
- /cmd_vel (geometry_msgs/Twist): Comandos de velocidad para el robot

Comportamiento según semáforo:
- 🚦 Inicio: Velocidad media óptima (0.2 m/s)
- 🟢 Verde: Aumenta velocidad (0.3 m/s)
- 🟡 Amarillo: Velocidad media (0.2 m/s)
- 🔴 Rojo: Se detiene completamente
"""

import math
import time
import rclpy
from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class PathGeneratorSquare(Node):
    def __init__(self):
        super().__init__('path_generator_square')

        # Colores para terminal
        self.colors = {
            'red': '\033[1;31m',    # Rojo brillante
            'yellow': '\033[1;33m', # Amarillo brillante
            'green': '\033[1;32m',  # Verde brillante
            'reset': '\033[0m',     # Resetear color
            'bold': '\033[1m',      # Negrita
            'blue': '\033[1;34m',   # Azul brillante
            'magenta': '\033[1;35m',# Magenta brillante
            'cyan': '\033[1;36m'    # Cian brillante
        }

        # Parámetros de trayectoria - SOLO PARA CUADRADOS
        self.declare_parameter('side_length', 1.0)
        self.declare_parameter('points_per_side', 12)  # Más puntos por lado
        self.declare_parameter('goal_tolerance', 0.04)  # Tolerancia más estricta
        self.declare_parameter('corner_pause', 0.5)    # Segundos de pausa en cada esquina

        # Parámetros de velocidad
        self.declare_parameter('initial_speed', 0.2)     # Velocidad inicial óptima
        self.declare_parameter('green_speed', 0.3)       # Velocidad con luz verde
        self.declare_parameter('yellow_speed', 0.2)      # Velocidad con luz amarilla
        self.declare_parameter('corner_speed', 0.1)      # Velocidad en esquinas
        self.declare_parameter('acceleration', 0.01)     # Aceleración gradual
        
        # Parámetros PID - Optimizados para cuadrados
        self.declare_parameter('KpPos', 0.8)            # Mayor ganancia proporcional
        self.declare_parameter('KiPos', 0.05)           # Ganancia integral posición
        self.declare_parameter('KdPos', 0.1)            # Mayor ganancia derivativa
        self.declare_parameter('KpHead', 1.5)           # Mayor ganancia para orientación
        self.declare_parameter('KiHead', 0.01)          # Pequeña ganancia integral
        self.declare_parameter('KdHead', 0.2)           # Mayor ganancia derivativa
        self.declare_parameter('max_angular', 1.0)      # Mayor velocidad angular para giros

        # Cargar parámetros
        self.side_length = self.get_parameter('side_length').value
        self.points_per_side = int(self.get_parameter('points_per_side').value)
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.corner_pause = self.get_parameter('corner_pause').value
        
        self.initial_speed = self.get_parameter('initial_speed').value
        self.green_speed = self.get_parameter('green_speed').value
        self.yellow_speed = self.get_parameter('yellow_speed').value
        self.corner_speed = self.get_parameter('corner_speed').value
        self.acceleration = self.get_parameter('acceleration').value
        self.max_angular = self.get_parameter('max_angular').value
        
        # Constantes PID
        self.kp_pos = self.get_parameter('KpPos').value
        self.ki_pos = self.get_parameter('KiPos').value
        self.kd_pos = self.get_parameter('KdPos').value
        self.kp_head = self.get_parameter('KpHead').value
        self.ki_head = self.get_parameter('KiHead').value
        self.kd_head = self.get_parameter('KdHead').value

        # Publishers
        self.setpoint_pub = self.create_publisher(Vector3, 'setpoint', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos.qos_profile_sensor_data
        )
        self.create_subscription(
            String, 
            '/traffic_light_color', 
            self.traffic_light_callback, 
            10
        )

        # Variables de estado de tráfico
        self.current_traffic_light = 'none'  # Estado inicial: ninguno
        self.last_detection_time = self.get_clock().now().nanoseconds * 1e-9
        self.detection_timeout = 2.0        # Segundos sin detección
        self.last_state = 'none'            # Último estado conocido

        # Variables para aceleración gradual
        self.current_speed = 0.0
        self.target_speed = self.initial_speed  # Inicialmente velocidad óptima

        # Variables para el controlador PID
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.prev_error_pos = 0.0
        self.integral_pos = 0.0
        self.prev_error_ang = 0.0
        self.integral_ang = 0.0
        self.prev_sample_time = self.get_clock().now().nanoseconds * 1e-9

        # Waypoints y control de trayectoria
        self.waypoints = self.generate_square(self.side_length, self.points_per_side)
        self.current_index = 0
        
        # Nuevas variables para control de esquinas
        self.corners = self.identify_corners()
        self.at_corner = False
        self.corner_start_time = 0.0
        self.in_corner_sequence = False
        self.corner_alignment_target = 0.0  # Orientación objetivo en la esquina
        
        # Trayectoria en segmentos
        self.segments = self.identify_segments()
        self.current_segment = 0

        if self.waypoints:
            self.goal = self.waypoints[self.current_index]
            self.publish_goal()
            self.get_logger().info(f"🚀 Path Generator Node started with SQUARE trajectory of size {self.side_length}m")
        else:
            self.get_logger().warn("⚠️ No se generaron waypoints.")

        # Timer para el control de velocidad (30Hz)
        self.timer = self.create_timer(0.033, self.control_loop)

        # Imprimir encabezado
        self.print_header()

    def print_header(self):
        c = self.colors
        print(f"\n{c['bold']}{'='*70}{c['reset']}")
        print(f"{c['cyan']}🚦 PATH GENERATOR OPTIMIZADO PARA CUADRADOS PERFECTOS 🚦{c['reset']}")
        print(f"{c['bold']}{'='*70}{c['reset']}")
        print(f"{c['magenta']}🛣️ Trayectoria: CUADRADO de lado {self.side_length}m{c['reset']}")
        print(f"{c['blue']}🚙 Velocidad inicial: {self.initial_speed} m/s{c['reset']}")
        print(f"{c['blue']}🧭 Esquinas: Pausa de {self.corner_pause}s y velocidad reducida a {self.corner_speed} m/s{c['reset']}")
        print(f"{c['yellow']}🚥 Velocidades según semáforo:{c['reset']}")
        print(f"{c['green']}   🟢 Verde: {self.green_speed} m/s{c['reset']}")
        print(f"{c['yellow']}   🟡 Amarillo: {self.yellow_speed} m/s{c['reset']}")
        print(f"{c['red']}   🔴 Rojo: 0.0 m/s (detenido){c['reset']}")
        print(f"{c['bold']}{'='*70}{c['reset']}\n")

    def print_status(self, message, color='blue'):
        c = self.colors
        if color in c:
            print(f"{c[color]}{message}{c['reset']}")
        else:
            print(f"{c['blue']}{message}{c['reset']}")

    def generate_square(self, side, points):
        """Genera waypoints para un cuadrado perfecto con más puntos en las esquinas"""
        waypoints = []
        
        # Esquina inferior izquierda a inferior derecha
        waypoints.extend([(i * side / points, 0.0) for i in range(points + 1)])
        
        # Esquina inferior derecha a superior derecha
        waypoints.extend([(side, i * side / points) for i in range(1, points + 1)])
        
        # Esquina superior derecha a superior izquierda
        waypoints.extend([(side - i * side / points, side) for i in range(1, points + 1)])
        
        # Esquina superior izquierda a inferior izquierda (completar el cuadrado)
        waypoints.extend([(0.0, side - i * side / points) for i in range(1, points)])
        
        return waypoints

    def identify_corners(self):
        """Identifica los índices de las esquinas del cuadrado"""
        corners = []
        points_per_side = self.points_per_side
        
        # Las 4 esquinas del cuadrado
        corners.append(0)  # Inicio (0,0)
        corners.append(points_per_side)  # Esquina inferior derecha
        corners.append(2 * points_per_side)  # Esquina superior derecha
        corners.append(3 * points_per_side)  # Esquina superior izquierda
        
        return corners

    def identify_segments(self):
        """Divide la trayectoria en los 4 segmentos del cuadrado"""
        segments = []
        points_per_side = self.points_per_side
        
        # Los 4 lados del cuadrado
        segments.append((0, points_per_side))  # Lado inferior
        segments.append((points_per_side, 2 * points_per_side))  # Lado derecho
        segments.append((2 * points_per_side, 3 * points_per_side))  # Lado superior
        segments.append((3 * points_per_side, 4 * points_per_side - 1))  # Lado izquierdo
        
        return segments

    def get_segment_target_angle(self, segment_index):
        """Devuelve el ángulo objetivo para cada segmento del cuadrado"""
        # Orientaciones para cada lado del cuadrado (en radianes)
        segment_angles = [0.0, math.pi/2, math.pi, -math.pi/2]
        return segment_angles[segment_index % 4]

    def publish_goal(self):
        vec = Vector3()
        vec.x, vec.y, vec.z = self.goal[0], self.goal[1], 0.0
        self.setpoint_pub.publish(vec)
        
        # Solo mostramos mensajes para waypoints importantes
        if self.current_index in self.corners:
            corner_num = self.corners.index(self.current_index) + 1
            self.get_logger().info(f"📌 ESQUINA {corner_num}: Nuevo waypoint publicado: ({vec.x:.2f}, {vec.y:.2f})")
        elif self.current_index % 3 == 0:  # Cada 3 waypoints en un segmento
            self.get_logger().info(f"📍 Waypoint intermedio: ({vec.x:.2f}, {vec.y:.2f})")

    def traffic_light_callback(self, msg):
        """Callback para manejar detecciones de semáforo"""
        new_color = msg.data.lower()  # Convertir a minúsculas para uniformidad
        
        # Actualizar tiempo de última detección
        self.last_detection_time = self.get_clock().now().nanoseconds * 1e-9
        
        # Guardar siempre el último estado válido
        self.last_state = new_color
        
        # Procesar si hay un cambio de estado
        if new_color != self.current_traffic_light:
            emoji = '🔴' if new_color == 'red' else '🟡' if new_color == 'yellow' else '🟢' if new_color == 'green' else '⚪'
            
            # Comportamiento para cada color
            if new_color == 'red':
                self.print_status(f"\n{emoji} SEMÁFORO ROJO DETECTADO - Deteniendo robot", 'red')
                self.stop_robot()
                self.target_speed = 0.0
                
            elif new_color == 'yellow':
                self.print_status(f"\n{emoji} SEMÁFORO AMARILLO DETECTADO - Velocidad media {self.yellow_speed} m/s", 'yellow')
                self.target_speed = self.yellow_speed
                
            elif new_color == 'green':
                self.print_status(f"\n{emoji} SEMÁFORO VERDE DETECTADO - Velocidad alta {self.green_speed} m/s", 'green')
                self.target_speed = self.green_speed
            
            # Actualizar estado
            self.current_traffic_light = new_color

    def odom_callback(self, msg):
        """Callback para actualizar la posición del robot y verificar llegada a waypoints"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extraer orientación del cuaternión
        q = msg.pose.pose.orientation
        # Convertir cuaternión a ángulo (yaw)
        yaw = self.quaternion_to_yaw(q)
        self.current_theta = yaw

        # Solo verificamos waypoints si no estamos detenidos en rojo o en una esquina
        if self.current_traffic_light != 'red' and not self.at_corner:
            dx = self.current_x - self.goal[0]
            dy = self.current_y - self.goal[1]
            dist = math.hypot(dx, dy)

            if dist <= self.goal_tolerance:
                # Verificar si llegamos a una esquina
                if self.current_index in self.corners:
                    corner_num = self.corners.index(self.current_index) + 1
                    self.print_status(f"🔄 ESQUINA {corner_num} ALCANZADA - Iniciando secuencia de giro", 'magenta')
                    self.at_corner = True
                    self.corner_start_time = self.get_clock().now().nanoseconds * 1e-9
                    self.in_corner_sequence = True
                    # Determinar ángulo objetivo para esta esquina
                    next_segment = min(self.current_segment + 1, len(self.segments) - 1)
                    self.corner_alignment_target = self.get_segment_target_angle(next_segment)
                    self.current_segment = next_segment
                else:
                    # Waypoint normal alcanzado
                    self.get_logger().info(f"✅ Waypoint alcanzado: ({self.goal[0]:.2f}, {self.goal[1]:.2f})")
                    self.advance_to_next_waypoint()

    def advance_to_next_waypoint(self):
        """Avanza al siguiente waypoint de la trayectoria"""
        self.current_index += 1
        if self.current_index < len(self.waypoints):
            self.goal = self.waypoints[self.current_index]
            self.publish_goal()
        else:
            self.get_logger().info("🏁 CUADRADO COMPLETADO: Todos los waypoints alcanzados.")
            self.stop_robot()

    def is_detection_timed_out(self):
        """Verifica si ha pasado suficiente tiempo sin detecciones"""
        current_time = self.get_clock().now().nanoseconds * 1e-9
        return (current_time - self.last_detection_time) > self.detection_timeout
    
    def update_speed(self):
        """Actualiza la velocidad con aceleración gradual"""
        if self.current_speed < self.target_speed:
            # Acelerar gradualmente
            self.current_speed = min(self.current_speed + self.acceleration, self.target_speed)
        elif self.current_speed > self.target_speed:
            # Desacelerar gradualmente (más rápido que acelerar)
            self.current_speed = max(self.current_speed - self.acceleration * 2, self.target_speed)

    def process_corner_sequence(self):
        """Maneja la secuencia especial para las esquinas"""
        curr_time = self.get_clock().now().nanoseconds * 1e-9
        elapsed = curr_time - self.corner_start_time
        
        # Si hemos estado en la esquina suficiente tiempo, continuamos
        if elapsed >= self.corner_pause:
            self.at_corner = False
            self.in_corner_sequence = False
            self.print_status(f"✅ Giro completado - Continuando trayectoria", 'green')
            self.advance_to_next_waypoint()
            return False
        
        # Estamos en medio de la pausa de la esquina
        # Controlamos la orientación mientras estamos detenidos
        error_ang = self.normalize_angle(self.corner_alignment_target - self.current_theta)
        
        # Creamos un comando que solo gira, sin avanzar
        cmd = Twist()
        cmd.linear.x = 0.0  # Detenido
        cmd.angular.z = max(min(self.kp_head * 1.5 * error_ang, self.max_angular), -self.max_angular)
        
        # Publicar comando
        self.cmd_vel_publisher.publish(cmd)
        
        # Información de diagnóstico
        if int(curr_time * 5) % 5 == 0:  # Cada 0.2 segundos aproximadamente
            c = self.colors
            print(f"{c['magenta']}🔄 Ajustando orientación en esquina: {self.current_theta:.2f} → {self.corner_alignment_target:.2f} | Error: {error_ang:.2f}{c['reset']}")
            print(f"{c['magenta']}⏱️ Tiempo en esquina: {elapsed:.1f}/{self.corner_pause}s{c['reset']}")
        
        return True

    def control_loop(self):
        """Bucle de control principal: ajusta velocidad según estado de semáforo"""
        # Obtener tiempo actual para cálculos PID
        curr_time = self.get_clock().now().nanoseconds * 1e-9
        dt = max(curr_time - self.prev_sample_time, 1e-6)
        self.prev_sample_time = curr_time
        
        # Si estamos en una secuencia de esquina, manejamos eso primero
        if self.at_corner:
            if self.process_corner_sequence():
                return
        
        # Si ha pasado mucho tiempo sin detectar un semáforo, usamos el último estado conocido
        if self.is_detection_timed_out() and self.last_state != 'none':
            c = self.colors
            emoji = '🔴' if self.last_state == 'red' else '🟡' if self.last_state == 'yellow' else '🟢'
            
            # Solo mostramos este mensaje ocasionalmente para no saturar la terminal
            if int(curr_time) % 5 == 0:  # Cada 5 segundos aproximadamente
                self.print_status(f"⏱️ Tiempo sin detección excedido - Usando último estado conocido: {emoji} {self.last_state.upper()}", 'yellow')
                
            # Usamos el último estado como estado actual para mantener comportamiento
            self.current_traffic_light = self.last_state
        
        # Si nunca hemos detectado un semáforo, usamos la velocidad inicial
        if self.current_traffic_light == 'none':
            self.target_speed = self.initial_speed
            
        # Si estamos en rojo, nos detenemos
        if self.current_traffic_light == 'red':
            self.stop_robot()
            return
        
        # Si no tenemos más waypoints, terminamos
        if self.current_index >= len(self.waypoints):
            self.stop_robot()
            return
        
        # Actualizar la velocidad actual con aceleración gradual
        self.update_speed()

        # Cálculo de errores para PID
        error_x = self.goal[0] - self.current_x
        error_y = self.goal[1] - self.current_y
        error_pos = math.sqrt(error_x**2 + error_y**2)
        
        # Determinar ángulo objetivo basado en el segmento actual
        if self.current_index in self.corners:
            # Si estamos en una esquina, usamos el ángulo del siguiente segmento
            next_segment = (self.current_segment + 1) % len(self.segments)
            target_ang = self.get_segment_target_angle(next_segment)
        else:
            # En un segmento recto, usamos el ángulo del segmento actual
            target_ang = self.get_segment_target_angle(self.current_segment)
            
            # Verificamos si estamos acercándonos a una esquina
            next_corner = None
            for corner in self.corners:
                if corner > self.current_index:
                    next_corner = corner
                    break
            
            # Si estamos a menos de 3 waypoints de una esquina, reducimos velocidad
            if next_corner is not None and next_corner - self.current_index <= 3:
                self.target_speed = min(self.target_speed, self.corner_speed)
        
        # Error de orientación basado en el ángulo objetivo
        error_ang = self.normalize_angle(target_ang - self.current_theta)
        
        # Derivadas e integrales para PID
        error_d_pos = (error_pos - self.prev_error_pos) / dt
        error_d_ang = (error_ang - self.prev_error_ang) / dt
        self.integral_pos += error_pos * dt
        self.integral_ang += error_ang * dt
        
        # Limitar términos integrales para evitar wind-up
        self.integral_pos = max(min(self.integral_pos, 0.5), -0.5)
        self.integral_ang = max(min(self.integral_ang, 0.5), -0.5)
        
        # Cálculo de señales de control
        v_control = self.kp_pos * error_pos + self.kd_pos * error_d_pos + self.ki_pos * self.integral_pos
        w_control = self.kp_head * error_ang + self.kd_head * error_d_ang + self.ki_head * self.integral_ang
        
        # Inicializar comando
        cmd = Twist()
        
        # Control de velocidad basado en la posición en la trayectoria
        # Determinamos si estamos en un segmento recto o cerca de una esquina
        is_near_corner = False
        for corner in self.corners:
            if abs(self.current_index - corner) <= 3:  # 3 waypoints antes o después de una esquina
                is_near_corner = True
                break
        
        if is_near_corner:
            # Cerca de esquina: reducimos velocidad y aumentamos precisión angular
            cmd.linear.x = min(v_control, self.corner_speed)
            cmd.angular.z = max(min(w_control * 1.5, self.max_angular), -self.max_angular)
        else:
            # Segmento recto: velocidad normal según semáforo
            cmd.linear.x = min(v_control, self.current_speed)
            cmd.angular.z = max(min(w_control, self.max_angular), -self.max_angular)
        
        # Si el error angular es grande, priorizamos girar antes que avanzar
        if abs(error_ang) > 0.3:  # Más de ~17 grados
            # Reducimos velocidad lineal para girar mejor
            cmd.linear.x *= max(0.3, 1.0 - abs(error_ang) / math.pi)
        
        # Publicar comando
        self.cmd_vel_publisher.publish(cmd)
        
        # Actualizar errores para la próxima iteración
        self.prev_error_pos = error_pos
        self.prev_error_ang = error_ang
        
        # Mostrar información si hay movimiento o cambio de estado (cada segundo aprox.)
        if (abs(cmd.linear.x) > 0.01 or abs(cmd.angular.z) > 0.01) and int(curr_time * 2) % 2 == 0:
            c = self.colors
            emoji = '🔴' if self.current_traffic_light == 'red' else '🟡' if self.current_traffic_light == 'yellow' else '🟢' if self.current_traffic_light == 'green' else '🚙'
            state = self.current_traffic_light.upper() if self.current_traffic_light != 'none' else 'INICIAL'
            
            # Información sobre el segmento actual
            segment_info = f"Segmento {self.current_segment+1}/4"
            if is_near_corner:
                segment_info += " (CERCA DE ESQUINA)"
            
            print(f"{c['bold']}{'─'*50}{c['reset']}")
            print(f"  {emoji} Estado: {state}  |  Waypoint: {self.current_index+1}/{len(self.waypoints)}")
            print(f"  🧩 {segment_info}")
            print(f"  🚗 Velocidad: v={cmd.linear.x:.2f} m/s, w={cmd.angular.z:.2f} rad/s")
            print(f"  🏎️ Vel. objetivo: {self.target_speed:.2f} m/s, Vel. actual: {self.current_speed:.2f} m/s")
            print(f"  📍 Posición: ({self.current_x:.2f}, {self.current_y:.2f}), θ={self.current_theta:.2f}")
            print(f"  🎯 Destino: ({self.goal[0]:.2f}, {self.goal[1]:.2f}) - Dist: {error_pos:.2f}m")
            print(f"{c['bold']}{'─'*50}{c['reset']}")

    def stop_robot(self):
        """Detiene el robot inmediatamente"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        self.current_speed = 0.0  # Reinicia la velocidad actual

    def normalize_angle(self, angle):
        """Normaliza un ángulo al rango [-pi, pi]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def quaternion_to_yaw(self, q):
        """Convierte un cuaternión a ángulo de yaw (radianes)"""
        # Convierte q.z y q.w a ángulo de yaw
        # Para PuzzleBot, típicamente q.z tiene el seno del ángulo/2 y q.w tiene el coseno
        if hasattr(q, 'z') and hasattr(q, 'w'):
            return 2 * math.atan2(q.z, q.w)
        return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = PathGeneratorSquare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_status("\n\n🛑 Nodo detenido por el usuario. ¡Hasta pronto! 👋\n", 'red')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()