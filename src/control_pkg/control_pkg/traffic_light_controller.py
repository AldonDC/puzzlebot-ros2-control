#!/usr/bin/env python3
'''
Traffic Light Controller for PuzzleBot

Este nodo implementa el comportamiento del robot según las señales de tráfico:
- 🔴 Rojo: Deténgase hasta que vea la luz verde
- 🟡 Amarillo: Conduzca a velocidad reducida (0.2 m/s)
- 🟢 Verde: Continúe su ruta a velocidad normal (0.3 m/s)

Subscribes:
- /traffic_light_color (std_msgs/String): Color detectado del semáforo
- /odom (nav_msgs/Odometry): Posición actual del robot

Publishes:
- /cmd_vel (geometry_msgs/Twist): Comandos de velocidad para el robot
'''

import rclpy
import math
import time
from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

class TrafficLightController(Node):
    def __init__(self):
        super().__init__('traffic_light_controller')
        
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
        
        # Suscripciones
        self.create_subscription(
            String, 
            '/traffic_light_color', 
            self.traffic_light_callback, 
            10
        )
        
        self.create_subscription(
            Odometry, 
            'odom', 
            self.odometry_callback, 
            qos.qos_profile_sensor_data
        )
        
        # Publicadores
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )


        
        
        # Parámetros del controlador
        self.declare_parameter('max_speed', 0.3)        # Velocidad máxima (m/s) - Verde
        self.declare_parameter('reduced_speed', 0.2)    # Velocidad reducida (m/s) - Amarillo - MODIFICADA A 0.2
        self.declare_parameter('start_speed', 0.03)     # Velocidad inicial (m/s) - Sin semáforo
        self.declare_parameter('max_angular', 0.5)      # Velocidad angular máxima (rad/s)
        self.declare_parameter('target_x', 1.0)         # Posición objetivo X (m)
        self.declare_parameter('target_y', 0.0)         # Posición objetivo Y (m)
        self.declare_parameter('auto_move', True)       # Movimiento automático
        self.declare_parameter('acceleration', 0.005)   # Aceleración gradual
        
        # Parámetros PID
        self.declare_parameter('KpPos', 0.5)
        self.declare_parameter('KiPos', 0.05)
        self.declare_parameter('KdPos', 0.01)
        self.declare_parameter('KpHead', 0.8)
        self.declare_parameter('KiHead', 0.0)
        self.declare_parameter('KdHead', 0.0)
        
        # Cargar parámetros
        self.max_speed = self.get_parameter('max_speed').value
        self.reduced_speed = self.get_parameter('reduced_speed').value
        self.start_speed = self.get_parameter('start_speed').value
        self.max_angular = self.get_parameter('max_angular').value
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.auto_move = self.get_parameter('auto_move').value
        self.acceleration = self.get_parameter('acceleration').value
        
        # Constantes PID
        self.kp_pos = self.get_parameter('KpPos').value
        self.ki_pos = self.get_parameter('KiPos').value
        self.kd_pos = self.get_parameter('KdPos').value
        self.kp_head = self.get_parameter('KpHead').value
        self.ki_head = self.get_parameter('KiHead').value
        self.kd_head = self.get_parameter('KdHead').value
        
        # Variables de estado
        self.current_traffic_light = 'none'  # Inicialmente no hay semáforo detectado
        self.waiting_for_green = False       # Flag para indicar si estamos esperando la luz verde
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.last_detection_time = self.get_clock().now().nanoseconds * 1e-9
        self.detection_timeout = 2.0  # Segundos sin detección para considerar que no hay semáforo
        
        # Variables para aceleración gradual - NUEVAS
        self.current_speed = 0.0
        self.target_speed = self.start_speed
        self.start_time = time.time()
        
        # Variables para el controlador PID
        self.prev_error_pos = 0.0
        self.integral_pos = 0.0
        self.prev_error_ang = 0.0
        self.integral_ang = 0.0
        self.prev_sample_time = self.get_clock().now().nanoseconds * 1e-9
        
        # Timer para el controlador
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # Imprimir encabezado
        self.print_header()
    
    def print_header(self):
        c = self.colors
        print(f"\n{c['bold']}{'='*70}{c['reset']}")
        print(f"{c['cyan']}🚦 CONTROLADOR DE SEMÁFORO PARA PUZZLEBOT 🚦{c['reset']}")
        print(f"{c['bold']}{'='*70}{c['reset']}")
        print(f"{c['blue']}✅ Inicializado y esperando detección de semáforos{c['reset']}")
        print(f"{c['blue']}📍 Destino configurado: ({self.target_x}, {self.target_y}){c['reset']}")
        print(f"{c['green']}🚗 Auto-movimiento: {'ACTIVADO' if self.auto_move else 'DESACTIVADO'}{c['reset']}")
        print(f"{c['yellow']}🐢 Velocidad inicial: {self.start_speed} m/s, Máxima: {self.max_speed} m/s{c['reset']}")
        print(f"{c['yellow']}🚥 Velocidades configuradas: Verde={self.max_speed} m/s, Amarillo={self.reduced_speed} m/s{c['reset']}")
        print(f"{c['bold']}{'='*70}{c['reset']}\n")
    
    def print_status(self, message, color='blue'):
        c = self.colors
        if color in c:
            print(f"{c[color]}{message}{c['reset']}")
        else:
            print(f"{c['blue']}{message}{c['reset']}")
    
    def traffic_light_callback(self, msg):
        """Callback para manejar detecciones de semáforo"""
        new_color = msg.data
        
        # Actualizar tiempo de última detección
        self.last_detection_time = self.get_clock().now().nanoseconds * 1e-9
        
        # Si el color ha cambiado, actualizamos el estado
        if new_color != self.current_traffic_light:
            # Obtener emoji para el color
            emoji = '🔴' if new_color == 'red' else '🟡' if new_color == 'yellow' else '🟢' if new_color == 'green' else '⚪'
            
            # Transiciones importantes
            if new_color == 'red' and self.current_traffic_light != 'red':
                self.print_status(f"\n{emoji} SEMÁFORO ROJO DETECTADO - Deteniendo robot", 'red')
                self.stop_robot()
                self.waiting_for_green = True
                # Resetear velocidad para cuando vuelva a arrancar
                self.current_speed = 0.0
                
            elif new_color == 'yellow' and self.current_traffic_light != 'yellow':
                self.print_status(f"\n{emoji} SEMÁFORO AMARILLO DETECTADO - Avanzando a velocidad media {self.reduced_speed} m/s", 'yellow')
                # Ajustar velocidad objetivo a la velocidad reducida (0.2 m/s)
                self.target_speed = self.reduced_speed
                # Con amarillo siempre avanzamos, sin importar si venimos de rojo
                self.waiting_for_green = False
                
            elif new_color == 'green':
                if self.waiting_for_green:
                    self.print_status(f"\n{emoji} SEMÁFORO VERDE DETECTADO - Continuando ruta a {self.max_speed} m/s", 'green')
                    self.waiting_for_green = False
                    # Reiniciar con velocidad alta para verde
                    self.current_speed = 0.0
                    self.target_speed = self.max_speed
                else:
                    self.print_status(f"\n{emoji} SEMÁFORO VERDE DETECTADO - Acelerando a {self.max_speed} m/s", 'green')
                    self.target_speed = self.max_speed
            
            # Actualizar estado
            self.current_traffic_light = new_color
    
    def odometry_callback(self, msg):
        """Callback para actualizar la posición del robot"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extraer orientación del cuaternión
        q = msg.pose.pose.orientation
        # Asumiendo que ya está en radianes en q.z
        self.current_theta = q.z if hasattr(q, 'z') else 0.0  
    
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
            # Desacelerar gradualmente
            self.current_speed = max(self.current_speed - self.acceleration * 2, self.target_speed)
    
    def control_loop(self):
        """Bucle de control principal"""
        # Obtener tiempo actual para cálculos PID
        curr_time = self.get_clock().now().nanoseconds * 1e-9
        dt = max(curr_time - self.prev_sample_time, 1e-6)
        self.prev_sample_time = curr_time
        
        # Si ha pasado mucho tiempo sin detectar un semáforo, reiniciamos el estado
        if self.is_detection_timed_out() and self.current_traffic_light != 'none':
            self.print_status("⏱️ Tiempo sin detección excedido - Reiniciando estado", 'yellow')
            self.current_traffic_light = 'none'
            # Si estábamos esperando verde, ya no esperamos
            if self.waiting_for_green:
                self.waiting_for_green = False
                # Volvemos a la velocidad inicial cuando no hay semáforo
                self.target_speed = self.start_speed
        
        # Si estamos esperando luz verde después de rojo, mantenemos el robot parado
        # Solo si el semáforo actual es rojo, no si es amarillo
        if self.waiting_for_green and self.current_traffic_light == 'red':
            self.stop_robot()
            return
        
        # Actualizar la velocidad actual con aceleración gradual
        self.update_speed()
        
        # Cálculo de errores para PID
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        error_pos = math.sqrt(error_x**2 + error_y**2)
        target_ang = math.atan2(error_y, error_x)
        error_ang = self.normalize_angle(target_ang - self.current_theta)
        
        # Derivadas e integrales para PID
        error_d_pos = (error_pos - self.prev_error_pos) / dt
        error_d_ang = (error_ang - self.prev_error_ang) / dt
        self.integral_pos += error_pos * dt
        self.integral_ang += error_ang * dt
        
        # Limitar términos integrales para evitar wind-up
        self.integral_pos = max(min(self.integral_pos, 1.0), -1.0)  # Más restrictivo
        self.integral_ang = max(min(self.integral_ang, 1.0), -1.0)  # Más restrictivo
        
        # Cálculo de señales de control
        v_control = self.kp_pos * error_pos + self.kd_pos * error_d_pos + self.ki_pos * self.integral_pos
        w_control = self.kp_head * error_ang + self.kd_head * error_d_ang + self.ki_head * self.integral_ang
        
        # Inicializar comando
        cmd = Twist()
        
        # Si llegamos al objetivo, detenerse
        if error_pos < 0.05:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.integral_pos = 0.0
            self.integral_ang = 0.0
            self.current_speed = 0.0
            self.print_status("🎯 OBJETIVO ALCANZADO", 'green')
            
        # Comportamiento según semáforo
        elif self.current_traffic_light == 'red':
            # Rojo: Detenerse
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.waiting_for_green = True
            self.current_speed = 0.0
            
        elif self.current_traffic_light == 'yellow':
            # Amarillo: Avanzar a velocidad reducida (0.2 m/s)
            # El amarillo siempre avanza, aunque vengamos de rojo
            cmd.linear.x = min(v_control, self.current_speed)
            cmd.angular.z = max(min(w_control, self.max_angular), -self.max_angular)
            
        elif self.current_traffic_light == 'green':
            # Verde: Velocidad alta (0.3 m/s)
            cmd.linear.x = min(v_control, self.current_speed)
            cmd.angular.z = max(min(w_control, self.max_angular), -self.max_angular)
            
        # Si no hay detección y auto_move está activo, moverse por defecto a velocidad inicial
        elif self.current_traffic_light == 'none' and self.auto_move and not self.waiting_for_green:
            # Mover automáticamente a velocidad actual (debería ser self.start_speed = 0.03 m/s)
            cmd.linear.x = self.current_speed
            # Ajustar ángulo si es necesario
            if abs(error_ang) > 0.2:
                cmd.angular.z = max(min(w_control, self.max_angular), -self.max_angular)
            else:
                cmd.angular.z = 0.0
            
        else:
            # En cualquier otro caso, mantener el robot parado
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        # Publicar comando
        self.cmd_vel_publisher.publish(cmd)
        
        # Actualizar errores para la próxima iteración
        self.prev_error_pos = error_pos
        self.prev_error_ang = error_ang
        
        # Mostrar información si hay movimiento o cambio de estado
        if abs(cmd.linear.x) > 0.01 or abs(cmd.angular.z) > 0.01:
            c = self.colors
            emoji = '🔴' if self.current_traffic_light == 'red' else '🟡' if self.current_traffic_light == 'yellow' else '🟢' if self.current_traffic_light == 'green' else '⚪'
            state = self.current_traffic_light.upper() if self.current_traffic_light != 'none' else 'SIN SEMÁFORO (AUTO-MOVIMIENTO)'
            
            print(f"{c['bold']}{'─'*50}{c['reset']}")
            print(f"  {emoji} Estado: {state}")
            print(f"  🚗 Velocidad: v={cmd.linear.x:.2f} m/s, w={cmd.angular.z:.2f} rad/s")
            print(f"  🏎️ Vel. objetivo: {self.target_speed:.2f} m/s, Vel. actual: {self.current_speed:.2f} m/s")
            print(f"  📍 Posición: ({self.current_x:.2f}, {self.current_y:.2f}), θ={self.current_theta:.2f}")
            print(f"  🎯 Destino: ({self.target_x:.2f}, {self.target_y:.2f}) - Distancia: {error_pos:.2f}m")
            print(f"{c['bold']}{'─'*50}{c['reset']}")
    
    def stop_robot(self):
        """Detiene el robot inmediatamente"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
    
    def normalize_angle(self, angle):
        """Normaliza un ángulo al rango [-pi, pi]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    controller = TrafficLightController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.print_status("\n\n🛑 Controlador detenido por el usuario. ¡Hasta pronto! 👋\n", 'red')
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()