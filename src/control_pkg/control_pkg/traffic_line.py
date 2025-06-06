#!/usr/bin/env python3
# traffic_line.py - Controlador integrado con dos cámaras: USB (semáforos) + CSI (línea)
# PRIORIDAD: SIGUE LÍNEAS - Semáforo solo limita velocidad máxima
# Ubicación: control_pkg/control_pkg/traffic_line.py

#CODE DE LINE FOLLOWER Y SEMAFORO CORRECTO


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from simple_pid import PID
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import sys


class TrafficLineController(Node):
    def __init__(self):
        super().__init__('traffic_line_controller')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/debug_image', 10)
        self.traffic_pub = self.create_publisher(String, '/traffic_light_color', 10)
        
        # QoS Profile para compatibilidad
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # =============================================================================
        # SUSCRIPTORES A AMBAS CÁMARAS
        # =============================================================================
        
        # Cámara CSI para seguimiento de línea
        self.csi_image_sub = self.create_subscription(
            Image, 
            '/puzzlebot/camera/image_raw',  # CSI para línea negra
            self.csi_image_callback, 
            qos_profile
        )
        
        # Cámara USB para detección de semáforos
        self.usb_image_sub = self.create_subscription(
            Image, 
            '/puzzlebot/usb_camera/image_raw',  # USB para semáforos
            self.usb_image_callback, 
            qos_profile
        )
        
        self.bridge = CvBridge()

        # =============================================================================
        # CONFIGURACIÓN DEL CONTROL DE LÍNEA (PRIORIDAD PRINCIPAL)
        # =============================================================================
        max_yaw = math.radians(45)  # Reducido para movimiento más suave
        self.max_thr = 0.15  # Velocidad máxima reducida para movimiento lento
        self.yaw_pid = PID(Kp=0.4, Ki=0, Kd=0.08, setpoint=0.0, output_limits=(-max_yaw, max_yaw))

        # Factores de ponderación para jerarquía de decisión
        self.center_weight = 0.7
        self.angle_weight = 0.3
        
        # Parámetros para búsqueda cuando se pierde la línea
        self.line_lost_count = 0
        self.max_line_lost_count = 5
        self.backup_speed = -0.05  # Velocidad de reversa más lenta
        self.last_yaw = 0.0
        self.in_search_mode = False
        self.search_yaw_options = [-0.1, 0.0, 0.1]
        self.search_yaw_index = 1
        self.search_time_counter = 0
        self.search_interval = 20

        # =============================================================================
        # VARIABLES PARA DETECCIÓN CONTINUA DE LÍNEA (PRIORIDAD)
        # =============================================================================
        
        # Variables para almacenar SIEMPRE los datos más recientes de línea
        self.current_line_throttle = 0.0
        self.current_line_yaw = 0.0
        self.current_line_found = False
        
        # =============================================================================
        # CONFIGURACIÓN DEL DETECTOR DE SEMÁFOROS (SOLO LIMITA VELOCIDAD)
        # =============================================================================
        
        # Estados del sistema - SOLO BASADOS EN LÍNEA
        self.ROBOT_STATES = {
            'FOLLOWING': 'FOLLOWING',      # Siguiendo línea normalmente
            'SEARCHING': 'SEARCHING'       # Buscando línea
        }
        
        self.current_state = self.ROBOT_STATES['FOLLOWING']
        self.current_traffic_color = None
        self.last_traffic_color = None
        self.traffic_confirmation_count = 0
        self.min_traffic_confirmations = 3
        
        # =============================================================================
        # LÍMITES DE VELOCIDAD SEGÚN SEMÁFORO (SOLO AFECTA VELOCIDAD MÁXIMA)
        # =============================================================================
        self.TRAFFIC_SPEED_LIMITS = {
            'red': 0.0,      # Rojo: STOP completely
            'yellow': 0.075,  # Amarillo: Half speed (0.15 m/s)
            'green': 0.15,    # Verde: Fast speed (0.3 m/s)
            None: 0.12        # Sin semáforo: Normal speed (0.2 m/s)
        }
        
        # Rangos de color HSV para detección de semáforos
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([40, 255, 255])
        self.lower_green = np.array([50, 80, 80])
        self.upper_green = np.array([90, 255, 255])
        
        # =============================================================================
        # VARIABLES DE CONTROL DE FRAMES
        # =============================================================================
        
        # Variables para los últimos frames recibidos
        self.latest_csi_frame = None
        self.latest_usb_frame = None
        self.last_line_detection = None
        self.last_traffic_detection = None
        
        # Control de timing para procesamiento
        self.last_control_update = time.time()
        self.control_interval = 0.05  # Actualizar control cada 50ms
        
        # Estadísticas y terminal
        self.start_time = time.time()
        self.frame_count_csi = 0
        self.frame_count_usb = 0
        self.last_status_print = 0
        self.status_interval = 1.0
        self.traffic_detections = {'red': 0, 'yellow': 0, 'green': 0}
        
        # Colores para terminal
        self.colors = {
            'red': '\033[1;31m', 'yellow': '\033[1;33m', 'green': '\033[1;32m',
            'reset': '\033[0m', 'bold': '\033[1m', 'blue': '\033[1;34m',
            'magenta': '\033[1;35m', 'cyan': '\033[1;36m', 'white': '\033[1;37m'
        }
        
        self.emojis = {
            'red': '🔴', 'yellow': '🟡', 'green': '🟢', 'none': '⚪',
            'usb': '📷', 'chip': '🎥', 'line': '📐', 'traffic': '🚦'
        }
        
        self._print_header()

    def _print_header(self):
        """Imprime encabezado del sistema integrado con dos cámaras"""
        c = self.colors
        e = self.emojis
        terminal_width = 90
        print("\n" + "=" * terminal_width)
        print(f"{c['cyan']} " + " " * 15 + "🚦 PRIORITY LINE FOLLOWER + TRAFFIC SPEED LIMITER 🚦" + f"{c['reset']}")
        print(f"{c['bold']}" + " " * 15 + f"{e['usb']} USB Camera (Traffic) + {e['chip']} CSI Camera (Line) " + f"{c['reset']}")
        print(f"{c['bold']}" + " " * 20 + "📐 LÍNEA = PRIORIDAD | 🚦 SEMÁFORO = LÍMITE VELOCIDAD" + f"{c['reset']}")
        print("=" * terminal_width)
        print(f"{c['bold']}Lanzado:{c['reset']} {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"{c['bold']}PID Config:{c['reset']} Kp={self.yaw_pid.Kp}, Ki={self.yaw_pid.Ki}, Kd={self.yaw_pid.Kd}")
        print(f"{c['bold']}Velocidad máxima:{c['reset']} {self.max_thr} m/s")
        print(f"{c['bold']}Cámaras:{c['reset']}")
        print(f"  {e['chip']} CSI: {c['white']}/puzzlebot/camera/image_raw{c['reset']} (Línea negra - PRIORIDAD)")
        print(f"  {e['usb']} USB: {c['white']}/puzzlebot/usb_camera/image_raw{c['reset']} (Semáforos - Límite velocidad)")
        print(f"{c['bold']}Límites:{c['reset']} {c['red']}Rojo=STOP{c['reset']} | {c['yellow']}Amarillo=0.15m/s{c['reset']} | {c['green']}Verde=0.3m/s{c['reset']}")
        print("-" * terminal_width)
        print(f"{c['bold']}{'TIEMPO':<8} {'ESTADO':<15} {'SEMÁFORO':<12} {'VEL_LÍNEA':<10} {'VEL_FINAL':<10} {'YAW':<10} {'FPS_CSI':<8} {'FPS_USB':<8} {'INFO':<15}{c['reset']}")
        print("-" * terminal_width)
        sys.stdout.flush()

    def _print_status(self, robot_state, traffic_color, line_throttle, final_throttle, yaw, info=""):
        """Imprime estado del sistema con información de ambas cámaras"""
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if current_time - self.last_status_print >= self.status_interval:
            c = self.colors
            e = self.emojis
            
            # Calcular FPS de ambas cámaras
            fps_csi = self.frame_count_csi / elapsed if elapsed > 0 else 0
            fps_usb = self.frame_count_usb / elapsed if elapsed > 0 else 0
            
            # Formatear estado del robot
            if robot_state == "FOLLOWING":
                state_fmt = f"{c['green']}{robot_state}{c['reset']}"
            else:
                state_fmt = f"{c['yellow']}{robot_state}{c['reset']}"
            
            # Formatear color del semáforo
            if traffic_color:
                emoji = e.get(traffic_color, '⚪')
                traffic_fmt = f"{c[traffic_color]}{emoji}{traffic_color.upper()}{c['reset']}"
            else:
                traffic_fmt = f"{c['reset']}⚪NONE{c['reset']}"
            
            print(f"\033[K{elapsed:<8.1f} {state_fmt:<25} {traffic_fmt:<20} {line_throttle:<10.3f} {final_throttle:<10.3f} {yaw:<10.2f} {fps_csi:<8.1f} {fps_usb:<8.1f} {info:<15}")
            sys.stdout.flush()
            self.last_status_print = current_time

    def csi_image_callback(self, msg):
        """Callback para cámara CSI - PROCESAMIENTO PRIORITARIO DE LÍNEA"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_csi_frame = frame.copy()
            self.frame_count_csi += 1
            
            # =============================================================================
            # PROCESAR SEGUIMIENTO DE LÍNEA SIEMPRE - PRIORIDAD ABSOLUTA
            # =============================================================================
            throttle, yaw, line_found = self.follow_line(frame)
            
            # ACTUALIZAR SIEMPRE los datos de línea más recientes
            self.current_line_throttle = throttle
            self.current_line_yaw = yaw
            self.current_line_found = line_found
            
            self.last_line_detection = (throttle, yaw, line_found, time.time())
            
            # Agregar información de la cámara al frame
            cv2.putText(frame, f"{self.emojis['chip']} CSI Camera - LINE PRIORITY", (10, frame.shape[0] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"Line Found: {line_found} | Throttle: {throttle:.3f}", (10, frame.shape[0] - 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # Actualizar control si es momento
            self._update_robot_control()
            
        except Exception as e:
            self.get_logger().error(f"Error en cámara CSI: {e}")

    def usb_image_callback(self, msg):
        """Callback para cámara USB - Detección de semáforos (solo límite de velocidad)"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_usb_frame = frame.copy()
            self.frame_count_usb += 1
            
            # Procesar detección de semáforos
            traffic_color, best_circle, all_circles = self.detect_traffic_light(frame)
            self.last_traffic_detection = (traffic_color, best_circle, time.time())
            
            # Procesar confirmación del color del semáforo
            if traffic_color:
                if traffic_color == self.last_traffic_color:
                    self.traffic_confirmation_count += 1
                else:
                    self.traffic_confirmation_count = 1
                    self.last_traffic_color = traffic_color
                
                # Confirmar color si tenemos suficientes detecciones
                if self.traffic_confirmation_count >= self.min_traffic_confirmations:
                    self.current_traffic_color = traffic_color
                    self.traffic_detections[traffic_color] += 1
                    
                    # Publicar color del semáforo
                    traffic_msg = String()
                    traffic_msg.data = traffic_color
                    self.traffic_pub.publish(traffic_msg)
                    
                    # Dibujar círculo detectado
                    if best_circle:
                        center, radius = best_circle
                        color_bgr = {'red': (0, 0, 255), 'yellow': (0, 255, 255), 'green': (0, 255, 0)}
                        cv2.circle(frame, center, radius, color_bgr.get(traffic_color, (255, 255, 255)), 3)
                        cv2.putText(frame, f"LIMIT: {self.TRAFFIC_SPEED_LIMITS[traffic_color]:.2f} m/s", 
                                    (center[0]-60, center[1]-radius-10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr.get(traffic_color, (255, 255, 255)), 2)
            
            # Agregar información de la cámara al frame
            cv2.putText(frame, f"{self.emojis['usb']} USB Camera - SPEED LIMITER", (10, frame.shape[0] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            
            # Mostrar límite de velocidad actual
            current_limit = self.TRAFFIC_SPEED_LIMITS.get(self.current_traffic_color, self.TRAFFIC_SPEED_LIMITS[None])
            cv2.putText(frame, f"Speed Limit: {current_limit:.2f} m/s", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                            
        except Exception as e:
            self.get_logger().error(f"Error en cámara USB: {e}")

    def _update_robot_control(self):
        """Control del robot - LÍNEA CONTROLA TODO, SEMÁFORO SOLO LIMITA VELOCIDAD"""
        current_time = time.time()
        
        # Controlar frecuencia de actualización
        if current_time - self.last_control_update < self.control_interval:
            return
            
        self.last_control_update = current_time
        
        # =============================================================================
        # USAR SIEMPRE LOS DATOS DE LÍNEA COMO BASE (PRIORIDAD ABSOLUTA)
        # =============================================================================
        line_throttle = self.current_line_throttle  # Velocidad calculada por línea
        line_yaw = self.current_line_yaw            # Dirección calculada por línea
        line_found = self.current_line_found        # Estado de detección de línea
        
        # =============================================================================
        # DETERMINAR ESTADO SOLO BASADO EN LÍNEA
        # =============================================================================
        if not line_found:
            self.current_state = self.ROBOT_STATES['SEARCHING']
        else:
            self.current_state = self.ROBOT_STATES['FOLLOWING']
        
        # =============================================================================
        # APLICAR CONTROL NORMAL DE LÍNEA
        # =============================================================================
        final_throttle = 0.0
        final_yaw = line_yaw  # YAW SIEMPRE viene de la línea
        info_text = ""
        
        if self.current_state == self.ROBOT_STATES['FOLLOWING']:
            # =========================================================================
            # SEGUIMIENTO NORMAL DE LÍNEA CON LÍMITE DE VELOCIDAD POR SEMÁFORO
            # =========================================================================
            
            # Obtener límite de velocidad según semáforo
            speed_limit = self.TRAFFIC_SPEED_LIMITS.get(self.current_traffic_color, 
                                                       self.TRAFFIC_SPEED_LIMITS[None])
            
            # APLICAR LÍMITE: La velocidad final es el mínimo entre línea y semáforo
            final_throttle = min(line_throttle, speed_limit)
            final_yaw = line_yaw  # YAW siempre viene de la línea
            
            # Información según semáforo
            if self.current_traffic_color == 'red':
                info_text = f"{self.emojis['red']} RED STOP"
            elif self.current_traffic_color == 'yellow':
                info_text = f"{self.emojis['yellow']} YELLOW LIMIT"
            elif self.current_traffic_color == 'green':
                info_text = f"{self.emojis['green']} GREEN OK"
            else:
                info_text = f"{self.emojis['line']} NO TRAFFIC"
            
            # Resetear contadores de línea perdida
            self.line_lost_count = 0
            self.in_search_mode = False
                        
        elif self.current_state == self.ROBOT_STATES['SEARCHING']:
            # =========================================================================
            # BÚSQUEDA DE LÍNEA - NO AFECTADA POR SEMÁFOROS
            # =========================================================================
            self.line_lost_count += 1
            
            if self.line_lost_count >= self.max_line_lost_count:
                self.in_search_mode = True
                
                # Retroceder con patrón de búsqueda
                final_throttle = self.backup_speed
                
                self.search_time_counter += 1
                if self.search_time_counter >= self.search_interval:
                    self.search_yaw_index = (self.search_yaw_index + 1) % len(self.search_yaw_options)
                    self.search_time_counter = 0
                
                final_yaw = self.search_yaw_options[self.search_yaw_index]
                info_text = f"🔍 SEARCHING ({self.search_yaw_index+1}/{len(self.search_yaw_options)})"
            else:
                info_text = f"⚠️ LOST {self.line_lost_count}/{self.max_line_lost_count}"

        # =============================================================================
        # PUBLICAR COMANDOS DE MOVIMIENTO
        # =============================================================================
        twist = Twist()
        twist.linear.x = float(final_throttle)
        twist.angular.z = float(final_yaw)
        self.cmd_vel_pub.publish(twist)

        # =============================================================================
        # CREAR IMAGEN DEBUG COMBINADA
        # =============================================================================
        self._create_debug_image()
        
        # Imprimir estado en terminal mostrando velocidad de línea vs final
        self._print_status(self.current_state, self.current_traffic_color, 
                          line_throttle, final_throttle, final_yaw, info_text)

    def _create_debug_image(self):
        """Crea una imagen debug combinando ambas cámaras"""
        if self.latest_csi_frame is None and self.latest_usb_frame is None:
            return
            
        debug_frame = None
        
        if self.latest_csi_frame is not None and self.latest_usb_frame is not None:
            # Redimensionar ambas imágenes al mismo tamaño
            height = 320
            width = 400
            csi_resized = cv2.resize(self.latest_csi_frame, (width, height))
            usb_resized = cv2.resize(self.latest_usb_frame, (width, height))
            
            # Combinar horizontalmente
            debug_frame = np.hstack((csi_resized, usb_resized))
            
            # Agregar información del estado
            cv2.putText(debug_frame, f"State: {self.current_state}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(debug_frame, f"Line Found: {self.current_line_found}", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if self.current_line_found else (0, 0, 255), 2)
            
            # Mostrar velocidades
            line_speed = self.current_line_throttle
            speed_limit = self.TRAFFIC_SPEED_LIMITS.get(self.current_traffic_color, self.TRAFFIC_SPEED_LIMITS[None])
            final_speed = min(line_speed, speed_limit)
            
            cv2.putText(debug_frame, f"Line Speed: {line_speed:.3f}", (400, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(debug_frame, f"Speed Limit: {speed_limit:.3f}", (400, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(debug_frame, f"Final Speed: {final_speed:.3f}", (400, 70), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        
        elif self.latest_csi_frame is not None:
            debug_frame = self.latest_csi_frame.copy()
            cv2.putText(debug_frame, "CSI Only - USB Camera Missing", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        
        elif self.latest_usb_frame is not None:
            debug_frame = self.latest_usb_frame.copy()
            cv2.putText(debug_frame, "USB Only - CSI Camera Missing", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        if debug_frame is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
            self.debug_pub.publish(debug_msg)

    def detect_traffic_light(self, frame):
        """Detecta semáforos en la imagen de la cámara USB"""
        # Convertir a HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Crear máscaras para cada color
        mask_red1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask_red2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_yellow = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
        # Operaciones morfológicas
        kernel = np.ones((5, 5), np.uint8)
        for mask in [mask_red, mask_yellow, mask_green]:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Encontrar contornos y círculos
        circles = {'red': [], 'yellow': [], 'green': []}
        masks = {'red': mask_red, 'yellow': mask_yellow, 'green': mask_green}
        
        for color, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 100:  # Filtrar contornos pequeños
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)
                    
                    # Calcular circularidad
                    perimeter = cv2.arcLength(contour, True)
                    circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
                    
                    # Si es suficientemente circular
                    if circularity > 0.6 and radius > 8:
                        circles[color].append((center, radius, area))
        
        # Determinar color dominante
        max_area = 0
        detected_color = None
        best_circle = None
        
        for color in circles:
            for circle in circles[color]:
                center, radius, area = circle
                if area > max_area:
                    max_area = area
                    detected_color = color
                    best_circle = (center, radius)
        
        return detected_color, best_circle, circles

    def follow_line(self, frame):
        """Algoritmo de seguimiento de línea para cámara CSI - PRIORIDAD ABSOLUTA"""
        if frame is None:
            return 0.0, 0.0, False

        frame_height, frame_width = frame.shape[:2]
        frame_center_x = frame_width / 2

        # Threshold y preprocesamiento
        dark_thres = 100
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, dark_thres, 255, cv2.THRESH_BINARY_INV)
        mask[:int(frame_height * 0.7), :] = 0
        mask = cv2.erode(mask, np.ones((3, 3), np.uint8), iterations=3)
        mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=5)

        # Contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        MIN_AREA = 1000
        contours = [c for c in contours if cv2.contourArea(c) > MIN_AREA]

        throttle, yaw = 0.0, 0.0
        line_found = len(contours) > 0
        
        if contours:
            # Función de puntuación para jerarquía de decisión
            def line_score(c):
                pt1, pt2, angle, cx, cy = self.get_contour_line(c)
                
                center_distance = abs(cx - frame_center_x)
                center_score = 1.0 - (center_distance / frame_center_x)
                
                max_angle = 80
                angle_deviation = min(abs(angle), max_angle) / max_angle
                angle_score = 1.0 - angle_deviation
                
                total_score = (self.center_weight * center_score) + (self.angle_weight * angle_score)
                return total_score

            # Seleccionar mejor línea
            scored_contours = [(c, line_score(c)) for c in contours]
            scored_contours.sort(key=lambda x: x[1], reverse=True)
            
            line_contour = scored_contours[0][0]
            best_score = scored_contours[0][1]

            # Visualización
            cv2.drawContours(frame, [line_contour], -1, (0, 255, 0), 2)
            
            # Calcular comandos de movimiento
            _, _, angle, cx, cy = self.get_contour_line(line_contour)
            normalized_x = (cx - frame_center_x) / frame_center_x
            
            # Línea central seleccionada
            cv2.line(frame, (int(cx), 0), (int(cx), frame_height), (255, 0, 0), 2)
            cv2.putText(frame, f"Line Score: {best_score:.2f}", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Calcular dirección y velocidad base
            yaw = self.yaw_pid(normalized_x)
            
            # Velocidad base según alineación - INCREASED FOR BETTER PERFORMANCE
            alignment = 1 - abs(normalized_x)
            align_thres = 0.3
            # Increased max speed for better line following
            max_speed = 0.25  # Increased from 0.12 to 0.25 for faster line following
            throttle = max_speed * ((alignment - align_thres) / (1 - align_thres)) if alignment >= align_thres else 0

        return throttle, yaw, line_found

    def get_contour_line(self, c, fix_vert=True):
        """Calcula línea desde contorno"""
        vx, vy, cx, cy = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
        scale = 100
        pt1 = (int(cx - vx * scale), int(cy - vy * scale))
        pt2 = (int(cx + vx * scale), int(cy + vy * scale))
        angle = math.degrees(math.atan2(vy, vx))

        if fix_vert:
            angle = angle - 90 * np.sign(angle)

        return pt1, pt2, angle, cx, cy


def main(args=None):
    rclpy.init(args=args)
    
    print("\n\033[1;36m" + "=" * 90)
    print(" " * 20 + "🚀 INICIANDO PRIORITY LINE FOLLOWER CONTROLLER 🚀")
    print(" " * 25 + "📐 LÍNEA = PRIORIDAD | 🚦 SEMÁFORO = LÍMITE")
    print(" " * 15 + "✨ SIGUE LÍNEAS PRIORITARIO - SEMÁFORO SOLO LIMITA VELOCIDAD ✨")
    print("=" * 90 + "\033[0m")
    
    node = TrafficLineController()
    
    try:
        print("\033[1;32m[INFO] ✅ Sistema prioritario de línea iniciado correctamente\033[0m")
        print("\033[1;34m[INFO] 🎥 Cámara CSI procesando línea con PRIORIDAD ABSOLUTA\033[0m")
        print("\033[1;34m[INFO] 📷 Cámara USB detectando semáforos como LÍMITE DE VELOCIDAD\033[0m")
        print("\033[1;33m[INFO] ⚡ Línea controla dirección y velocidad | Semáforo limita velocidad máxima\033[0m")
        print("\033[1;35m[INFO] 🚦 Límites: Rojo=STOP | Amarillo=0.15m/s | Verde=0.3m/s | Sin=0.2m/s\033[0m\n")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\033[1;33m[INFO] 🛑 Interrupción detectada - Deteniendo sistema prioritario\033[0m")
    except Exception as e:
        print(f"\n\033[1;31m[ERROR] ❌ Error en sistema prioritario: {str(e)}\033[0m")
    finally:
        print("\033[1;36m[INFO] 🔄 Cerrando controlador prioritario...\033[0m")
        
        # Enviar comando de stop final
        final_twist = Twist()
        final_twist.linear.x = 0.0
        final_twist.angular.z = 0.0
        node.cmd_vel_pub.publish(final_twist)
        
        node.destroy_node()
        rclpy.shutdown()
        
        # Estadísticas finales
        elapsed = time.time() - node.start_time
        print("\n" + "-" * 90)
        print(f"\033[1m📊 ESTADÍSTICAS FINALES DEL SISTEMA PRIORITARIO\033[0m")
        print(f"⏱️  Tiempo total de operación: {elapsed:.2f} segundos")
        print(f"🎥 Frames CSI procesados: {node.frame_count_csi} (FPS: {node.frame_count_csi / elapsed if elapsed > 0 else 0:.2f})")
        print(f"📷 Frames USB procesados: {node.frame_count_usb} (FPS: {node.frame_count_usb / elapsed if elapsed > 0 else 0:.2f})")
        print(f"🚦 Límites de velocidad aplicados:")
        for color, count in node.traffic_detections.items():
            emoji = node.emojis[color]
            limit = node.TRAFFIC_SPEED_LIMITS[color]
            print(f"   {emoji} {color.capitalize()}: {count} veces (límite: {limit:.2f} m/s)")
        print(f"📐 Estado final de línea: {'Detectada' if node.current_line_found else 'No detectada'}")
        print(f"📐 Velocidad final de línea: {node.current_line_throttle:.3f} m/s")
        print("-" * 90 + "\n")


if __name__ == '__main__':
    main()