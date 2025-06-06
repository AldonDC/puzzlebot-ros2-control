#!/usr/bin/env python3
#nombre del code :  traffic_detector.py
# COMANDO PARA CORRER EL NODO: ./install/detector_pkg/bin/traffic_detector

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time

class TrafficSignalDetector(Node):
    def __init__(self):
        super().__init__('traffic_signal_detector')
        
        # Configuración de QoS compatible con el nodo de la cámara
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Suscriptor a la cámara del PuzzleBot con QoS compatible
        self.image_sub = self.create_subscription(
            Image,
            '/puzzlebot/usb_camera/image_raw',
            self.image_callback,
            qos_profile
        )
        
        # Publicador para el color detectado
        self.signal_pub = self.create_publisher(String, '/traffic_light_color', 10)
        
        # Para evitar publicar el mismo color repetidamente
        self.last_color = None
        self.same_color_count = 0
        self.min_confirmations = 3  # Número de detecciones iguales para confirmar
        
        # Contador de detecciones
        self.total_detections = 0
        self.color_counts = {'red': 0, 'yellow': 0, 'green': 0}
        self.start_time = time.time()
        
        # RANGOS HSV DEFINITIVOS Y ROBUSTOS - ACTUALIZADOS
        # Rojo (debido al círculo del espacio HSV, necesitamos dos rangos para el rojo)
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([179, 255, 255])  # Cambiado de 180 a 179
        
        # Amarillo - Rango robusto de Ivan ✔️
        self.lower_yellow = np.array([18, 80, 80])
        self.upper_yellow = np.array([35, 255, 255])
        
        # Verde - Rango robusto actualizado
        self.lower_green = np.array([40, 70, 70])
        self.upper_green = np.array([85, 255, 255])
        
        # Colores para terminal
        self.terminal_colors = {
            'red': '\033[1;31m',    # Rojo brillante
            'yellow': '\033[1;33m', # Amarillo brillante
            'green': '\033[1;32m',  # Verde brillante
            'reset': '\033[0m',     # Resetear color
            'bold': '\033[1m',      # Negrita
            'blue': '\033[1;34m',   # Azul brillante
            'magenta': '\033[1;35m',# Magenta brillante
            'cyan': '\033[1;36m'    # Cian brillante
        }
        
        # Emojis para colores
        self.color_emojis = {
            'red': '🔴',
            'yellow': '🟡',
            'green': '🟢',
            'none': '⚪'
        }
        
        # Imprimir encabezado bonito
        self.print_header()
        
        # Estado inicial
        self.print_status("Esperando detecciones...", "⏳")

    def print_header(self):
        c = self.terminal_colors
        print(f"\n{c['bold']}{'='*80}{c['reset']}")
        print(f"{c['cyan']}🚦 DETECTOR DE SEMÁFOROS AVANZADO 🚦{c['reset']}")
        print(f"{c['bold']}{'='*80}{c['reset']}")
        print(f"{c['blue']}✅ Inicializado y conectado al tópico: {c['magenta']}/puzzlebot/usb_camera/image_raw{c['reset']}")
        print(f"{c['blue']}📡 Publicando en: {c['magenta']}/traffic_light_color{c['reset']}")
        print(f"{c['bold']}{'='*80}{c['reset']}\n")

    def print_status(self, message, emoji=""):
        c = self.terminal_colors
        print(f"{c['cyan']}{emoji} {message}{c['reset']}")

    def print_detection(self, color, confidence=0):
        c = self.terminal_colors
        emoji = self.color_emojis.get(color, '❓')
        color_text = f"{c[color]}{color.upper()}{c['reset']}" if color in c else color
        
        runtime = time.time() - self.start_time
        hours, remainder = divmod(runtime, 3600)
        minutes, seconds = divmod(remainder, 60)
        
        self.total_detections += 1
        if color in self.color_counts:
            self.color_counts[color] += 1
        
        # Calcula las estadísticas
        stats = {
            'red_percent': (self.color_counts['red'] / self.total_detections) * 100 if self.total_detections > 0 else 0,
            'yellow_percent': (self.color_counts['yellow'] / self.total_detections) * 100 if self.total_detections > 0 else 0,
            'green_percent': (self.color_counts['green'] / self.total_detections) * 100 if self.total_detections > 0 else 0
        }
        
        print(f"\n{c['bold']}{'─'*80}{c['reset']}")
        print(f"{c['bold']}⏱️ TIEMPO: {int(hours):02d}:{int(minutes):02d}:{int(seconds):02d} | 🔍 DETECCIÓN #{self.total_detections}{c['reset']}")
        print(f"{c['bold']}{'─'*80}{c['reset']}")
        print(f"  {emoji} Semáforo detectado: {color_text} (confianza: {confidence}/10)")
        print(f"  📊 Estadísticas:")
        print(f"    {self.color_emojis['red']} Rojo: {self.color_counts['red']} ({stats['red_percent']:.1f}%)")
        print(f"    {self.color_emojis['yellow']} Amarillo: {self.color_counts['yellow']} ({stats['yellow_percent']:.1f}%)")
        print(f"    {self.color_emojis['green']} Verde: {self.color_counts['green']} ({stats['green_percent']:.1f}%)")
        print(f"{c['bold']}{'─'*80}{c['reset']}\n")

    def imgmsg_to_cv2(self, img_msg):
        """Convierte un mensaje de Image de ROS a una imagen de OpenCV sin usar cv_bridge"""
        # Obtener las dimensiones de la imagen
        height = img_msg.height
        width = img_msg.width
        
        # Crear un array NumPy con los datos de la imagen
        if img_msg.encoding == 'bgr8':
            dtype = np.uint8
            shape = (height, width, 3)
        elif img_msg.encoding == 'mono8':
            dtype = np.uint8
            shape = (height, width)
        else:
            self.get_logger().error(f"Formato de imagen no soportado: {img_msg.encoding}")
            return None
            
        # Convertir los bytes de la imagen a un array NumPy
        data = np.frombuffer(img_msg.data, dtype=dtype)
        
        # Reshapear el array a las dimensiones de la imagen
        cv_image = data.reshape(shape)
        
        return cv_image

    def detect_colored_circles(self, frame):
        """Detecta círculos de colores específicos en la imagen"""
        # Convertir a HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Crear máscaras para cada color con los rangos robustos actualizados
        mask_red1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask_red2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        mask_yellow = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
        # Aplicar operaciones morfológicas para mejorar la detección
        kernel = np.ones((5, 5), np.uint8)
        
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)
        
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
        
        # Encontrar contornos en cada máscara
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Resultados
        circles = {
            'red': [],
            'yellow': [],
            'green': []
        }
        
        # Parámetros de detección mejorados
        MIN_AREA = 80  # Área mínima en píxeles
        MAX_AREA = 5000  # Área máxima en píxeles
        MIN_CIRCULARITY = 0.7  # Circularidad mínima (más estricto)
        MIN_RADIUS = 5  # Radio mínimo en píxeles
        MAX_ASPECT_RATIO = 1.3  # Relación de aspecto máxima (1.0 sería un círculo perfecto)
        
        def process_contours(contours, color):
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Verificar área mínima y máxima
                if area < MIN_AREA or area > MAX_AREA:
                    continue
                
                # Encontrar el rectángulo que encierra el contorno
                rect = cv2.minAreaRect(contour)
                width, height = rect[1]
                
                # Calcular relación de aspecto
                aspect_ratio = max(width, height) / (min(width, height) + 1e-6)
                if aspect_ratio > MAX_ASPECT_RATIO:
                    continue
                
                # Encontrar el círculo mínimo que encierra el contorno
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                
                # Verificar radio mínimo
                if radius < MIN_RADIUS:
                    continue
                
                # Calcular circularidad
                perimeter = cv2.arcLength(contour, True)
                circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
                
                # Verificar circularidad
                if circularity > MIN_CIRCULARITY:
                    circles[color].append((center, radius, area))
        
        # Procesar contornos para cada color
        process_contours(contours_red, 'red')
        process_contours(contours_yellow, 'yellow')
        process_contours(contours_green, 'green')
        
        return circles

    def determine_dominant_color(self, circles):
        """Determina el color dominante basado en los círculos detectados"""
        colors = list(circles.keys())
        max_area = 0
        dominant_color = None
        largest_circle = None
        confidence = 0
        
        for color in colors:
            for circle in circles[color]:
                center, radius, area = circle
                if area > max_area:
                    max_area = area
                    dominant_color = color
                    largest_circle = (center, radius)
                    # Calcular confianza basada en el área y la circularidad (1-10)
                    confidence = min(10, int(area / 100))
        
        return dominant_color, largest_circle, confidence

    def image_callback(self, msg):
        try:
            # Convertir mensaje de imagen a formato OpenCV
            frame = self.imgmsg_to_cv2(msg)
            if frame is None:
                return
        except Exception as e:
            self.get_logger().error(f"Error al convertir imagen: {e}")
            return

        # Redimensionar para mejor rendimiento
        frame = cv2.resize(frame, (320, 240))
        
        # Detectar círculos de colores específicos
        circles = self.detect_colored_circles(frame)
        
        # Determinar el color dominante
        current_color, best_circle, confidence = self.determine_dominant_color(circles)
        
        # Procesar el color detectado
        if current_color:
            # Lógica de confirmación para evitar falsos positivos
            if current_color == self.last_color:
                self.same_color_count += 1
            else:
                self.same_color_count = 1
                self.last_color = current_color
            
            # Publicar solo si tenemos suficientes confirmaciones
            if self.same_color_count >= self.min_confirmations:
                color_msg = String()
                color_msg.data = current_color
                self.signal_pub.publish(color_msg)
                
                # Mostrar información en la terminal de manera bonita
                self.print_detection(current_color, confidence)
                
def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignalDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n🛑 Detector detenido por el usuario. ¡Hasta pronto! 👋\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()