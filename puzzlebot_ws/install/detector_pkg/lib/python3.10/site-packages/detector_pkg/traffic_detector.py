#!/usr/bin/env python3
#nombre del code :  traffic_detector.py
# COMANDO PARA CORRER EL NODO: ./install/detector_pkg/bin/traffic_detector

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
from ultralytics import YOLO
from ultralytics.utils import LOGGER
LOGGER.setLevel("WARNING")


class TrafficSignalDetector(Node):
    def __init__(self):
        super().__init__('traffic_signal_detector')
        
        # Configuración de QoS compatible con el nodo de la cámara
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Suscriptor a la cámara del PuzzleBot con QoS compatible (CompressedImage)
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/puzzlebot/usb_camera/image_raw/compressed',
            self.image_callback,
            qos_profile
        )
        
        # Publicador para el color detectado
        self.signal_pub = self.create_publisher(String, '/traffic_light_color', 10)
        
        # Publicador de imagen debug (CompressedImage)
        self.debug_pub = self.create_publisher(CompressedImage, '/debug_image_traffic/compressed', 10)
        
        # Para evitar publicar el mismo color repetidamente
        self.last_color = None
        self.same_color_count = 0
        self.min_confirmations = 3  # Número de detecciones iguales para confirmar
        
        # Contador de detecciones
        self.total_detections = 0
        self.color_counts = {'red': 0, 'yellow': 0, 'green': 0}
        self.start_time = time.time()
        
        # === PERFILES HSV PARA DIFERENTES CONDICIONES ===
        self.hsv_profiles = {
            'default': {
                'red1': ([0, 60, 40], [15, 255, 255]),
                'red2': ([165, 60, 40], [179, 255, 255]),
                'yellow': ([15, 80, 80], [50, 255, 255]),
                'green': ([35, 60, 60], [100, 255, 255])
            },
            'usb_cam_bright': {
                'red1': ([0, 80, 60], [12, 255, 255]),
                'red2': ([165, 80, 60], [179, 255, 255]),
                'yellow': ([18, 100, 100], [45, 255, 255]),
                'green': ([42, 90, 90], [95, 255, 255])
            },
            'csi_lowlight': {
                'red1': ([0, 40, 30], [15, 255, 255]),
                'red2': ([165, 40, 30], [179, 255, 255]),
                'yellow': ([15, 60, 60], [50, 255, 255]),
                'green': ([35, 40, 40], [100, 255, 255])
            },
            'indoor': {
                'red1': ([0, 50, 30], [15, 255, 255]),
                'red2': ([165, 50, 30], [179, 255, 255]),
                'yellow': ([18, 70, 70], [50, 255, 255]),
                'green': ([40, 60, 60], [100, 255, 255])
            },
            'outdoor': {
                'red1': ([0, 80, 60], [10, 255, 255]),
                'red2': ([160, 80, 60], [179, 255, 255]),
                'yellow': ([18, 90, 90], [50, 255, 255]),
                'green': ([40, 80, 80], [100, 255, 255])
            }
        }
        # Selecciona el perfil activo aquí:
        self.active_hsv_profile = 'default'  # Cambia a 'usb_cam_bright', 'csi_lowlight', 'indoor', 'outdoor' según tu cámara/ambiente
        self._set_hsv_ranges_from_profile()
        
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

        # Modelo YOLO para semáforos
        self.model = YOLO('/home/serch/puzzlebot_ws/src/detector_pkg/detector_pkg/bestTrafficRObo.pt')
        self.class_map = {0: 'green', 1: 'red', 2: 'semaforo', 3: 'yellow'}

    def print_header(self):
        c = self.terminal_colors
        print(f"\n{c['bold']}{'='*80}{c['reset']}")
        print(f"{c['cyan']}🚦 DETECTOR DE SEMÁFOROS AVANZADO 🚦{c['reset']}")
        print(f"{c['bold']}{'='*80}{c['reset']}")
        print(f"{c['blue']}✅ Inicializado y conectado al tópico: {c['magenta']}/puzzlebot/usb_camera/image_raw/compressed{c['reset']}")
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

    def compressed_imgmsg_to_cv2(self, compressed_msg):
        """Convierte un mensaje CompressedImage de ROS a una imagen de OpenCV"""
        try:
            # Decodificar los datos comprimidos usando OpenCV
            np_arr = np.frombuffer(compressed_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            return cv_image
        except Exception as e:
            self.get_logger().error(f"Error al decodificar imagen comprimida: {e}")
            return None

    def cv2_to_compressed_imgmsg(self, cv_image, header=None):
        """Convierte una imagen de OpenCV a un mensaje CompressedImage"""
        try:
            # Codificar la imagen como JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]  # Calidad JPEG al 80%
            result, encoded_img = cv2.imencode('.jpg', cv_image, encode_param)
            
            if not result:
                self.get_logger().error("Error al codificar imagen a JPEG")
                return None
            
            # Crear el mensaje CompressedImage
            compressed_msg = CompressedImage()
            if header:
                compressed_msg.header = header
            else:
                compressed_msg.header.stamp = self.get_clock().now().to_msg()
                compressed_msg.header.frame_id = "camera_frame"
            
            compressed_msg.format = "jpeg"
            compressed_msg.data = encoded_img.tobytes()
            
            return compressed_msg
        except Exception as e:
            self.get_logger().error(f"Error al crear mensaje comprimido: {e}")
            return None

    def get_dominant_color_in_bbox(self, frame, bbox):
        """Determina el color dominante en un bounding box"""
        x1, y1, x2, y2 = bbox
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return None
        
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mean_hue = np.mean(hsv_roi[:,:,0])
        mean_sat = np.mean(hsv_roi[:,:,1])
        mean_val = np.mean(hsv_roi[:,:,2])
        
        # Clasificación robusta por hue
        if ((mean_hue < 25 or mean_hue > 155) and mean_sat > 35 and mean_val > 35):
            return 'red'
        elif (15 < mean_hue < 50 and mean_sat > 35 and mean_val > 35):
            return 'yellow'
        elif (35 < mean_hue < 100 and mean_sat > 35 and mean_val > 35):
            return 'green'
        else:
            return None

    def image_callback(self, msg):
        """Callback principal para procesar imágenes comprimidas"""
        try:
            # Decodificar imagen comprimida
            frame = self.compressed_imgmsg_to_cv2(msg)
            if frame is None:
                self.get_logger().error("No se pudo decodificar la imagen comprimida")
                return
        except Exception as e:
            self.get_logger().error(f"Error al procesar imagen comprimida: {e}")
            return

        # Redimensionar para procesamiento más rápido
        frame_resized = cv2.resize(frame, (320, 240))
        frame_debug = frame_resized.copy()
        
        # Variables para la mejor detección
        color_detected = None
        best_conf = 0
        best_area = 0
        best_box = None
        best_class = None
        all_detections = []  # Para almacenar todas las detecciones
        
        # Detección con modelo YOLO
        try:
            results = self.model(frame_resized)
            for r in results:
                for box in r.boxes:
                    class_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    x1, y1, x2, y2 = xyxy
                    w, h = x2 - x1, y2 - y1
                    area = w * h
                    aspect_ratio = w / h if h > 0 else 0
                    class_name = self.class_map.get(class_id, 'unknown')
                    
                    # Guardar todas las detecciones para el debug
                    all_detections.append({
                        'class_name': class_name,
                        'conf': conf,
                        'bbox': (x1, y1, x2, y2),
                        'area': area,
                        'aspect_ratio': aspect_ratio,
                        'is_valid': 0.7 < aspect_ratio < 1.3 and conf > 0.4
                    })
                    
                    # Solo considerar si el bounding box es cuadrado y confianza suficiente
                    if 0.7 < aspect_ratio < 1.3 and conf > 0.4:
                        if conf > best_conf or (conf == best_conf and area > best_area):
                            best_conf = conf
                            best_area = area
                            best_box = (x1, y1, x2, y2)
                            best_class = class_name
        except Exception as e:
            self.get_logger().error(f"Error en detección YOLO: {e}")

        # === CREACIÓN DE IMAGEN DEBUG MEJORADA ===
        self.create_debug_image(frame_debug, all_detections, best_box, best_class, best_conf)
        
        # Publicar color detectado si hay
        if best_class is not None and best_class != 'semaforo':
            color_detected = best_class
            if color_detected:
                color_msg = String()
                color_msg.data = color_detected
                self.signal_pub.publish(color_msg)
                self.print_detection(color_detected, int(best_conf*10))
        
        # Publicar imagen debug comprimida
        try:
            debug_compressed_msg = self.cv2_to_compressed_imgmsg(frame_debug, msg.header)
            if debug_compressed_msg:
                self.debug_pub.publish(debug_compressed_msg)
        except Exception as e:
            self.get_logger().error(f"Error al publicar imagen debug: {e}")

    def create_debug_image(self, frame_debug, all_detections, best_box, best_class, best_conf):
        """Crea una imagen debug mejorada con información visual"""
        # 1. Crear overlay semi-transparente para información
        overlay = frame_debug.copy()
        
        # 2. Dibujar todas las detecciones con diferentes estilos
        for detection in all_detections:
            x1, y1, x2, y2 = detection['bbox']
            class_name = detection['class_name']
            conf = detection['conf']
            is_valid = detection['is_valid']
            
            # Colores mejorados para cada clase
            if class_name == 'green':
                color = (0, 255, 0)
                text_color = (255, 255, 255)
            elif class_name == 'red':
                color = (0, 0, 255)
                text_color = (255, 255, 255)
            elif class_name == 'yellow':
                color = (0, 255, 255)
                text_color = (0, 0, 0)
            elif class_name == 'semaforo':
                color = (255, 255, 255)
                text_color = (0, 0, 0)
            else:
                color = (128, 128, 128)
                text_color = (255, 255, 255)
            
            # Estilo de rectángulo según validez
            if is_valid:
                # Detección válida: línea gruesa
                cv2.rectangle(frame_debug, (x1, y1), (x2, y2), color, 3)
                # Esquinas destacadas para detecciones válidas
                corner_length = 15
                cv2.line(frame_debug, (x1, y1), (x1 + corner_length, y1), color, 5)
                cv2.line(frame_debug, (x1, y1), (x1, y1 + corner_length), color, 5)
                cv2.line(frame_debug, (x2, y2), (x2 - corner_length, y2), color, 5)
                cv2.line(frame_debug, (x2, y2), (x2, y2 - corner_length), color, 5)
                cv2.line(frame_debug, (x1, y2), (x1 + corner_length, y2), color, 5)
                cv2.line(frame_debug, (x1, y2), (x1, y2 - corner_length), color, 5)
                cv2.line(frame_debug, (x2, y1), (x2 - corner_length, y1), color, 5)
                cv2.line(frame_debug, (x2, y1), (x2, y1 + corner_length), color, 5)
            else:
                # Detección inválida: línea punteada
                cv2.rectangle(frame_debug, (x1, y1), (x2, y2), (128, 128, 128), 2)
                # Diagonal para indicar que es inválida
                cv2.line(frame_debug, (x1, y1), (x2, y2), (0, 0, 255), 2)
            
            # Fondo semi-transparente para el texto
            label = f"{class_name} {conf:.2f}"
            (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            
            # Rectángulo de fondo para el texto
            cv2.rectangle(overlay, (x1, y1 - text_h - 10), (x1 + text_w + 10, y1), color, -1)
            
            # Texto principal
            cv2.putText(frame_debug, label, (x1 + 5, y1 - 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)
            
            # Información adicional si es detección válida
            if is_valid:
                area_text = f"Area: {detection['area']}"
                ratio_text = f"R: {detection['aspect_ratio']:.2f}"
                cv2.putText(frame_debug, area_text, (x1, y2 + 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                cv2.putText(frame_debug, ratio_text, (x1, y2 + 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # 3. Destacar la mejor detección con un marco especial
        if best_box is not None:
            x1, y1, x2, y2 = best_box
            # Marco doble para la mejor detección
            cv2.rectangle(frame_debug, (x1-5, y1-5), (x2+5, y2+5), (255, 255, 0), 3)
            cv2.rectangle(frame_debug, (x1-2, y1-2), (x2+2, y2+2), (0, 255, 255), 2)
            
            # Etiqueta "MEJOR" en la esquina superior
            cv2.putText(frame_debug, "MEJOR", (x1, y1-25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # 4. Panel de información en la parte superior
        info_height = 80
        cv2.rectangle(overlay, (0, 0), (320, info_height), (0, 0, 0), -1)
        
        # Información del estado actual
        status_text = f"Detecciones: {len(all_detections)}"
        valid_count = sum(1 for d in all_detections if d['is_valid'])
        valid_text = f"Validas: {valid_count}"
        
        if best_class:
            best_text = f"Mejor: {best_class.upper()} ({best_conf:.2f})"
            color_for_best = (0, 255, 0) if best_class == 'green' else \
                            (0, 0, 255) if best_class == 'red' else \
                            (0, 255, 255) if best_class == 'yellow' else (255, 255, 255)
        else:
            best_text = "Mejor: NINGUNA"
            color_for_best = (128, 128, 128)
        
        # Textos del panel de información
        cv2.putText(frame_debug, status_text, (10, 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame_debug, valid_text, (10, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame_debug, best_text, (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_for_best, 2)
        
        # Tiempo de ejecución
        current_time = time.time()
        runtime = current_time - self.start_time
        minutes, seconds = divmod(runtime, 60)
        time_text = f"Tiempo: {int(minutes):02d}:{int(seconds):02d}"
        cv2.putText(frame_debug, time_text, (200, 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Número de detección
        detection_text = f"Det #{self.total_detections}"
        cv2.putText(frame_debug, detection_text, (200, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # 5. Aplicar el overlay con transparencia
        alpha = 0.7
        frame_debug[:] = cv2.addWeighted(overlay, 1-alpha, frame_debug, alpha, 0)
        
        # 6. Líneas de referencia (opcional, para ayudar en calibración)
        # Línea central vertical
        cv2.line(frame_debug, (160, 0), (160, 240), (100, 100, 100), 1)
        # Línea central horizontal  
        cv2.line(frame_debug, (0, 120), (320, 120), (100, 100, 100), 1)

    def _set_hsv_ranges_from_profile(self):
        """Configura los rangos HSV según el perfil activo"""
        p = self.hsv_profiles[self.active_hsv_profile]
        self.lower_red1 = np.array(p['red1'][0])
        self.upper_red1 = np.array(p['red1'][1])
        self.lower_red2 = np.array(p['red2'][0])
        self.upper_red2 = np.array(p['red2'][1])
        self.lower_yellow = np.array(p['yellow'][0])
        self.upper_yellow = np.array(p['yellow'][1])
        self.lower_green = np.array(p['green'][0])
        self.upper_green = np.array(p['green'][1])

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