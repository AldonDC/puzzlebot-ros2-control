import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # AGREGADO: Para publicar el nombre de la clase
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import sys
from ultralytics import YOLO
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class SimpleTrafficSignDetector(Node):
    def __init__(self):
        super().__init__('simple_traffic_sign_detector')

        # Configuración de suscriptores y publicadores
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Suscribirse a la cámara USB del PuzzleBot
        self.subscription = self.create_subscription(
            Image, '/puzzlebot/usb_camera/image_raw', self.image_callback, qos_profile)
        
        # Publicar imagen con detecciones para visualización    
        self.debug_pub = self.create_publisher(Image, '/debug_image_sign', 10)
        
        # AGREGADO: Publicador para enviar el nombre de la clase detectada
        self.class_pub = self.create_publisher(String, '/detected_traffic_sign', 10)
        
        self.bridge = CvBridge()

        # MODELO ÚNICO - Actualizar ruta si es necesario
        self.model_path = '/home/serch/puzzlebot_ws/src/detector_pkg/detector_pkg/bestLALO.pt'
        
        self.load_model()
        
        # Configuración de detección
        self.confidence_threshold = 0.5
        self.iou_threshold = 0.4
        
        # Variables para control de detecciones
        self.last_detected_signs = {}
        self.detection_cooldown = 1.5
        self.frame_count = 0
        self.start_time = time.time()
        
        # Control de estabilidad
        self.detection_history = {}
        self.stability_threshold = 2
        
        # MODIFICADO: MAPEO DE CLASES SIN SEMAFORO (clase 4)
        self.traffic_classes = {
            0: "Forward",
            1: "GiveWay", 
            2: "Right",
            3: "Roundabout",
            # 4: "Semaforo", ELIMINADO
            5: "Stop",
            6: "construction",
            7: "left"
        }
        
        # Sistema de filtrado mejorado
        self.noise_filter_enabled = True
        self.min_detection_size = 800
        self.max_detection_size = 100000
        self.aspect_ratio_min = 0.4
        self.aspect_ratio_max = 2.5
        
        # MODIFICADO: EMOJIS SIN SEMAFORO
        self.sign_emojis = {
            "Forward": "⬆️",
            "GiveWay": "⚠️",
            "Right": "➡️", 
            "Roundabout": "🔄",
            # "Semaforo": "🚦", ELIMINADO
            "Stop": "🛑",
            "construction": "🚧",
            "left": "⬅️"
        }
        
        # MODIFICADO: COLORES SIN LA CLASE 4
        self.class_colors = {
            0: (0, 255, 0),      # Forward - Verde
            1: (0, 165, 255),    # GiveWay - Naranja
            2: (255, 0, 0),      # Right - Azul
            3: (255, 255, 0),    # Roundabout - Cian
            # 4: (0, 255, 255),    # Semaforo - Amarillo ELIMINADO
            5: (0, 0, 255),      # Stop - Rojo
            6: (0, 140, 255),    # Construction - Naranja oscuro
            7: (255, 0, 255)     # Left - Magenta
        }
        
        # Estadísticas
        self.total_detections = 0
        self.unique_signs_detected = set()
        self.detection_count_per_class = {name: 0 for name in self.traffic_classes.values()}
        
        # AGREGADO: Control para publicación de clases
        self.last_published_class = None
        self.last_publish_time = 0
        self.publish_cooldown = 1.0  # Publicar la misma clase máximo cada 1 segundo
        
        # Imprimir información inicial
        self._print_startup_info()

    def load_model(self):
        """Carga y verifica el modelo"""
        self.model = None
        
        try:
            if os.path.exists(self.model_path):
                self.model = YOLO(self.model_path)
                self.get_logger().info(f"✅ Modelo '{self.model_path}' cargado correctamente")
                
                # Verificar información del modelo
                if hasattr(self.model.model, 'names'):
                    model_classes = self.model.model.names
                    self.get_logger().info(f"📋 Clases del modelo: {model_classes}")
                    
                    # Verificar compatibilidad
                    if len(model_classes) == 8:
                        self.get_logger().info("✅ Modelo compatible - 8 clases detectadas")
                        
                        # Actualizar clases si el modelo tiene nombres diferentes
                        for class_id, class_name in model_classes.items():
                            if class_id in self.traffic_classes:
                                if self.traffic_classes[class_id] != class_name:
                                    self.get_logger().info(f"🔄 Actualizando clase {class_id}: {self.traffic_classes[class_id]} -> {class_name}")
                                    self.traffic_classes[class_id] = class_name
                    else:
                        self.get_logger().warning(f"⚠️ El modelo tiene {len(model_classes)} clases, se esperaban 8")
                        
            else:
                self.get_logger().error(f"❌ Modelo '{self.model_path}' no encontrado")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error cargando modelo: {str(e)}")

    def _print_startup_info(self):
        """Imprime información de inicio mejorada"""
        print("\n" + "=" * 80)
        print(" " * 20 + "\033[1;36m🚗 PUZZLEBOT TRAFFIC DETECTOR v2.0\033[0m")
        print(" " * 25 + "\033[1;33m⚡ MODELO ACTUALIZADO ⚡\033[0m")
        print("=" * 80)
        print(f"\033[1mIniciado\033[0m: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"\033[1mModelo\033[0m: {os.path.basename(self.model_path)}")
        print(f"  └─ Estado: {'✅ Cargado' if self.model else '❌ Error'}")
        print(f"  └─ Confianza mínima: {self.confidence_threshold}")
        print(f"  └─ IoU threshold: {self.iou_threshold}")
        print("-" * 80)
        print("\033[1m🎯 CLASES CONFIGURADAS PARA DETECCIÓN:\033[0m")
        for class_id, name in self.traffic_classes.items():
            emoji = self.sign_emojis.get(name, "🚦")
            color_rgb = self.class_colors[class_id]
            print(f"  {class_id:2d}: {emoji} {name:12} (Color: RGB{color_rgb})")
        print("\033[1;33m  ⚠️  Nota: Clase 4 (Semaforo) ha sido deshabilitada\033[0m")
        print("-" * 80)
        print("\033[1m📡 PUBLICANDO CLASES DETECTADAS EN: /detected_traffic_sign\033[0m")
        print("-" * 80)
        print("\033[1m🔍 MONITOREANDO DETECCIONES EN TIEMPO REAL:\033[0m")
        print("-" * 80)
        sys.stdout.flush()

    def detect_traffic_signs(self, frame):
        """Ejecuta detección con el modelo optimizado"""
        detections = []
        
        if self.model is None:
            return detections
            
        try:
            # Ejecutar inferencia con parámetros optimizados
            results = self.model(
                frame, 
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                verbose=False,
                imgsz=640,
                device='cpu'  # Especificar dispositivo
            )
            
            for result in results:
                boxes = result.boxes
                if boxes is not None and len(boxes) > 0:
                    xyxy = boxes.xyxy.cpu().numpy()
                    conf = boxes.conf.cpu().numpy()
                    cls = boxes.cls.cpu().numpy()
                    
                    for i in range(len(xyxy)):
                        class_id = int(cls[i])
                        
                        # MODIFICADO: Ignorar clase 4 (Semaforo) y solo procesar clases conocidas
                        if class_id == 4 or class_id not in self.traffic_classes:
                            continue
                        
                        x1, y1, x2, y2 = xyxy[i].astype(int)
                        confidence = float(conf[i])
                        
                        # Aplicar filtros de calidad mejorados
                        if self.apply_quality_filters(x1, y1, x2, y2, frame):
                            class_name = self.traffic_classes[class_id]
                            emoji = self.sign_emojis[class_name]
                            color = self.class_colors[class_id]
                            area = (x2 - x1) * (y2 - y1)
                            
                            detections.append({
                                'class_id': class_id,
                                'class_name': class_name,
                                'confidence': confidence,
                                'bbox': (x1, y1, x2, y2),
                                'emoji': emoji,
                                'color': color,
                                'area': area,
                                'center': ((x1 + x2) // 2, (y1 + y2) // 2)
                            })
                            
        except Exception as e:
            self.get_logger().error(f"Error en detección: {str(e)}")
            
        return detections

    def apply_quality_filters(self, x1, y1, x2, y2, frame):
        """Aplica filtros de calidad mejorados para detecciones"""
        # Filtro de área
        area = (x2 - x1) * (y2 - y1)
        if area < self.min_detection_size or area > self.max_detection_size:
            return False
        
        # Filtro de aspecto ratio
        width = x2 - x1
        height = y2 - y1
        if height == 0:
            return False
            
        aspect_ratio = width / height
        if aspect_ratio < self.aspect_ratio_min or aspect_ratio > self.aspect_ratio_max:
            return False
        
        # Filtro de posición (evitar bordes)
        frame_h, frame_w = frame.shape[:2]
        margin = 10
        if (x1 < margin or y1 < margin or 
            x2 > frame_w - margin or y2 > frame_h - margin):
            return False
        
        # Filtro de contenido de la región
        try:
            roi = frame[y1:y2, x1:x2]
            if roi.size > 0:
                # Verificar que no sea una región completamente oscura o clara
                mean_intensity = np.mean(roi)
                if mean_intensity < 15 or mean_intensity > 240:
                    return False
                
                # Verificar varianza (evitar regiones uniformes)
                variance = np.var(roi)
                if variance < 100:  # Muy uniforme
                    return False
        except:
            return False
        
        return True

    def publish_detected_class(self, class_name):
        """AGREGADO: Publica el nombre de la clase detectada"""
        current_time = time.time()
        
        # Verificar si debemos publicar (cooldown y cambio de clase)
        should_publish = (
            class_name != self.last_published_class or 
            current_time - self.last_publish_time > self.publish_cooldown
        )
        
        if should_publish:
            msg = String()
            msg.data = class_name
            self.class_pub.publish(msg)
            
            # Actualizar estado de publicación
            self.last_published_class = class_name
            self.last_publish_time = current_time
            
            # Log en terminal
            print(f"\n📡 \033[1;35m[PUBLICADO]\033[0m: {class_name} → /detected_traffic_sign")

    def process_detections(self, frame):
        """Procesa las detecciones y aplica filtros de estabilidad mejorados"""
        all_detections = self.detect_traffic_signs(frame)
        
        stable_detections = []
        current_time = time.time()
        
        # AGREGADO: Para rastrear la detección más confiable
        best_detection = None
        best_confidence = 0
        
        for detection in all_detections:
            x1, y1, x2, y2 = detection['bbox']
            class_name = detection['class_name']
            confidence = detection['confidence']
            
            # Sistema de estabilidad basado en regiones
            region_size = 80  # Tamaño de región para agrupación
            region_key = f"{int(x1/region_size)}_{int(y1/region_size)}"
            detection_key = f"{class_name}_{region_key}"
            
            if detection_key not in self.detection_history:
                self.detection_history[detection_key] = []
            
            self.detection_history[detection_key].append({
                'time': current_time,
                'confidence': confidence,
                'bbox': detection['bbox']
            })
            
            # Limpiar historial antiguo (mantener últimos 3 segundos)
            self.detection_history[detection_key] = [
                det for det in self.detection_history[detection_key] 
                if current_time - det['time'] < 3.0
            ]
            
            # Verificar estabilidad
            stable_count = len(self.detection_history[detection_key])
            if stable_count >= self.stability_threshold:
                
                # AGREGADO: Actualizar mejor detección
                if confidence > best_confidence:
                    best_detection = detection
                    best_confidence = confidence
                
                # Sistema de cooldown mejorado para terminal
                show_in_terminal = (detection_key not in self.last_detected_signs or 
                                  current_time - self.last_detected_signs[detection_key] > self.detection_cooldown)
                
                if show_in_terminal:
                    # Indicadores de estabilidad
                    if stable_count >= 8:
                        stability_indicator = "🎯"
                        stability_text = "ESTABLE"
                    elif stable_count >= 5:
                        stability_indicator = "⚡"
                        stability_text = "FIRME"
                    else:
                        stability_indicator = "📍"
                        stability_text = "DETECTADO"
                    
                    # Estimación de distancia mejorada
                    area = detection['area']
                    if area > 25000:
                        distance_est = "MUY CERCA"
                        distance_color = "\033[1;31m"  # Rojo
                    elif area > 12000:
                        distance_est = "CERCA"
                        distance_color = "\033[1;33m"  # Amarillo
                    elif area > 6000:
                        distance_est = "MEDIA"
                        distance_color = "\033[1;36m"  # Cian
                    elif area > 2000:
                        distance_est = "LEJOS"
                        distance_color = "\033[1;37m"  # Blanco
                    else:
                        distance_est = "MUY LEJOS"
                        distance_color = "\033[1;90m"  # Gris
                    
                    # Imprimir detección con formato mejorado
                    print(f"{detection['emoji']} {stability_indicator} "
                          f"\033[1;32m{stability_text}\033[0m: "
                          f"\033[1;37m{class_name}\033[0m | "
                          f"Conf: \033[1;36m{confidence:.3f}\033[0m | "
                          f"Área: {area:,}px² | "
                          f"Dist: {distance_color}{distance_est}\033[0m | "
                          f"Estabilidad: {stable_count}/8")
                    
                    self.last_detected_signs[detection_key] = current_time
                    self.total_detections += 1
                    self.unique_signs_detected.add(class_name)
                    self.detection_count_per_class[class_name] += 1
                
                stable_detections.append(detection)
        
        # AGREGADO: Publicar la mejor detección
        if best_detection:
            self.publish_detected_class(best_detection['class_name'])
        
        return stable_detections

    def draw_detection(self, frame, detection):
        """Dibuja las detecciones con estilo mejorado"""
        x1, y1, x2, y2 = detection['bbox']
        class_name = detection['class_name']
        emoji = detection['emoji']
        confidence = detection['confidence']
        color = detection['color']
        area = detection['area']
        
        # Grosor adaptativo basado en área
        thickness = max(2, min(6, int(area / 4000)))
        
        # Cuadro principal con efecto de profundidad
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        
        # Sombra del cuadro
        shadow_offset = max(2, thickness)
        shadow_color = tuple(int(c * 0.3) for c in color)
        cv2.rectangle(frame, (x1 + shadow_offset, y1 + shadow_offset), 
                     (x2 + shadow_offset, y2 + shadow_offset), shadow_color, thickness)
        
        # Esquinas destacadas más prominentes
        corner_size = max(12, min(30, int(area / 800)))
        corner_thickness = thickness + 1
        
        # Esquinas superiores
        cv2.line(frame, (x1, y1), (x1 + corner_size, y1), color, corner_thickness)
        cv2.line(frame, (x1, y1), (x1, y1 + corner_size), color, corner_thickness)
        cv2.line(frame, (x2, y1), (x2 - corner_size, y1), color, corner_thickness)
        cv2.line(frame, (x2, y1), (x2, y1 + corner_size), color, corner_thickness)
        
        # Esquinas inferiores
        cv2.line(frame, (x1, y2), (x1 + corner_size, y2), color, corner_thickness)
        cv2.line(frame, (x1, y2), (x1, y2 - corner_size), color, corner_thickness)
        cv2.line(frame, (x2, y2), (x2 - corner_size, y2), color, corner_thickness)
        cv2.line(frame, (x2, y2), (x2, y2 - corner_size), color, corner_thickness)
        
        # Tamaño de fuente adaptativo
        if area > 20000:
            font_scale = 1.2
            emoji_scale = 2.5
        elif area > 10000:
            font_scale = 1.0
            emoji_scale = 2.0
        elif area > 5000:
            font_scale = 0.8
            emoji_scale = 1.5
        elif area > 2000:
            font_scale = 0.6
            emoji_scale = 1.2
        else:
            font_scale = 0.5
            emoji_scale = 1.0
        
        text_thickness = max(1, int(font_scale * 2))
        
        # Etiqueta principal
        label = f"{class_name}"
        label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_thickness)[0]
        
        # Fondo de la etiqueta con gradiente
        label_bg_height = label_size[1] + 30
        overlay = frame.copy()
        cv2.rectangle(overlay, (x1, y1 - label_bg_height), 
                     (x1 + label_size[0] + 25, y1), color, -1)
        cv2.addWeighted(overlay, 0.8, frame, 0.2, 0, frame)
        
        # Texto principal con contorno mejorado
        text_x, text_y = x1 + 12, y1 - 12
        
        # Contorno negro
        cv2.putText(frame, label, (text_x, text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), text_thickness + 2)
        # Texto blanco
        cv2.putText(frame, label, (text_x, text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), text_thickness)
        
        # Emoji prominente
        emoji_x = max(10, x1 - 70)
        emoji_y = y1 + int(50 * emoji_scale)
        
        # Contorno del emoji
        cv2.putText(frame, emoji, (emoji_x, emoji_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, emoji_scale, (0, 0, 0), max(3, int(emoji_scale * 4)))
        # Emoji coloreado
        cv2.putText(frame, emoji, (emoji_x, emoji_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, emoji_scale, color, max(2, int(emoji_scale * 2)))
        
        # Información de confianza
        conf_text = f"{confidence:.3f}"
        conf_y = y2 + 25
        cv2.putText(frame, conf_text, (x1, conf_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.7, (0, 0, 0), text_thickness + 1)
        cv2.putText(frame, conf_text, (x1, conf_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.7, color, text_thickness)
        
        # Punto central mejorado
        center_x, center_y = detection['center']
        center_radius = max(4, int(area / 6000))
        cv2.circle(frame, (center_x, center_y), center_radius + 2, (0, 0, 0), -1)
        cv2.circle(frame, (center_x, center_y), center_radius, (0, 255, 0), -1)
        cv2.circle(frame, (center_x, center_y), center_radius + 4, (255, 255, 255), 2)

    def add_info_overlay(self, frame, detections):
        """Añade información del sistema mejorada"""
        h, w = frame.shape[:2]
        
        # Estadísticas en tiempo real
        current_time = time.time()
        fps = self.frame_count / (current_time - self.start_time) if current_time > self.start_time else 0
        
        # Panel principal mejorado
        panel_width = 500
        panel_height = 180  # MODIFICADO: Aumentado para incluir info de publicación
        info_bg_color = (25, 25, 35)
        border_color = (0, 255, 255)
        
        # Fondo con transparencia
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (10 + panel_width, 10 + panel_height), info_bg_color, -1)
        cv2.addWeighted(overlay, 0.85, frame, 0.15, 0, frame)
        
        # Borde decorativo
        cv2.rectangle(frame, (10, 10), (10 + panel_width, 10 + panel_height), border_color, 3)
        cv2.rectangle(frame, (12, 12), (8 + panel_width, 8 + panel_height), border_color, 1)
        
        # Título principal
        title_text = "PUZZLEBOT TRAFFIC DETECTOR v2.0"
        cv2.putText(frame, title_text, (20, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, border_color, 2)
        
        # Línea separadora
        cv2.line(frame, (20, 50), (panel_width - 10, 50), border_color, 2)
        
        # Información del sistema en dos columnas
        info_left = [
            f"FPS: {fps:.1f}",
            f"Frame: {self.frame_count:,}",
            f"Detecciones: {len(detections)}"
        ]
        
        info_right = [
            f"Total: {self.total_detections:,}",
            f"Tipos: {len(self.unique_signs_detected)}/7",  # MODIFICADO: 7 en lugar de 8
            f"Modelo: bestLALO.pt"
        ]
        
        # Columna izquierda
        for i, line in enumerate(info_left):
            y_pos = 75 + i * 22
            cv2.putText(frame, line, (25, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        
        # Columna derecha
        for i, line in enumerate(info_right):
            y_pos = 75 + i * 22
            cv2.putText(frame, line, (280, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        
        # Estado del modelo
        model_status = "✅ ACTIVO" if self.model else "❌ ERROR"
        cv2.putText(frame, f"Estado: {model_status}", (25, 150), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0) if self.model else (0, 0, 255), 1)
        
        # AGREGADO: Estado de publicación
        if self.last_published_class:
            pub_text = f"📡 Publicando: {self.last_published_class}"
            cv2.putText(frame, pub_text, (280, 150), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 1)
        
        # Panel de detecciones activas
        if detections:
            # Posición del panel inferior
            panel_bottom_height = min(120, 30 + len(detections) * 25)
            y_start = h - panel_bottom_height - 10
            
            # Fondo del panel inferior
            overlay2 = frame.copy()
            cv2.rectangle(overlay2, (10, y_start), (w - 10, h - 10), (20, 30, 20), -1)
            cv2.addWeighted(overlay2, 0.85, frame, 0.15, 0, frame)
            cv2.rectangle(frame, (10, y_start), (w - 10, h - 10), (0, 255, 0), 2)
            
            # Título del panel
            cv2.putText(frame, "🎯 DETECCIONES ACTIVAS", (20, y_start + 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Mostrar hasta 3 detecciones más relevantes (mayor confianza)
            sorted_detections = sorted(detections, key=lambda x: x['confidence'], reverse=True)
            
            for i, det in enumerate(sorted_detections[:3]):
                y_text = y_start + 50 + i * 25
                confidence_color = det['color']
                
                # Información completa de la detección
                det_text = f"{det['emoji']} {det['class_name']} | Conf: {det['confidence']:.3f} | Área: {det['area']:,}px²"
                
                # Texto con contorno
                cv2.putText(frame, det_text, (20, y_text), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(frame, det_text, (20, y_text), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, confidence_color, 2)

    def image_callback(self, msg):
        """Callback principal optimizado"""
        try:
            # Convertir mensaje ROS a OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_count += 1
            
            # Ejecutar detección
            detections = self.process_detections(frame)
            
            # Dibujar todas las detecciones
            for detection in detections:
                self.draw_detection(frame, detection)
            
            # Añadir información overlay
            self.add_info_overlay(frame, detections)
            
            # Publicar imagen procesada
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.debug_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f"Error publicando imagen: {str(e)}")
                
        except Exception as e:
            self.get_logger().error(f"Error en callback: {str(e)}")

    def print_final_stats(self):
        """Imprime estadísticas finales detalladas"""
        elapsed = time.time() - self.start_time
        print("\n" + "=" * 80)
        print("\033[1;36m📊 ESTADÍSTICAS FINALES DEL DETECTOR\033[0m")
        print("=" * 80)
        print(f"\033[1mTiempo total de operación\033[0m: {elapsed:.2f} segundos")
        print(f"\033[1mFrames procesados\033[0m: {self.frame_count:,}")
        print(f"\033[1mVelocidad promedio\033[0m: {self.frame_count / elapsed if elapsed > 0 else 0:.2f} FPS")
        print(f"\033[1mDetecciones totales\033[0m: {self.total_detections:,}")
        print(f"\033[1mTipos únicos detectados\033[0m: {len(self.unique_signs_detected)}/7")  # MODIFICADO: 7 en lugar de 8
        
        print(f"\033[1mEficiencia de detección\033[0m: {self.total_detections / self.frame_count * 100 if self.frame_count > 0 else 0:.2f}% (detecciones por frame)")
        
        if self.unique_signs_detected:
            print(f"\n\033[1m🎯 SEÑALES DETECTADAS CON ÉXITO:\033[0m")
            for class_name in sorted(self.unique_signs_detected):
                emoji = self.sign_emojis.get(class_name, "🚦")
                count = self.detection_count_per_class[class_name]
                percentage = (count / self.total_detections * 100) if self.total_detections > 0 else 0
                print(f"    {emoji} {class_name:12} - {count:4,} detecciones ({percentage:5.1f}%)")
        else:
            print(f"\n\033[1;33m⚠️ No se detectaron señales durante la sesión\033[0m")
        
        # Clases no detectadas
        not_detected = set(self.traffic_classes.values()) - self.unique_signs_detected
        if not_detected:
            print(f"\n\033[1;90m📝 SEÑALES NO DETECTADAS EN ESTA SESIÓN:\033[0m")
            for class_name in sorted(not_detected):
                emoji = self.sign_emojis.get(class_name, "🚦")
                print(f"    {emoji} {class_name}")
        
        print("\n" + "=" * 80)
        print("\033[1;32m✅ Sesión completada exitosamente\033[0m")
        print("=" * 80)


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleTrafficSignDetector()
    
    try:
        print("\n" + "🚀" * 20)
        print("\033[1;32m[INFO] ✅ SISTEMA INICIADO CORRECTAMENTE!\033[0m")
        print("\033[1;32m[INFO] 📹 Conectado a: /puzzlebot/usb_camera/image_raw\033[0m")
        print(f"\033[1;32m[INFO] 🎯 Modelo activo: bestLALO.pt (7 clases - sin Semáforo)\033[0m")  # MODIFICADO
        print("\033[1;32m[INFO] 📺 Visualización: rqt_image_view -> /debug_image\033[0m")
        print("\033[1;32m[INFO] 📡 Publicando clases en: /detected_traffic_sign\033[0m")  # AGREGADO
        print("\033[1;32m[INFO] 🔍 Monitoreo iniciado - Las detecciones aparecerán abajo:\033[0m")
        print("🚀" * 20 + "\n")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n\033[1;33m[INFO] 🛑 Sistema detenido por el usuario\033[0m")
    except Exception as e:
        print(f"\n\033[1;31m[ERROR] ❌ Error durante la ejecución: {str(e)}\033[0m")
    finally:
        print("\033[1;36m[INFO] 🔄 Cerrando sistema y generando reporte...\033[0m")
        node.print_final_stats()
        node.destroy_node()
        rclpy.shutdown()
        print("\033[1;36m[INFO] 👋 ¡Hasta la próxima!\033[0m")


if __name__ == '__main__':
    main()