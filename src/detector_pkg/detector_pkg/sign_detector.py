import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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
        self.debug_pub = self.create_publisher(Image, '/debug_image', 10)
        self.bridge = CvBridge()

        # MODELO ÚNICO - Actualizar ruta si es necesario
        self.model_path = '/home/alfonso/puzzlebot-ros2-control/src/detector_pkg/detector_pkg/best.pt'
        
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
        
        # MAPEO DE CLASES ACTUALIZADO según tu modelo
        self.traffic_classes = {
            0: "Forward",
            1: "GiveWay", 
            2: "Right",
            3: "Roundabout",
            4: "Semaforo",
            5: "Stop",
            6: "construction",
            7: "left"
        }
        
        # Sistema de filtrado
        self.noise_filter_enabled = True
        self.min_detection_size = 800
        self.max_detection_size = 100000
        self.aspect_ratio_min = 0.4
        self.aspect_ratio_max = 2.5
        
        # Emojis actualizados para cada señal de tu modelo
        self.sign_emojis = {
            "Forward": "⬆️",
            "GiveWay": "⚠️",
            "Right": "↪️", 
            "Roundabout": "🔄",
            "Semaforo": "🚦",
            "Stop": "🛑",
            "construction": "🚧",
            "left": "↩️"
        }
        
        # Colores para cada clase
        self.class_colors = {}
        np.random.seed(42)
        for class_id in self.traffic_classes.keys():
            self.class_colors[class_id] = tuple(map(int, np.random.randint(50, 255, 3)))
        
        # Estadísticas
        self.total_detections = 0
        self.unique_signs_detected = set()
        
        # Imprimir información inicial
        self._print_startup_info()

    def load_model(self):
        """Carga el modelo único"""
        self.model = None
        
        try:
            if os.path.exists(self.model_path):
                self.model = YOLO(self.model_path)
                self.get_logger().info(f"✅ Modelo '{self.model_path}' cargado correctamente")
                
                # Obtener información del modelo
                if hasattr(self.model.model, 'names'):
                    model_classes = self.model.model.names
                    self.get_logger().info(f"📋 Clases del modelo: {model_classes}")
                    
                    # Verificar si las clases coinciden y actualizar si es necesario
                    if model_classes != self.traffic_classes:
                        self.get_logger().info("🔄 Actualizando clases basado en el modelo real...")
                        self.traffic_classes = {}
                        for class_id, class_name in model_classes.items():
                            self.traffic_classes[class_id] = class_name
                        
                        # Actualizar emojis para nuevas clases si las hay
                        for class_name in self.traffic_classes.values():
                            if class_name not in self.sign_emojis:
                                self.sign_emojis[class_name] = "🚦"  # Emoji por defecto
                                
            else:
                self.get_logger().error(f"❌ Modelo '{self.model_path}' no encontrado")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error cargando modelo: {str(e)}")

    def _print_startup_info(self):
        """Imprime información de inicio"""
        print("\n" + "=" * 70)
        print(" " * 15 + "\033[1;36m🚗 PUZZLEBOT TRAFFIC DETECTOR\033[0m")
        print(" " * 20 + "\033[1;33m⚡ MODELO ACTUALIZADO ⚡\033[0m")
        print("=" * 70)
        print(f"\033[1mIniciado\033[0m: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"\033[1mModelo\033[0m: {self.model_path}")
        print(f"  └─ Estado: {'✅ Cargado' if self.model else '❌ Error'}")
        print(f"  └─ Confianza: {self.confidence_threshold}")
        print("-" * 70)
        print("\033[1m🎯 CLASES A DETECTAR:\033[0m")
        for class_id, name in self.traffic_classes.items():
            emoji = self.sign_emojis.get(name, "🚦")
            print(f"  {class_id:2d}: {emoji} {name}")
        print("-" * 70)
        print("\033[1m🔍 DETECCIONES EN TIEMPO REAL:\033[0m")
        print("-" * 70)
        sys.stdout.flush()

    def detect_traffic_signs(self, frame):
        """Ejecuta detección con el modelo único"""
        detections = []
        
        if self.model is None:
            return detections
            
        try:
            results = self.model(
                frame, 
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                verbose=False,
                imgsz=640
            )
            
            for result in results:
                boxes = result.boxes
                if boxes is not None and len(boxes) > 0:
                    xyxy = boxes.xyxy.cpu().numpy()
                    conf = boxes.conf.cpu().numpy()
                    cls = boxes.cls.cpu().numpy()
                    
                    for i in range(len(xyxy)):
                        class_id = int(cls[i])
                        
                        # Solo procesar clases conocidas
                        if class_id not in self.traffic_classes:
                            continue
                        
                        x1, y1, x2, y2 = xyxy[i].astype(int)
                        confidence = float(conf[i])
                        
                        # Aplicar filtros de calidad
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
                                'area': area
                            })
                            
        except Exception as e:
            self.get_logger().error(f"Error en detección: {str(e)}")
            
        return detections

    def apply_quality_filters(self, x1, y1, x2, y2, frame):
        """Aplica filtros de calidad para detecciones"""
        # Filtro de área
        area = (x2 - x1) * (y2 - y1)
        if area < self.min_detection_size or area > self.max_detection_size:
            return False
        
        # Filtro de aspecto ratio
        width = x2 - x1
        height = y2 - y1
        aspect_ratio = width / height if height > 0 else 1
        if aspect_ratio < self.aspect_ratio_min or aspect_ratio > self.aspect_ratio_max:
            return False
        
        # Filtro de posición
        frame_h, frame_w = frame.shape[:2]
        if x1 < 5 or y1 < 5 or x2 > frame_w - 5 or y2 > frame_h - 5:
            return False
        
        # Filtro de densidad
        roi = frame[y1:y2, x1:x2]
        if roi.size > 0:
            density = np.mean(roi)
            if density < 10 or density > 250:
                return False
        
        return True

    def process_detections(self, frame):
        """Procesa las detecciones y aplica filtros de estabilidad"""
        all_detections = self.detect_traffic_signs(frame)
        
        stable_detections = []
        current_time = time.time()
        
        for detection in all_detections:
            x1, y1, x2, y2 = detection['bbox']
            class_name = detection['class_name']
            confidence = detection['confidence']
            
            # Sistema de estabilidad
            region_key = f"{int(x1/100)}_{int(y1/100)}"
            detection_key = f"{class_name}_{region_key}"
            
            if detection_key not in self.detection_history:
                self.detection_history[detection_key] = []
            
            self.detection_history[detection_key].append({
                'time': current_time,
                'confidence': confidence
            })
            
            # Limpiar historial antiguo
            self.detection_history[detection_key] = [
                det for det in self.detection_history[detection_key] 
                if current_time - det['time'] < 3.0
            ]
            
            # Verificar estabilidad
            stable_count = len(self.detection_history[detection_key])
            if stable_count >= self.stability_threshold:
                
                # Sistema de cooldown para terminal
                show_in_terminal = (detection_key not in self.last_detected_signs or 
                                  current_time - self.last_detected_signs[detection_key] > self.detection_cooldown)
                
                if show_in_terminal:
                    # Indicadores
                    stability_indicator = "🎯" if stable_count >= 5 else "⚡"
                    
                    # Estimación de distancia
                    area = detection['area']
                    if area > 20000:
                        distance_est = "MUY CERCA"
                    elif area > 8000:
                        distance_est = "CERCA"
                    elif area > 3000:
                        distance_est = "MEDIA"
                    elif area > 1500:
                        distance_est = "LEJOS"
                    else:
                        distance_est = "MUY LEJOS"
                    
                    print(f"{detection['emoji']} {stability_indicator} "
                          f"DETECTADO: {class_name} | "
                          f"Conf: {confidence:.2f} | Área: {area}px² | Dist: {distance_est}")
                    
                    self.last_detected_signs[detection_key] = current_time
                    self.total_detections += 1
                    self.unique_signs_detected.add(class_name)
                
                stable_detections.append(detection)
        
        return stable_detections

    def draw_detection(self, frame, detection):
        """Dibuja las detecciones en el frame"""
        x1, y1, x2, y2 = detection['bbox']
        class_name = detection['class_name']
        emoji = detection['emoji']
        confidence = detection['confidence']
        color = detection['color']
        area = detection['area']
        
        # Grosor adaptativo
        thickness = max(2, min(5, int(area / 5000)))
        
        # Cuadro principal
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        
        # Esquinas destacadas
        corner_size = max(10, min(25, int(area / 1000)))
        cv2.line(frame, (x1, y1), (x1 + corner_size, y1), color, thickness + 1)
        cv2.line(frame, (x1, y1), (x1, y1 + corner_size), color, thickness + 1)
        cv2.line(frame, (x2, y1), (x2 - corner_size, y1), color, thickness + 1)
        cv2.line(frame, (x2, y1), (x2, y1 + corner_size), color, thickness + 1)
        cv2.line(frame, (x1, y2), (x1 + corner_size, y2), color, thickness + 1)
        cv2.line(frame, (x1, y2), (x1, y2 - corner_size), color, thickness + 1)
        cv2.line(frame, (x2, y2), (x2 - corner_size, y2), color, thickness + 1)
        cv2.line(frame, (x2, y2), (x2, y2 - corner_size), color, thickness + 1)
        
        # Tamaño de fuente adaptativo
        if area > 15000:
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
        
        # Etiqueta
        label = class_name
        label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_thickness)[0]
        
        # Fondo de la etiqueta
        overlay = frame.copy()
        cv2.rectangle(overlay, (x1, y1 - label_size[1] - 25), 
                     (x1 + label_size[0] + 20, y1), color, -1)
        frame = cv2.addWeighted(frame, 0.7, overlay, 0.3, 0)
        
        # Texto con contorno
        cv2.putText(frame, label, (x1 + 10, y1 - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), text_thickness + 1)
        cv2.putText(frame, label, (x1 + 10, y1 - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), text_thickness)
        
        # Emoji
        emoji_x = max(5, x1 - 60)
        emoji_y = y1 + int(40 * emoji_scale)
        cv2.putText(frame, emoji, (emoji_x, emoji_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, emoji_scale, (0, 0, 0), max(2, int(emoji_scale * 3)))
        cv2.putText(frame, emoji, (emoji_x, emoji_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, emoji_scale, color, max(1, int(emoji_scale * 2)))
        
        # Información adicional
        conf_text = f"{confidence:.2f}"
        cv2.putText(frame, conf_text, (x1, y2 + 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.8, (0, 0, 0), text_thickness + 1)
        cv2.putText(frame, conf_text, (x1, y2 + 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.8, color, text_thickness)
        
        # Punto central
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        cv2.circle(frame, (center_x, center_y), max(3, int(area / 8000)), (0, 255, 0), -1)
        cv2.circle(frame, (center_x, center_y), max(5, int(area / 6000)), (255, 255, 255), 2)

    def add_info_overlay(self, frame, detections):
        """Añade información del sistema"""
        h, w = frame.shape[:2]
        
        # Estadísticas
        current_time = time.time()
        fps = self.frame_count / (current_time - self.start_time) if current_time > self.start_time else 0
        
        # Panel principal
        info_bg_color = (30, 30, 30)
        cv2.rectangle(frame, (10, 10), (450, 140), info_bg_color, -1)
        cv2.rectangle(frame, (10, 10), (450, 140), (0, 255, 255), 2)
        
        # Título
        cv2.putText(frame, "TRAFFIC SIGN DETECTOR", (20, 35), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # Información del sistema
        info_lines = [
            f"FPS: {fps:.1f} | Frame: {self.frame_count}",
            f"Detecciones actuales: {len(detections)}",
            f"Total acumulado: {self.total_detections}",
            f"Tipos unicos: {len(self.unique_signs_detected)}"
        ]
        
        for i, line in enumerate(info_lines):
            y_pos = 60 + i * 18
            cv2.putText(frame, line, (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Lista de detecciones actuales
        if detections:
            y_start = h - 100
            cv2.rectangle(frame, (10, y_start - 15), (w - 10, h - 10), (20, 20, 20), -1)
            cv2.rectangle(frame, (10, y_start - 15), (w - 10, h - 10), (0, 255, 0), 2)
            
            cv2.putText(frame, "DETECCIONES ACTIVAS:", (20, y_start), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Mostrar hasta 3 detecciones
            for i, det in enumerate(detections[:3]):
                text = f"{det['emoji']} {det['class_name']} ({det['confidence']:.2f})"
                cv2.putText(frame, text, (20, y_start + 20 + i * 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, det['color'], 2)

    def image_callback(self, msg):
        """Callback principal"""
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
        """Imprime estadísticas finales"""
        elapsed = time.time() - self.start_time
        print("\n" + "=" * 70)
        print("\033[1;36m📊 ESTADÍSTICAS FINALES\033[0m")
        print("=" * 70)
        print(f"\033[1mTiempo total\033[0m: {elapsed:.2f} segundos")
        print(f"\033[1mFrames procesados\033[0m: {self.frame_count}")
        print(f"\033[1mFPS promedio\033[0m: {self.frame_count / elapsed if elapsed > 0 else 0:.2f}")
        print(f"\033[1mDetecciones totales\033[0m: {self.total_detections}")
        print(f"\033[1mTipos únicos detectados\033[0m: {len(self.unique_signs_detected)}")
        
        if self.unique_signs_detected:
            print(f"\033[1mSeñales encontradas\033[0m:")
            for sign_name in sorted(self.unique_signs_detected):
                emoji = self.sign_emojis.get(sign_name, "🚦")
                print(f"    {emoji} {sign_name}")
        
        print("=" * 70)


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleTrafficSignDetector()
    
    try:
        print("\033[1;32m[INFO] ✅ Sistema iniciado correctamente!\033[0m")
        print("\033[1;32m[INFO] 📹 Conectado a: /puzzlebot/usb_camera/image_raw\033[0m")
        print(f"\033[1;32m[INFO] 🎯 Usando modelo actualizado con 8 clases\033[0m")
        print("\033[1;32m[INFO] 📺 Ver detecciones en: rqt_image_view -> /debug_image\033[0m")
        print("\033[1;32m[INFO] 🔍 Detecciones aparecerán aquí:\033[0m\n")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n\033[1;33m[INFO] 🛑 Sistema detenido por usuario\033[0m")
    except Exception as e:
        print(f"\n\033[1;31m[ERROR] Error durante ejecución: {str(e)}\033[0m")
    finally:
        print("\033[1;36m[INFO] 🔄 Cerrando sistema...\033[0m")
        node.print_final_stats()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()