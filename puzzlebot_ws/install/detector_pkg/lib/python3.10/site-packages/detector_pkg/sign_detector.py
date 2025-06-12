import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
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
            CompressedImage, 
            '/puzzlebot/usb_camera/image_raw/compressed', 
            self.image_callback, 
            qos_profile
        )
        
        # Publicar imagen con detecciones para visualización    
        self.debug_pub = self.create_publisher(CompressedImage, '/debug_image_sign/compressed', 10)
        
        # Publicador para enviar el nombre de la clase detectada
        self.class_pub = self.create_publisher(String, '/detected_traffic_sign', 10)
        
        self.bridge = CvBridge()

        # Ruta del modelo
        self.model_path = '/home/serch/puzzlebot_ws/src/detector_pkg/detector_pkg/bestLALO.pt'
        
        # Inicializar modelo
        self.model = None
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
        
        # MAPEO DE CLASES SIN SEMAFORO (clase 4 eliminada)
        self.traffic_classes = {
            0: "Forward",
            1: "GiveWay", 
            2: "Right",
            3: "Roundabout",
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
        
        # EMOJIS SIN SEMAFORO
        self.sign_emojis = {
            "Forward": "⬆️",
            "GiveWay": "⚠️",
            "Right": "➡️", 
            "Roundabout": "🔄",
            "Stop": "🛑",
            "construction": "🚧",
            "left": "⬅️"
        }
        
        # COLORES SIN LA CLASE 4
        self.class_colors = {
            0: (0, 255, 0),      # Forward - Verde
            1: (0, 165, 255),    # GiveWay - Naranja
            2: (255, 0, 0),      # Right - Azul
            3: (255, 255, 0),    # Roundabout - Cian
            5: (0, 0, 255),      # Stop - Rojo
            6: (0, 140, 255),    # Construction - Naranja oscuro
            7: (255, 0, 255)     # Left - Magenta
        }
        
        # Estadísticas
        self.total_detections = 0
        self.unique_signs_detected = set()
        self.detection_count_per_class = {name: 0 for name in self.traffic_classes.values()}
        
        # Control para publicación de clases
        self.last_published_class = None
        self.last_publish_time = 0
        self.publish_cooldown = 1.0
        
        # Contador de imágenes publicadas para debug
        self.published_images = 0
        
        # Imprimir información inicial
        self._print_startup_info()

    def load_model(self):
        """Carga y verifica el modelo"""
        try:
            if os.path.exists(self.model_path):
                self.model = YOLO(self.model_path)
                self.get_logger().info(f"✅ Modelo '{self.model_path}' cargado correctamente")
                
                # Verificar información del modelo
                if hasattr(self.model.model, 'names'):
                    model_classes = self.model.model.names
                    self.get_logger().info(f"📋 Clases del modelo: {model_classes}")
                    
                    # Actualizar mapeo de clases si es necesario
                    for class_id, class_name in model_classes.items():
                        if class_id in self.traffic_classes:
                            if self.traffic_classes[class_id] != class_name:
                                self.get_logger().info(f"🔄 Actualizando clase {class_id}: {self.traffic_classes[class_id]} -> {class_name}")
                                # Actualizar también el diccionario de estadísticas
                                old_name = self.traffic_classes[class_id]
                                if old_name in self.detection_count_per_class:
                                    del self.detection_count_per_class[old_name]
                                self.traffic_classes[class_id] = class_name
                                self.detection_count_per_class[class_name] = 0
                        
            else:
                self.get_logger().error(f"❌ Modelo '{self.model_path}' no encontrado")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error cargando modelo: {str(e)}")

    def _print_startup_info(self):
        """Imprime información de inicio"""
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
            color_rgb = self.class_colors.get(class_id, (255, 255, 255))
            print(f"  {class_id:2d}: {emoji} {name:12} (Color: RGB{color_rgb})")
        print("\033[1;33m  ⚠️  Nota: Clase 4 (Semaforo) ha sido deshabilitada\033[0m")
        print("-" * 80)
        print("\033[1m📡 PUBLICANDO CLASES DETECTADAS EN: /detected_traffic_sign\033[0m")
        print("-" * 80)
        print("\033[1m🔍 MONITOREANDO DETECCIONES EN TIEMPO REAL:\033[0m")
        print("-" * 80)
        sys.stdout.flush()

    def detect_traffic_signs(self, frame):
        """Ejecuta detección con el modelo"""
        detections = []
        
        if self.model is None:
            return detections
            
        try:
            # Ejecutar inferencia
            results = self.model(
                frame, 
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                verbose=False,
                imgsz=640,
                device='cpu'
            )
            
            for result in results:
                boxes = result.boxes
                if boxes is not None and len(boxes) > 0:
                    xyxy = boxes.xyxy.cpu().numpy()
                    conf = boxes.conf.cpu().numpy()
                    cls = boxes.cls.cpu().numpy()
                    
                    for i in range(len(xyxy)):
                        class_id = int(cls[i])
                        
                        # Ignorar clase 4 (Semaforo) y solo procesar clases conocidas
                        if class_id == 4 or class_id not in self.traffic_classes:
                            continue
                        
                        x1, y1, x2, y2 = xyxy[i].astype(int)
                        confidence = float(conf[i])
                        
                        # Aplicar filtros de calidad
                        if self.apply_quality_filters(x1, y1, x2, y2, frame):
                            class_name = self.traffic_classes[class_id]
                            emoji = self.sign_emojis.get(class_name, "🚦")
                            color = self.class_colors.get(class_id, (255, 255, 255))
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
        """Aplica filtros de calidad para detecciones"""
        try:
            # Verificar que las coordenadas sean válidas
            if x1 >= x2 or y1 >= y2:
                return False
            
            # Verificar límites del frame
            frame_h, frame_w = frame.shape[:2]
            if x1 < 0 or y1 < 0 or x2 >= frame_w or y2 >= frame_h:
                return False
            
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
            margin = 10
            if (x1 < margin or y1 < margin or 
                x2 > frame_w - margin or y2 > frame_h - margin):
                return False
            
            # Filtro de contenido de la región
            roi = frame[y1:y2, x1:x2]
            if roi.size > 0:
                # Verificar que no sea una región completamente oscura o clara
                mean_intensity = np.mean(roi)
                if mean_intensity < 15 or mean_intensity > 240:
                    return False
                
                # Verificar varianza (evitar regiones uniformes)
                variance = np.var(roi)
                if variance < 100:
                    return False
                    
        except Exception as e:
            self.get_logger().debug(f"Error en filtros de calidad: {str(e)}")
            return False
        
        return True

    def publish_detected_class(self, class_name):
        """Publica el nombre de la clase detectada"""
        current_time = time.time()
        
        # Verificar si debemos publicar (cooldown y cambio de clase)
        should_publish = (
            class_name != self.last_published_class or 
            current_time - self.last_publish_time > self.publish_cooldown
        )
        
        if should_publish:
            try:
                msg = String()
                msg.data = class_name
                self.class_pub.publish(msg)
                
                # Actualizar estado de publicación
                self.last_published_class = class_name
                self.last_publish_time = current_time
                
                # Log en terminal
                print(f"\n📡 \033[1;35m[PUBLICADO]\033[0m: {class_name} → /detected_traffic_sign")
                
            except Exception as e:
                self.get_logger().error(f"Error publicando clase: {str(e)}")

    def process_detections(self, frame):
        """Procesa las detecciones y aplica filtros de estabilidad"""
        all_detections = self.detect_traffic_signs(frame)
        
        stable_detections = []
        current_time = time.time()
        
        # Para rastrear la detección más confiable
        best_detection = None
        best_confidence = 0
        
        for detection in all_detections:
            x1, y1, x2, y2 = detection['bbox']
            class_name = detection['class_name']
            confidence = detection['confidence']
            
            # Sistema de estabilidad basado en regiones
            region_size = 80
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
                
                # Actualizar mejor detección
                if confidence > best_confidence:
                    best_detection = detection
                    best_confidence = confidence
                
                # Sistema de cooldown para terminal
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
                    
                    # Estimación de distancia
                    area = detection['area']
                    if area > 25000:
                        distance_est = "MUY CERCA"
                        distance_color = "\033[1;31m"
                    elif area > 12000:
                        distance_est = "CERCA"
                        distance_color = "\033[1;33m"
                    elif area > 6000:
                        distance_est = "MEDIA"
                        distance_color = "\033[1;36m"
                    elif area > 2000:
                        distance_est = "LEJOS"
                        distance_color = "\033[1;37m"
                    else:
                        distance_est = "MUY LEJOS"
                        distance_color = "\033[1;90m"
                    
                    # Imprimir detección
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
                    if class_name in self.detection_count_per_class:
                        self.detection_count_per_class[class_name] += 1
                
                stable_detections.append(detection)
        
        # Publicar la mejor detección
        if best_detection:
            self.publish_detected_class(best_detection['class_name'])
        
        return stable_detections

    def draw_detection(self, frame, detection):
        """Dibuja las detecciones con estilo limpio"""
        x1, y1, x2, y2 = detection['bbox']
        class_name = detection['class_name']
        emoji = detection['emoji']
        confidence = detection['confidence']
        color = detection['color']
        
        # Cuadro principal
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        
        # Etiqueta con fondo
        label = f"{emoji} {class_name}"
        font_scale = 0.7
        thickness = 2
        
        # Tamaño del texto
        (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
        
        # Fondo de la etiqueta
        cv2.rectangle(frame, (x1, y1 - text_height - 10), (x1 + text_width + 10, y1), color, -1)
        
        # Texto
        cv2.putText(frame, label, (x1 + 5, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)
        
        # Confianza
        conf_text = f"{confidence:.2f}"
        cv2.putText(frame, conf_text, (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # Punto central
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        cv2.circle(frame, (center_x, center_y), 3, (0, 255, 0), -1)

    def add_info_overlay(self, frame, detections):
        """Añade información del sistema"""
        h, w = frame.shape[:2]
        
        # Estadísticas
        current_time = time.time()
        elapsed = current_time - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        
        # Panel superior
        panel_height = 100
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (400, panel_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # Información básica
        info_lines = [
            f"FPS: {fps:.1f}",
            f"Detecciones: {len(detections)}",
            f"Total: {self.total_detections}"
        ]
        
        for i, line in enumerate(info_lines):
            y_pos = 35 + i * 20
            cv2.putText(frame, line, (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Estado de publicación
        if self.last_published_class:
            pub_text = f"Publicando: {self.last_published_class}"
            cv2.putText(frame, pub_text, (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Panel inferior para detecciones activas
        if detections:
            # Mostrar la mejor detección
            best_detection = max(detections, key=lambda x: x['confidence'])
            
            # Panel inferior
            panel_y = h - 50
            overlay2 = frame.copy()
            cv2.rectangle(overlay2, (10, panel_y), (w - 10, h - 10), (0, 0, 0), -1)
            cv2.addWeighted(overlay2, 0.7, frame, 0.3, 0, frame)
            
            # Información de la mejor detección
            det_text = f"MEJOR: {best_detection['emoji']} {best_detection['class_name']} ({best_detection['confidence']:.3f})"
            cv2.putText(frame, det_text, (20, panel_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, best_detection['color'], 2)

    def image_callback(self, msg):
        """Callback principal"""
        try:
            # Convertir mensaje ROS a OpenCV
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_count += 1
            
            # Ejecutar detección
            detections = self.process_detections(frame)
            
            # Dibujar detecciones
            for detection in detections:
                self.draw_detection(frame, detection)
            
            # Añadir información overlay
            self.add_info_overlay(frame, detections)
            
            # Publicar imagen procesada
            try:
                # Configurar calidad de compresión para mejor visualización
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                _, buffer = cv2.imencode('.jpg', frame, encode_param)
                
                debug_msg = CompressedImage()
                debug_msg.header.stamp = self.get_clock().now().to_msg()
                debug_msg.header.frame_id = "camera_frame"
                debug_msg.format = "jpeg"
                debug_msg.data = buffer.tobytes()
                
                self.debug_pub.publish(debug_msg)
                self.published_images += 1
                
                # Log cada 100 imágenes publicadas
                if self.published_images % 100 == 0:
                    self.get_logger().info(f"📸 Imágenes publicadas: {self.published_images}")
                
            except Exception as e:
                self.get_logger().error(f"Error publicando imagen: {str(e)}")
                
        except Exception as e:
            self.get_logger().error(f"Error en callback: {str(e)}")

    def print_final_stats(self):
        """Imprime estadísticas finales"""
        elapsed = time.time() - self.start_time
        print("\n" + "=" * 80)
        print("\033[1;36m📊 ESTADÍSTICAS FINALES DEL DETECTOR\033[0m")
        print("=" * 80)
        print(f"\033[1mTiempo total de operación\033[0m: {elapsed:.2f} segundos")
        print(f"\033[1mFrames procesados\033[0m: {self.frame_count:,}")

        print(f"\033[1mDetecciones totales\033[0m: {self.total_detections:,}")
        print(f"\033[1mTipos únicos detectados\033[0m: {len(self.unique_signs_detected)}/7")
        
        efficiency = (self.total_detections / self.frame_count * 100) if self.frame_count > 0 else 0
        print(f"\033[1mEficiencia de detección\033[0m: {efficiency:.2f}% (detecciones por frame)")
        
        if self.unique_signs_detected:
            print(f"\n\033[1m🎯 SEÑALES DETECTADAS CON ÉXITO:\033[0m")
            for class_name in sorted(self.unique_signs_detected):
                emoji = self.sign_emojis.get(class_name, "🚦")
                count = self.detection_count_per_class.get(class_name, 0)
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
        print("\033[1;32m[INFO] 📹 Conectado a: /puzzlebot/usb_camera/image_raw/compressed\033[0m")
        print("\033[1;32m[INFO] 🎯 Modelo activo: bestLALO.pt (7 clases - sin Semáforo)\033[0m")
        print("\033[1;32m[INFO] 📺 Visualización: rqt_image_view -> /debug_image\033[0m")
        print("\033[1;32m[INFO] 📡 Publicando clases en: /detected_traffic_sign\033[0m")
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