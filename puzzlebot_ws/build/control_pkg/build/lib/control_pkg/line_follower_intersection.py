import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from simple_pid import PID
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import sys


class FollowLineNode(Node):
    def __init__(self):
        super().__init__('follow_line_node')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Subscription for compressed images
        self.subscription = self.create_subscription(
            CompressedImage, 
            '/puzzlebot/camera/image_raw/compressed', 
            self.image_callback, 
            qos_profile
        )
        # Debug publisher for compressed images
        self.debug_pub = self.create_publisher(CompressedImage, '/debug_image/compressed', 10)
        self.intersection_pub = self.create_publisher(String, '/intersection', 10)

        # Navigation enable/disable subscription
        self.navigation_enabled = True  # Default to enabled
        self.navigation_enable_sub = self.create_subscription(
            Bool,
            '/navigation_enable',
            self.navigation_enable_callback,
            10
        )

        self.vel_sub = self.create_subscription(
            Twist,
            '/max_vel',
            self.velocity_callback , # Reusing the same callback for simplicity
            10
        )

        self.bridge = CvBridge()

        # Camera resizing parameters
        self.camera_height = 280
        self.camera_width = 360

        # PID controller
        max_yaw = math.radians(45)
        self.max_thr = 0.14
        self.yaw_pid = PID(Kp=0.458, Ki=0.002, Kd=0.08, setpoint=0.0, output_limits=(-max_yaw, max_yaw))

        # Factores de ponderación para la jerarquía de decisión
        self.center_weight = 0.7
        self.angle_weight = 0.3
        
        # Parámetros para el movimiento de reversa durante búsqueda
        self.line_lost_count = 0
        self.max_line_lost_count = 10
        self.backup_speed = -0.1
        self.last_yaw = 0.0
        self.in_search_mode = False
        self.search_yaw_options = [-0.1, 0.0, 0.1]
        self.search_yaw_index = 1
        self.search_time_counter = 0
        self.search_interval = 20
        
        # ============= PARÁMETROS PARA DETECCIÓN DE LÍNEAS SEGMENTADAS =============
        self.intersection_detected = False
        self.intersection_stopped = False
        self.line_ended = False
        
        # NUEVA VARIABLE: Control de publicación única
        self.intersection_message_published = False
        
        # Detección específica de líneas segmentadas horizontales
        self.segmented_line_detection_count = 0
        self.segmented_line_threshold = 1  # CAMBIADO: Solo 1 frame para parar inmediatamente
        
        # Parámetros para detectar segmentos rectangulares
        self.min_segment_area = 125  # Área mínima de cada segmento rectangular
        self.max_segment_area = 1800  # Área máxima de cada segmento
        self.min_segments_in_line = 2  # Mínimo número de segmentos para considerar línea
        self.max_segment_gap = 35  # Distancia máxima entre segmentos consecutivos
        self.horizontal_tolerance = 30  # Tolerancia en píxeles para considerar segmentos "horizontales"
        
        # Parámetros específicos para rectangulos
        self.min_segment_width = 15  # Ancho mínimo del segmento
        self.max_segment_width = 80  # Ancho máximo del segmento
        self.min_segment_height = 8  # Alto mínimo del segmento
        self.max_segment_height = 25  # Alto máximo del segmento
        self.aspect_ratio_min = 2  # Ratio mínimo ancho/alto (rectangulos horizontales)
        self.aspect_ratio_max = 8.0  # Ratio máximo ancho/alto
        
        # ROI para detección de líneas segmentadas
        self.segmented_roi_start = 0.35  # Desde 35% hacia abajo
        self.segmented_roi_end = 0.95  # Hasta 95% hacia abajo
        
        # Control general
        self.intersection_cooldown = 0
        self.intersection_cooldown_max = 80
        self.final_stop_triggered = False
        
        # Variables para estadísticas y terminal
        self.start_time = time.time()
        self.frame_count = 0
        self.last_status_print = 0
        self.status_interval = 1.0
        
        self.prev_cx = None
        self.prev_cy = None
        self.prev_area = None
        self.lost_count = 0
        self.max_lost_memory = 5  # Number of frames to keep memory
        
        self._print_header()
    
    def velocity_callback(self, msg):
        """Callback para recibir la velocidad máxima del robot"""
        self.max_thr = msg.data
        self.get_logger().info(f"Velocidad máxima actualizada: {self.max_thr} m/s")

    def _print_header(self):
        """Imprime un encabezado atractivo en la terminal"""
        terminal_width = 80
        print("\n" + "=" * terminal_width)
        print(" " * 15 + "\033[1;36mPUZZLEBOT SEGMENTED LINE DETECTOR\033[0m")
        print(" " * 10 + "\033[1;33m🚀 DETECCIÓN DE LÍNEAS SEGMENTADAS HORIZONTALES 🚀\033[0m")
        print("=" * terminal_width)
        print(f"\033[1mLanzado\033[0m: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"\033[1mConfiguración\033[0m: PID(Kp={self.yaw_pid.Kp}, Ki={self.yaw_pid.Ki}, Kd={self.yaw_pid.Kd})")
        print(f"\033[1mVelocidad máxima\033[0m: {self.max_thr} m/s")
        print(f"\033[1mDetección\033[0m: PARADA INMEDIATA al detectar segmentos")
        print(f"\033[1mROI detección\033[0m: {self.segmented_roi_start*100:.0f}% - {self.segmented_roi_end*100:.0f}% del frame")
        print(f"\033[1mImagen\033[0m: Usando CompressedImage")
        print("-" * terminal_width)
        print("\033[1m{:<8} {:<12} {:<10} {:<10} {:<12} {:<20}\033[0m".format(
            "TIEMPO", "ESTADO", "VEL(m/s)", "YAW(rad/s)", "FPS", "INFO"))
        print("-" * terminal_width)
        sys.stdout.flush()

    def _print_status(self, state, throttle, yaw, info=""):
        """Imprime actualizaciones de estado en formato bonito"""
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        self.frame_count += 1
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        
        if current_time - self.last_status_print >= self.status_interval:
            if state == "TRACKING":
                state_formatted = f"\033[1;32m{state}\033[0m"
            elif state == "SEARCHING":
                state_formatted = f"\033[1;31m{state}\033[0m" 
            elif state == "BACKUP":
                state_formatted = f"\033[1;33m{state}\033[0m"
            elif state == "SEGMENTED_DETECTED":
                state_formatted = f"\033[1;35m{state}\033[0m"
            elif state == "FINISHED":
                state_formatted = f"\033[1;36m{state}\033[0m"
            else:
                state_formatted = state
                
            print("\033[K{:<8.1f} {:<12} {:<10.2f} {:<10.2f} {:<12.1f} {:<20}".format(
                elapsed, state_formatted, throttle, yaw, fps, info))
            sys.stdout.flush()
            
            self.last_status_print = current_time

    def detect_segmented_horizontal_lines(self, frame, mask):
        """
        Detecta líneas segmentadas horizontales (rectángulos consecutivos)
        MODIFICADO: Resetea el flag cuando no hay intersección
        """
        if self.intersection_cooldown > 0:
            return False, []
        
        frame_height, frame_width = frame.shape[:2]
        
        # Definir ROI para detectar líneas segmentadas
        roi_start = int(frame_height * self.segmented_roi_start)
        roi_end = int(frame_height * self.segmented_roi_end)
        
        # Extraer ROI
        roi_mask = mask[roi_start:roi_end, :]
        roi_frame = frame[roi_start:roi_end, :].copy()
        
        if roi_mask.size == 0:
            return False, []
        
        # Limpiar la máscara para mejor detección de segmentos rectangulares
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        clean_mask = cv2.morphologyEx(roi_mask, cv2.MORPH_CLOSE, kernel)
        clean_mask = cv2.morphologyEx(clean_mask, cv2.MORPH_OPEN, kernel)
        
        # Encontrar contornos (estos serán nuestros segmentos rectangulares)
        contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filtrar contornos por forma rectangular y tamaño
        segment_candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filtrar por área
            if self.min_segment_area <= area <= self.max_segment_area:
                # Obtener rectángulo que encierra el contorno
                x, y, w, h = cv2.boundingRect(contour)
                
                # Verificar dimensiones del rectángulo
                if (self.min_segment_width <= w <= self.max_segment_width and 
                    self.min_segment_height <= h <= self.max_segment_height):
                    
                    # Calcular aspect ratio (ancho/alto)
                    aspect_ratio = float(w) / h if h > 0 else 0
                    
                    # Solo rectángulos horizontales (más anchos que altos)
                    if self.aspect_ratio_min <= aspect_ratio <= self.aspect_ratio_max:
                        
                        # Verificar que el contorno llene suficientemente el rectángulo
                        rect_area = w * h
                        fill_ratio = area / rect_area if rect_area > 0 else 0
                        
                        if fill_ratio >= 0.6:  # Al menos 60% del rectángulo debe estar lleno
                            # Obtener centro del segmento
                            cx = x + w // 2
                            cy = y + h // 2
                            
                            segment_candidates.append({
                                'center': (cx, cy),
                                'bbox': (x, y, w, h),
                                'area': area,
                                'aspect_ratio': aspect_ratio,
                                'contour': contour,
                                'fill_ratio': fill_ratio
                            })
                            
                            # Dibujar segmento detectado
                            cv2.rectangle(roi_frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                            cv2.circle(roi_frame, (cx, cy), 3, (255, 0, 255), -1)
                            cv2.putText(roi_frame, f"{aspect_ratio:.1f}", (x, y - 5), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # Si hay al menos 3 segmentos candidatos, es intersección
        intersection_detected = len(segment_candidates) >= 3
        horizontal_segmented_lines = []  # No buscamos líneas, solo segmentos
        if not intersection_detected and self.intersection_message_published:
            self.intersection_message_published = False
            print(f"\n\033[1;33m🔄 RESET: Listo para detectar nueva intersección\033[0m")
        
        if intersection_detected:
            cv2.putText(frame, "🛑 INTERSECTION DETECTED!", 
                       (10, roi_start + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 3)
        
        # Actualizar frame original con ROI procesada
        frame[roi_start:roi_end, :] = roi_frame
        
        # Mostrar información de detección
        cv2.putText(frame, f"Segment candidates: {len(segment_candidates)}", 
                   (10, roi_start + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(frame, f"Segmented lines: {len(horizontal_segmented_lines)}", 
                   (10, roi_start + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return intersection_detected, horizontal_segmented_lines

    def publish_intersection_message(self):
        """
        NUEVA FUNCIÓN: Publica el mensaje de intersección solo una vez
        """
        if not self.intersection_message_published:
            msg = String()
            msg.data = "intersection"
            self.intersection_pub.publish(msg)
            self.intersection_message_published = True
            self.get_logger().info("🔲 Intersection message published!")
            print(f"\n\033[1;35m📢 MENSAJE DE INTERSECCIÓN PUBLICADO\033[0m")

    def detect_line_end(self, frame, contours):
        """
        Detecta si la línea central ha terminado
        """
        if len(contours) == 0:
            return True
        
        frame_height = frame.shape[0]
        all_near_bottom = True
        
        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cy = int(M["m01"] / M["m00"])
                if cy < frame_height * 0.9:
                    all_near_bottom = False
                    break
        
        return all_near_bottom and len(contours) < 2

    def navigation_enable_callback(self, msg):
        """Callback for navigation enable/disable messages"""
        self.navigation_enabled = msg.data
        status = "ENABLED" if self.navigation_enabled else "DISABLED"
        self.get_logger().info(f"Line follower {status}")
        
        # If disabled, stop the robot
        if not self.navigation_enabled:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            
            # Reset search mode and line lost count
            self.in_search_mode = False
            self.line_lost_count = 0
            self.search_time_counter = 0
            self.search_yaw_index = 1

    def image_callback(self, msg):
        try:
            # Decode compressed image properly
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.resize(frame, (self.camera_width, self.camera_height))

            # Preprocesamiento base
            dark_thres = 100
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, mask = cv2.threshold(gray, dark_thres, 255, cv2.THRESH_BINARY_INV)

            # Reducir cooldown si está activo
            if self.intersection_cooldown > 0:
                self.intersection_cooldown -= 1

            # ✅ DETECCIÓN DE INTERSECCIÓN SIEMPRE ACTIVA
            segmented_detected, segmented_lines = self.detect_segmented_horizontal_lines(frame, mask)

            if not self.navigation_enabled:
                # Publicar imagen de debug indicando que detección sigue activa
                cv2.putText(frame, "NAVIGATION DISABLED - DETECTION ACTIVE", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                debug_frame = cv2.resize(frame, (self.camera_width, self.camera_height))
                # Convert to compressed image for publishing
                debug_msg = self.bridge.cv2_to_compressed_imgmsg(debug_frame, dst_format='jpeg')
                self.debug_pub.publish(debug_msg)
                return

            # Si ya terminamos, no procesar más
            if self.final_stop_triggered:
                cv2.putText(frame, "INTERSECTION REACHED - ROBOT STOPPED!", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3)
                cv2.putText(frame, "Segmented line intersection detected", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                self._print_status("FINISHED", 0.0, 0.0, "PARADA INMEDIATA - Intersección")
                
                debug_frame = cv2.resize(frame, (self.camera_width, self.camera_height))
                # Convert to compressed image for publishing
                debug_msg = self.bridge.cv2_to_compressed_imgmsg(debug_frame, dst_format='jpeg')
                self.debug_pub.publish(debug_msg)
                return
            
            # LÓGICA PRINCIPAL: Manejar detección de intersección
            if segmented_detected:
                self.segmented_line_detection_count += 1
                self.publish_intersection_message()
                
                # PUBLICAR MENSAJE SOLO UNA VEZ cuando se detecta por primera vez

                print(f"\n\033[1;35m🔲 SEGMENTOS DETECTADOS - ROBOT DETENIDO\033[0m")
                print(f"\033[1;32m📊 Segmentos encontrados: {len(segmented_lines) if segmented_lines else 'N/A'}\033[0m")
                
                self._print_status("SEGMENTED_DETECTED", 0.0, 0.0, "¡INTERSECCIÓN! Robot detenido")
                
            else:
                self.segmented_line_detection_count = 0
                
                # Seguimiento de línea normal
                throttle, yaw, line_found = self.follow_line(frame)
                
                # Detectar fin de línea
                if not line_found and self.line_lost_count > self.max_line_lost_count:
                    gray_bottom = gray[int(frame.shape[0] * 0.7):, :]
                    _, mask_bottom = cv2.threshold(gray_bottom, dark_thres, 255, cv2.THRESH_BINARY_INV)
                    white_pixels_bottom = cv2.countNonZero(mask_bottom)
                    
                    if white_pixels_bottom < 500:
                        self.line_ended = True
                
                # Continuar con seguimiento normal
                if not line_found:
                    self.line_lost_count += 1
                    
                    if self.line_lost_count >= self.max_line_lost_count:
                        self.in_search_mode = True
                        throttle = self.backup_speed
                        
                        self.search_time_counter += 1
                        if self.search_time_counter >= self.search_interval:
                            self.search_yaw_index = (self.search_yaw_index + 1) % len(self.search_yaw_options)
                            self.search_time_counter = 0
                        
                        yaw = self.search_yaw_options[self.search_yaw_index]
                        
                        info = f"Búsqueda: {self.search_yaw_index}/{len(self.search_yaw_options)-1}"
                        self._print_status("BACKUP", throttle, yaw, info)
                        
                        cv2.putText(frame, f"Searching: Backing up (yaw={yaw:.2f})", (10, 200), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                else:
                    self.line_lost_count = 0
                    self.in_search_mode = False
                    self.search_time_counter = 0
                    self.search_yaw_index = 1
                    self.last_yaw = yaw
                    
                    self._print_status("TRACKING", throttle, yaw)

                # Publicar comando de movimiento (solo si no hemos detectado intersección)
                twist = Twist()
                twist.linear.x = float(throttle)
                twist.angular.z = float(yaw)
                self.publisher.publish(twist)

            # Mostrar información de detección en frame
            cv2.putText(frame, f"Segmented detection: {self.segmented_line_detection_count}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"Message published: {'YES' if self.intersection_message_published else 'NO'}", 
                       (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"Line lost: {self.line_lost_count}/{self.max_line_lost_count}", 
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            if segmented_detected:
                cv2.putText(frame, "🔲 SEGMENTS DETECTED - STOPPED!", 
                           (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
            
            if self.line_ended:
                cv2.putText(frame, "LINE ENDED!", 
                           (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 3)
            
            # Dibujar ROI de detección
            roi_start = int(frame.shape[0] * self.segmented_roi_start)
            roi_end = int(frame.shape[0] * self.segmented_roi_end)
            cv2.rectangle(frame, (0, roi_start), (frame.shape[1], roi_end), (255, 0, 0), 2)
            cv2.putText(frame, "SEGMENTED DETECTION ROI", (5, roi_start - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            # Publicar frame de debug como compressed image
            debug_frame = cv2.resize(frame, (self.camera_width, self.camera_height))
            # Convert to compressed image for publishing
            debug_msg = self.bridge.cv2_to_compressed_imgmsg(debug_frame, dst_format='jpeg')
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f"Error en callback: {str(e)}")
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)

    def follow_line(self, frame):
        if frame is None:
            return 0.0, 0.0, False

        frame_height, frame_width = frame.shape[:2]
        frame_center_x = frame_width / 2

        # Threshold y preprocesamiento
        dark_thres = 100
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, dark_thres, 255, cv2.THRESH_BINARY_INV)
        mask[:int(frame_height * 0.8), :] = 0
        mask = cv2.erode(mask, np.ones((3, 3), np.uint8), iterations=3)
        mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=5)

        # Contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        MIN_AREA = 800
        contours = [c for c in contours if cv2.contourArea(c) > MIN_AREA]

        throttle, yaw = 0.0, 0.0
        line_found = False
        
        if contours:
            line_found = True
            
            def line_score(c):
                pt1, pt2, angle, cx, cy = self.get_contour_line(c)
                
                center_distance = abs(cx - frame_center_x)
                center_score = 1.0 - (center_distance / frame_center_x)
                
                max_angle = 80
                angle_deviation = min(abs(angle), max_angle) / max_angle
                angle_score = 1.0 - angle_deviation
                
                total_score = (self.center_weight * center_score) + (self.angle_weight * angle_score)
                return total_score

            scored_contours = [(c, line_score(c)) for c in contours]
            scored_contours.sort(key=lambda x: x[1], reverse=True)
            
            line_contour = scored_contours[0][0]

            cv2.drawContours(frame, [line_contour], -1, (0, 255, 0), 2)
            for c, score in scored_contours[1:]:
                cv2.drawContours(frame, [c], -1, (0, 0, 255), 2)

            _, _, angle, cx, cy = self.get_contour_line(line_contour)
            normalized_x = (cx - frame_center_x) / frame_center_x
            
            cv2.line(frame, (int(cx), 0), (int(cx), frame_height), (255, 0, 0), 2)

            yaw = self.yaw_pid(normalized_x)
            
            alignment = 1 - abs(normalized_x)
            align_thres = 0.3
            throttle = self.max_thr * ((alignment - align_thres) / (1 - align_thres)) if alignment >= align_thres else 0

        return throttle, yaw, line_found

    def get_contour_line(self, c, fix_vert=True):
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
    
    print("\n\033[1;36m" + "=" * 80)
    print(" " * 15 + "SISTEMA DE DETECCIÓN DE LÍNEAS SEGMENTADAS")
    print(" " * 20 + "(USANDO COMPRESSED IMAGES)")
    print("=" * 80 + "\033[0m")
    
    node = FollowLineNode()
    
    try:
        print("\033[1;32m[INFO] Sistema de PARADA INMEDIATA en segmentos iniciado\033[0m")
        print("\033[1;33m[INFO] El robot se detendrá INMEDIATAMENTE al detectar segmentos\033[0m")
        print("\033[1;31m[ADVERTENCIA] Mensaje se publica SOLO UNA VEZ por intersección\033[0m")
        print("\033[1;34m[INFO] ROI de detección: 35% - 95% del frame\033[0m")
        print("\033[1;35m[INFO] Usando CompressedImage para mayor eficiencia\033[0m\n")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\033[1;33m[INFO] Interrupción de teclado detectada\033[0m")
    except Exception as e:
        print(f"\n\033[1;31m[ERROR] Se produjo un error: {str(e)}\033[0m")
    finally:
        print("\033[1;36m[INFO] Cerrando sistema...\033[0m")
        node.destroy_node()
        rclpy.shutdown()
        
        elapsed = time.time() - node.start_time
        print("\n" + "-" * 80)
        print(f"\033[1mTiempo total\033[0m: {elapsed:.2f} segundos")
        print(f"\033[1mFrames procesados\033[0m: {node.frame_count}")
        print(f"\033[1mFPS promedio\033[0m: {node.frame_count / elapsed if elapsed > 0 else 0:.2f}")
        print("-" * 80 + "\n")

if __name__ == '__main__':
    main()
