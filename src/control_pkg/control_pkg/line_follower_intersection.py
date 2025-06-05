import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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
        self.subscription = self.create_subscription(Image, '/puzzlebot/camera/image_raw', self.image_callback, qos_profile)
        self.debug_pub = self.create_publisher(Image, '/debug_image', 10)
        self.bridge = CvBridge()

        # Camera resizing parameters
        self.camera_height = 250
        self.camera_width = 430

        # PID controller
        max_yaw = math.radians(45)
        self.max_thr = 0.14
        self.yaw_pid = PID(Kp=0.62, Ki=0.05, Kd=0.15, setpoint=0.0, output_limits=(-max_yaw, max_yaw))

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
        
        # Detección específica de líneas segmentadas horizontales
        self.segmented_line_detection_count = 0
        self.segmented_line_threshold = 1  # CAMBIADO: Solo 1 frame para parar inmediatamente
        
        # Parámetros para detectar segmentos rectangulares
        self.min_segment_area = 200  # Área mínima de cada segmento rectangular
        self.max_segment_area = 1500  # Área máxima de cada segmento
        self.min_segments_in_line = 3  # Mínimo número de segmentos para considerar línea
        self.max_segment_gap = 30  # Distancia máxima entre segmentos consecutivos
        self.horizontal_tolerance = 20  # Tolerancia en píxeles para considerar segmentos "horizontales"
        
        # Parámetros específicos para rectangulos
        self.min_segment_width = 15  # Ancho mínimo del segmento
        self.max_segment_width = 80  # Ancho máximo del segmento
        self.min_segment_height = 8  # Alto mínimo del segmento
        self.max_segment_height = 25  # Alto máximo del segmento
        self.aspect_ratio_min = 1.5  # Ratio mínimo ancho/alto (rectangulos horizontales)
        self.aspect_ratio_max = 8.0  # Ratio máximo ancho/alto
        
        # ROI para detección de líneas segmentadas
        self.segmented_roi_start = 0.35  # Desde 60% hacia abajo
        self.segmented_roi_end = 0.8    # Hasta 90% hacia abajo
        
        # Control general
        self.intersection_cooldown = 0
        self.intersection_cooldown_max = 80
        self.final_stop_triggered = False
        
        # Variables para estadísticas y terminal
        self.start_time = time.time()
        self.frame_count = 0
        self.last_status_print = 0
        self.status_interval = 1.0
        
        self._print_header()

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
        
        # Buscar líneas horizontales formadas por segmentos rectangulares
        horizontal_segmented_lines = []
        
        if len(segment_candidates) >= self.min_segments_in_line:
            # Agrupar segmentos por altura (Y similar = línea horizontal)
            y_groups = {}
            for segment in segment_candidates:
                cy = segment['center'][1]
                
                # Buscar grupo existente dentro de la tolerancia
                found_group = False
                for group_y in list(y_groups.keys()):
                    if abs(cy - group_y) <= self.horizontal_tolerance:
                        y_groups[group_y].append(segment)
                        found_group = True
                        break
                
                if not found_group:
                    y_groups[cy] = [segment]
            
            # Analizar cada grupo para ver si forma una línea segmentada
            for group_y, segments_in_group in y_groups.items():
                if len(segments_in_group) >= self.min_segments_in_line:
                    # Ordenar segmentos por X
                    segments_in_group.sort(key=lambda s: s['center'][0])
                    
                    # Verificar distancias entre segmentos consecutivos
                    valid_line = True
                    gaps = []
                    
                    for i in range(len(segments_in_group) - 1):
                        # Calcular distancia entre el final de un segmento y el inicio del siguiente
                        x1_end = segments_in_group[i]['bbox'][0] + segments_in_group[i]['bbox'][2]  # x + width
                        x2_start = segments_in_group[i + 1]['bbox'][0]
                        gap = x2_start - x1_end
                        gaps.append(gap)
                        
                        # Si el gap es demasiado grande o negativo (solapamiento), no es válido
                        if gap < 0 or gap > self.max_segment_gap:
                            valid_line = False
                            break
                    
                    if valid_line and len(gaps) > 0:
                        # Calcular la longitud total de la línea
                        first_segment = segments_in_group[0]
                        last_segment = segments_in_group[-1]
                        line_start = first_segment['bbox'][0]
                        line_end = last_segment['bbox'][0] + last_segment['bbox'][2]
                        line_length = line_end - line_start
                        
                        # Solo considerar líneas que cubran una buena porción del ancho
                        min_line_length = frame_width * 0.3  # Al menos 30% del ancho del frame
                        
                        if line_length >= min_line_length:
                            horizontal_segmented_lines.append({
                                'segments': segments_in_group,
                                'y_position': group_y + roi_start,  # Ajustar coordenada Y al frame completo
                                'avg_gap': np.mean(gaps) if gaps else 0,
                                'line_length': line_length,
                                'num_segments': len(segments_in_group)
                            })
                            
                            # Dibujar línea segmentada detectada
                            for segment in segments_in_group:
                                x, y, w, h = segment['bbox']
                                cv2.rectangle(roi_frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                            
                            # Dibujar línea conectando los segmentos
                            start_x = line_start
                            end_x = line_end
                            line_y = group_y
                            cv2.line(roi_frame, (start_x, line_y), (end_x, line_y), (0, 255, 0), 4)
                            
                            # Texto indicativo
                            cv2.putText(roi_frame, f"SEGMENTED LINE: {len(segments_in_group)} segments", 
                                      (start_x, line_y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            cv2.putText(roi_frame, f"Length: {line_length:.0f}px", 
                                      (start_x, line_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        # Actualizar frame original con ROI procesada
        frame[roi_start:roi_end, :] = roi_frame
        
        # Mostrar información de detección
        cv2.putText(frame, f"Segment candidates: {len(segment_candidates)}", 
                   (10, roi_start + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(frame, f"Segmented lines: {len(horizontal_segmented_lines)}", 
                   (10, roi_start + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Determinar si se detectó intersección
        intersection_detected = len(horizontal_segmented_lines) >= 1
        
        if intersection_detected:
            cv2.putText(frame, "🛑 INTERSECTION - STOPPING NOW!", 
                       (10, roi_start + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 3)
        
        return intersection_detected, horizontal_segmented_lines

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

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.resize(frame, (self.camera_width, self.camera_height))
            
            # Si ya terminamos, no procesar más
            if self.final_stop_triggered:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher.publish(twist)
                
                cv2.putText(frame, "INTERSECTION REACHED - ROBOT STOPPED!", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3)
                cv2.putText(frame, "Segmented line intersection detected", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                self._print_status("FINISHED", 0.0, 0.0, "PARADA INMEDIATA - Intersección")
                
                debug_frame = cv2.resize(frame, (self.camera_width, self.camera_height))
                debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
                self.debug_pub.publish(debug_msg)
                return
            
            # Preprocesar imagen
            dark_thres = 100
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, mask = cv2.threshold(gray, dark_thres, 255, cv2.THRESH_BINARY_INV)
            
            # Reducir cooldown si está activo
            if self.intersection_cooldown > 0:
                self.intersection_cooldown -= 1
            
            # DETECCIÓN DE LÍNEAS SEGMENTADAS HORIZONTALES
            segmented_detected, segmented_lines = self.detect_segmented_horizontal_lines(frame, mask)
            
            # PARADA INMEDIATA cuando se detecten segmentos
            if segmented_detected:
                self.segmented_line_detection_count += 1
                # INTERSECCIÓN CONFIRMADA INMEDIATAMENTE
                intersection_confirmed = True
                
                if not self.final_stop_triggered:
                    self.final_stop_triggered = True
                    self.intersection_cooldown = self.intersection_cooldown_max
                    
                    print(f"\n\033[1;35m🔲 SEGMENTOS DETECTADOS - PARADA INMEDIATA\033[0m")
                    print(f"\033[1;32m📊 Líneas segmentadas encontradas: {len(segmented_lines)}\033[0m")
                    for i, line in enumerate(segmented_lines):
                        print(f"\033[1;33m   Línea {i+1}: {line['num_segments']} segmentos, longitud: {line['line_length']:.0f}px\033[0m")
                    print("\033[1;31m🛑 ROBOT DETENIDO EN INTERSECCIÓN\033[0m")
                    print("\033[1;32m✅ MISIÓN COMPLETADA\033[0m")
            else:
                self.segmented_line_detection_count = 0
                intersection_confirmed = False
            
            # Seguimiento de línea normal
            throttle, yaw, line_found = self.follow_line(frame)
            
            # Detectar fin de línea
            if not line_found and self.line_lost_count > self.max_line_lost_count:
                gray_bottom = gray[int(frame.shape[0] * 0.7):, :]
                _, mask_bottom = cv2.threshold(gray_bottom, dark_thres, 255, cv2.THRESH_BINARY_INV)
                white_pixels_bottom = cv2.countNonZero(mask_bottom)
                
                if white_pixels_bottom < 500:
                    self.line_ended = True
            
            # DECISIÓN FINAL: PARAR ROBOT INMEDIATAMENTE cuando se detecten segmentos
            if intersection_confirmed or self.line_ended:
                # Parar robot INMEDIATAMENTE
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher.publish(twist)
                
                status_msg = "INTERSECCIÓN - PARADA INMEDIATA" if intersection_confirmed else "Fin de línea"
                self._print_status("FINISHED", 0.0, 0.0, status_msg)
                
            else:
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
                    
                    if segmented_detected:
                        self._print_status("SEGMENTED_DETECTED", throttle, yaw, f"¡SEGMENTOS! Parando...")
                    else:
                        self._print_status("TRACKING", throttle, yaw)

                # Publicar comando de movimiento
                twist = Twist()
                twist.linear.x = float(throttle)
                twist.angular.z = float(yaw)
                self.publisher.publish(twist)

            # Mostrar información de detección en frame
            cv2.putText(frame, f"Segmented detection: {self.segmented_line_detection_count} (IMMEDIATE STOP)", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"Line lost: {self.line_lost_count}/{self.max_line_lost_count}", 
                       (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            if intersection_confirmed:
                cv2.putText(frame, "🛑 IMMEDIATE STOP - INTERSECTION!", 
                           (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3)
            
            if segmented_detected and not intersection_confirmed:
                cv2.putText(frame, "🔲 SEGMENTS DETECTED - STOPPING NOW!", 
                           (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
            
            if self.line_ended:
                cv2.putText(frame, "LINE ENDED!", 
                           (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 3)
            
            # Dibujar ROI de detección
            roi_start = int(frame.shape[0] * self.segmented_roi_start)
            roi_end = int(frame.shape[0] * self.segmented_roi_end)
            cv2.rectangle(frame, (0, roi_start), (frame.shape[1], roi_end), (255, 0, 0), 2)
            cv2.putText(frame, "SEGMENTED DETECTION ROI", (5, roi_start - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            # Publicar frame de debug
            debug_frame = cv2.resize(frame, (self.camera_width, self.camera_height))
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
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
        mask[:int(frame_height * 0.7), :] = 0
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
    print("=" * 80 + "\033[0m")
    
    node = FollowLineNode()
    
    try:
        print("\033[1;32m[INFO] Sistema de PARADA INMEDIATA en segmentos iniciado\033[0m")
        print("\033[1;33m[INFO] El robot se detendrá INMEDIATAMENTE al detectar segmentos\033[0m")
        print("\033[1;31m[ADVERTENCIA] SIN confirmación de frames - Parada instantánea\033[0m")
        print("\033[1;34m[INFO] ROI de detección: 60% - 90% del frame\033[0m\n")
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