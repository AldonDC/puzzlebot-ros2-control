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


class DualCameraLineFollowerNode(Node):
    def __init__(self):
        super().__init__('dual_camera_line_follower')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 📸 SUSCRIPTORES PARA AMBAS CÁMARAS
        self.csi_subscription = self.create_subscription(
            Image, '/puzzlebot/camera/image_raw', 
            self.csi_callback, qos_profile)
        
        self.usb_subscription = self.create_subscription(
            Image, '/puzzlebot/usb_camera/image_raw', 
            self.usb_callback, qos_profile)
        
        # Publicadores de debug para ambas cámaras
        self.csi_debug_pub = self.create_publisher(Image, '/debug_csi_image', 10)
        self.usb_debug_pub = self.create_publisher(Image, '/debug_usb_image', 10)
        
        self.bridge = CvBridge()

        # PID controller para CSI
        max_yaw = math.radians(60)
        self.max_thr = 0.2
        self.yaw_pid = PID(Kp=0.6, Ki=0, Kd=0.1, setpoint=0.0, output_limits=(-max_yaw, max_yaw))

        # Factores de ponderación para la jerarquía de decisión (CSI)
        self.center_weight = 0.7
        self.angle_weight = 0.3
        
        # Variables de control de movimiento (CSI)
        self.line_lost_count = 0
        self.max_line_lost_count = 10
        self.backup_speed = -0.1
        self.last_yaw = 0.0
        self.in_search_mode = False
        self.search_yaw_options = [-0.1, 0.0, 0.1]
        self.search_yaw_index = 1
        self.search_time_counter = 0
        self.search_interval = 20
        
        # 🚦 VARIABLES PARA DETECCIÓN (USB CAMERA)
        self.intersection_detected = False
        self.end_of_line_detected = False
        self.is_stopped = False
        self.intersection_threshold = 3
        self.min_contour_area_intersection = 800
        self.stop_time = 0
        self.stop_duration = 2.0
        self.end_line_threshold = 15
        self.end_line_counter = 0
        
        # 📊 VARIABLES DE ESTADO COMPARTIDAS
        self.csi_throttle = 0.0
        self.csi_yaw = 0.0
        self.csi_line_found = False
        self.csi_frame_available = False
        self.usb_frame_available = False
        
        # Variables para estadísticas
        self.start_time = time.time()
        self.frame_count = 0
        self.csi_frame_count = 0
        self.usb_frame_count = 0
        self.last_status_print = 0
        self.status_interval = 1.0
        
        self._print_header()

    def _print_header(self):
        """Imprime un encabezado atractivo en la terminal"""
        terminal_width = 80
        print("\n" + "=" * terminal_width)
        print(" " * 15 + "\033[1;36mPUZZLEBOT DUAL CAMERA LINE FOLLOWER\033[0m")
        print(" " * 10 + "\033[1;33m📸 CSI: Line Following | 🔌 USB: Intersection Detection\033[0m")
        print("=" * terminal_width)
        print(f"\033[1mLanzado\033[0m: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"\033[1mCSI Camera\033[0m: /puzzlebot/camera/image_raw (Line Following)")
        print(f"\033[1mUSB Camera\033[0m: /puzzlebot/usb_camera/image_raw (Intersection Detection)")
        print(f"\033[1mPID Config\033[0m: Kp={self.yaw_pid.Kp}, Ki={self.yaw_pid.Ki}, Kd={self.yaw_pid.Kd}")
        print(f"\033[1mVel máxima\033[0m: {self.max_thr} m/s | \033[1mParada\033[0m: {self.stop_duration}s")
        print("-" * terminal_width)
        print("\033[1m{:<8} {:<15} {:<8} {:<8} {:<10} {:<10} {:<25}\033[0m".format(
            "TIEMPO", "ESTADO", "CSI_FPS", "USB_FPS", "VEL(m/s)", "YAW(rad/s)", "INFO"))
        print("-" * terminal_width)
        sys.stdout.flush()

    def _print_status(self, state, throttle, yaw, info=""):
        """Imprime actualizaciones de estado en formato bonito"""
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        self.frame_count += 1
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        csi_fps = self.csi_frame_count / elapsed if elapsed > 0 else 0
        usb_fps = self.usb_frame_count / elapsed if elapsed > 0 else 0
        
        if current_time - self.last_status_print >= self.status_interval:
            # Formatear estado con colores
            if state == "TRACKING":
                state_formatted = f"\033[1;32m{state}\033[0m"
            elif state == "SEARCHING":
                state_formatted = f"\033[1;31m{state}\033[0m" 
            elif state == "BACKUP":
                state_formatted = f"\033[1;33m{state}\033[0m"
            elif state == "🚦 INTERSECTION":
                state_formatted = f"\033[1;35m{state}\033[0m"
            elif state == "🛑 END_OF_LINE":
                state_formatted = f"\033[1;31m{state}\033[0m"
            elif state == "⏸️ STOPPED":
                state_formatted = f"\033[1;36m{state}\033[0m"
            else:
                state_formatted = state
                
            print("\033[K{:<8.1f} {:<15} {:<8.1f} {:<8.1f} {:<10.2f} {:<10.2f} {:<25}".format(
                elapsed, state_formatted, csi_fps, usb_fps, throttle, yaw, info))
            sys.stdout.flush()
            
            self.last_status_print = current_time

    def csi_callback(self, msg):
        """📸 CSI CAMERA: Procesa line following"""
        self.csi_frame_count += 1
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Procesar line following
        throttle, yaw, line_found = self.follow_line_csi(frame)
        
        # Guardar estado para el control principal
        self.csi_throttle = throttle
        self.csi_yaw = yaw
        self.csi_line_found = line_found
        self.csi_frame_available = True
        
        # Agregar información de estado en la imagen CSI
        cv2.putText(frame, "📸 CSI CAMERA - LINE FOLLOWING", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(frame, f"Line Found: {'YES' if line_found else 'NO'}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if line_found else (0, 0, 255), 2)
        cv2.putText(frame, f"Throttle: {throttle:.2f} | Yaw: {yaw:.2f}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Publicar imagen de debug CSI
        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.csi_debug_pub.publish(debug_msg)
        
        # Procesar comando de movimiento si tenemos ambas cámaras
        self.process_movement_command()

    def usb_callback(self, msg):
        """🔌 USB CAMERA: Detecta intersecciones y final de línea"""
        self.usb_frame_count += 1
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Procesar detección de intersecciones y final de línea
        self.detect_intersection_and_end(frame)
        self.usb_frame_available = True
        
        # Agregar información de estado en la imagen USB
        cv2.putText(frame, "🔌 USB CAMERA - INTERSECTION DETECTION", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
        
        status_color = (255, 0, 255) if self.intersection_detected else (0, 255, 0)
        cv2.putText(frame, f"Intersection: {'DETECTED' if self.intersection_detected else 'NO'}", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        
        end_color = (0, 0, 255) if self.end_of_line_detected else (0, 255, 0)
        cv2.putText(frame, f"End of Line: {'YES' if self.end_of_line_detected else 'NO'}", 
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, end_color, 2)
        
        if self.is_stopped:
            cv2.putText(frame, "🛑 ROBOT STOPPED", (10, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        # Publicar imagen de debug USB
        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.usb_debug_pub.publish(debug_msg)

    def process_movement_command(self):
        """Procesa el comando de movimiento final combinando ambas cámaras"""
        if not (self.csi_frame_available and self.usb_frame_available):
            return
            
        current_time = time.time()
        throttle = self.csi_throttle
        yaw = self.csi_yaw
        
        # 🚦 LÓGICA DE INTERSECCIÓN (desde USB)
        if self.intersection_detected and not self.is_stopped and not self.end_of_line_detected:
            self.is_stopped = True
            self.stop_time = current_time
            throttle = 0.0
            yaw = 0.0
            self._print_status("🚦 INTERSECTION", throttle, yaw, "Parada por intersección")
        
        # 🛑 LÓGICA DE FINAL DE LÍNEA (desde USB)
        elif self.end_of_line_detected and not self.is_stopped:
            self.is_stopped = True
            throttle = 0.0
            yaw = 0.0
            self._print_status("🛑 END_OF_LINE", throttle, yaw, "Final de línea detectado")
        
        # ⏸️ MANEJO DE PARADA TEMPORAL EN INTERSECCIÓN
        elif self.is_stopped and self.intersection_detected and not self.end_of_line_detected:
            elapsed_stop = current_time - self.stop_time
            remaining_time = self.stop_duration - elapsed_stop
            
            if remaining_time > 0:
                throttle = 0.0
                yaw = 0.0
                self._print_status("⏸️ STOPPED", throttle, yaw, f"Reanuda en {remaining_time:.1f}s")
            else:
                # Reanudar movimiento
                self.is_stopped = False
                self.intersection_detected = False
                self._print_status("TRACKING", throttle, yaw, "Reanudando desde intersección")
        
        # 🏃 MOVIMIENTO NORMAL (CSI controla)
        elif not self.is_stopped:
            if self.csi_line_found:
                self._print_status("TRACKING", throttle, yaw, "Siguiendo línea CSI")
            else:
                self._print_status("SEARCHING", throttle, yaw, "Buscando línea CSI")
        
        # Publicar comando final
        twist = Twist()
        twist.linear.x = float(throttle)
        twist.angular.z = float(yaw)
        self.publisher.publish(twist)

    def follow_line_csi(self, frame):
        """Lógica de seguimiento de línea para CSI camera"""
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

        # Mostrar líneas candidatas
        for i, c in enumerate(contours):
            pt1, pt2, angle, cx, cy = self.get_contour_line(c)
            cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
            cv2.putText(frame, str(i), (int(cx), int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

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
                
                score_text = f"C:{center_score:.2f} A:{angle_score:.2f} T:{total_score:.2f}"
                cv2.putText(frame, score_text, (int(cx), int(cy) - 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                return total_score

            scored_contours = [(c, line_score(c)) for c in contours]
            scored_contours.sort(key=lambda x: x[1], reverse=True)
            
            line_contour = scored_contours[0][0]
            best_score = scored_contours[0][1]

            cv2.drawContours(frame, [line_contour], -1, (0, 255, 0), 2)
            for c, score in scored_contours[1:]:
                cv2.drawContours(frame, [c], -1, (0, 0, 255), 2)

            _, _, angle, cx, cy = self.get_contour_line(line_contour)
            normalized_x = (cx - frame_center_x) / frame_center_x
            
            cv2.line(frame, (int(cx), 0), (int(cx), frame_height), (255, 0, 0), 2)
            cv2.putText(frame, f"Selected: Score={best_score:.2f}", (10, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            yaw = self.yaw_pid(normalized_x)
            alignment = 1 - abs(normalized_x)
            align_thres = 0.3
            throttle = self.max_thr * ((alignment - align_thres) / (1 - align_thres)) if alignment >= align_thres else 0

        return throttle, yaw, line_found

    def detect_intersection_and_end(self, frame):
        """Detecta intersecciones y final de línea usando USB camera"""
        frame_height, frame_width = frame.shape[:2]
        
        # Preprocesamiento similar pero para toda la imagen
        dark_thres = 100
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, dark_thres, 255, cv2.THRESH_BINARY_INV)
        mask = cv2.erode(mask, np.ones((3, 3), np.uint8), iterations=2)
        mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=3)

        # Encontrar contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [c for c in contours if cv2.contourArea(c) > self.min_contour_area_intersection]
        
        # Dibujar todos los contornos detectados
        cv2.drawContours(frame, valid_contours, -1, (255, 255, 0), 2)
        cv2.putText(frame, f"Contours detected: {len(valid_contours)}", (10, 150), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 🚦 DETECCIÓN DE INTERSECCIÓN
        if len(valid_contours) >= self.intersection_threshold and not self.intersection_detected:
            # Verificar distribución horizontal
            centers_x = []
            for contour in valid_contours:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    centers_x.append(cx)
                    cv2.circle(frame, (cx, int(M["m01"] / M["m00"])), 5, (255, 0, 255), -1)
            
            if len(centers_x) >= self.intersection_threshold:
                centers_x.sort()
                spread = centers_x[-1] - centers_x[0]
                
                if spread > frame_width * 0.4:
                    self.intersection_detected = True
                    cv2.putText(frame, "🚦 INTERSECTION DETECTED!", (10, 200), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 3)
                    
                    for contour in valid_contours:
                        cv2.drawContours(frame, [contour], -1, (255, 0, 255), 3)
        
        # 🛑 DETECCIÓN DE FINAL DE LÍNEA
        if len(valid_contours) == 0:
            self.end_line_counter += 1
        else:
            self.end_line_counter = 0
            
        if self.end_line_counter >= self.end_line_threshold and not self.end_of_line_detected:
            self.end_of_line_detected = True
            cv2.putText(frame, "🛑 END OF LINE DETECTED!", (10, 230), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        
        # Información de debug
        cv2.putText(frame, f"End counter: {self.end_line_counter}/{self.end_line_threshold}", 
                   (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

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
    print(" " * 20 + "INICIANDO SISTEMA DUAL CAMERA")
    print(" " * 15 + "📸 CSI: LINE FOLLOWING | 🔌 USB: DETECTION")
    print("=" * 80 + "\033[0m")
    
    node = DualCameraLineFollowerNode()
    try:
        print("\033[1;32m[INFO] Nodo dual camera iniciado. Esperando ambas cámaras...\033[0m\n")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\033[1;33m[INFO] Interrupción de teclado detectada\033[0m")
    except Exception as e:
        print(f"\n\033[1;31m[ERROR] Se produjo un error: {str(e)}\033[0m")
    finally:
        print("\033[1;36m[INFO] Cerrando nodo dual camera...\033[0m")
        node.destroy_node()
        rclpy.shutdown()
        
        elapsed = time.time() - node.start_time
        print("\n" + "-" * 80)
        print(f"\033[1mTiempo total\033[0m: {elapsed:.2f} segundos")
        print(f"\033[1mFrames CSI\033[0m: {node.csi_frame_count}")
        print(f"\033[1mFrames USB\033[0m: {node.usb_frame_count}")
        print(f"\033[1mFPS CSI\033[0m: {node.csi_frame_count / elapsed if elapsed > 0 else 0:.2f}")
        print(f"\033[1mFPS USB\033[0m: {node.usb_frame_count / elapsed if elapsed > 0 else 0:.2f}")
        print(f"\033[1mIntersecciones\033[0m: {'SÍ' if node.intersection_detected else 'NO'}")
        print(f"\033[1mFinal línea\033[0m: {'SÍ' if node.end_of_line_detected else 'NO'}")
        print("-" * 80 + "\n")

if __name__ == '__main__':
    main()