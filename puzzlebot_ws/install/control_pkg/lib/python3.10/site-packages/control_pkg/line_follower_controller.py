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
        self.camera_height = 240
        self.camera_width = 420

        # PID controller
        max_yaw = math.radians(45)
        self.max_thr = 0.12
        self.yaw_pid = PID(Kp=0.62, Ki=0.05, Kd=0.15, setpoint=0.0, output_limits=(-max_yaw, max_yaw))

        # Factores de ponderación para la jerarquía de decisión
        self.center_weight = 0.7  # Mayor peso a la centralidad
        self.angle_weight = 0.3   # Menor peso al ángulo
        
        # Parámetros para el movimiento de reversa durante búsqueda
        self.line_lost_count = 0
        self.max_line_lost_count = 5  # Frames perdidos antes de activar reversa
        self.backup_speed = -0.1    # Velocidad de reversa (negativa para ir hacia atrás)
        self.last_yaw = 0.0         # Último valor de yaw para mantener orientación durante reversa
        self.in_search_mode = False # Estado de búsqueda
        self.search_yaw_options = [-0.1, 0.0, 0.1]  # Opciones de giro durante búsqueda
        self.search_yaw_index = 1   # Índice de la opción de giro (iniciamos en 0 = centro)
        self.search_time_counter = 0 # Contador para cambiar la estrategia de búsqueda
        self.search_interval = 20    # Cambiar estrategia cada X frames
        
        # Variables para estadísticas y terminal
        self.start_time = time.time()
        self.frame_count = 0
        self.last_status_print = 0
        self.status_interval = 1.0  # Actualizar la terminal cada 1 segundo
        
        # Imprimir encabezado bonito al inicio
        self._print_header()

    def _print_header(self):
        """Imprime un encabezado atractivo en la terminal"""
        terminal_width = 80
        print("\n" + "=" * terminal_width)
        print(" " * 25 + "\033[1;36mPUZZLEBOT LINE FOLLOWER\033[0m")
        print(" " * 22 + "\033[1;33m⚡ HIGH PERFORMANCE MODE ⚡\033[0m")
        print("=" * terminal_width)
        print(f"\033[1mLanzado\033[0m: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"\033[1mConfiguración\033[0m: PID(Kp={self.yaw_pid.Kp}, Ki={self.yaw_pid.Ki}, Kd={self.yaw_pid.Kd})")
        print(f"\033[1mVelocidad máxima\033[0m: {self.max_thr} m/s")
        print(f"\033[1mResolución cámara\033[0m: {self.camera_width}x{self.camera_height}")
        print(f"\033[1mEstratégia\033[0m: Control jerárquico (centro: {self.center_weight}, ángulo: {self.angle_weight})")
        print(f"\033[1mModo seguridad\033[0m: {'Activado' if self.max_line_lost_count > 0 else 'Desactivado'}")
        print("-" * terminal_width)
        print("\033[1m{:<8} {:<12} {:<10} {:<10} {:<12} {:<20}\033[0m".format(
            "TIEMPO", "ESTADO", "VEL(m/s)", "YAW(rad/s)", "FPS", "INFO"))
        print("-" * terminal_width)
        sys.stdout.flush()

    def _print_status(self, state, throttle, yaw, info=""):
        """Imprime actualizaciones de estado en formato bonito"""
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        # Actualizar contador de frames y calcular FPS
        self.frame_count += 1
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        
        # Limitar actualizaciones de la terminal para no sobrecargarla
        if current_time - self.last_status_print >= self.status_interval:
            # Dar formato al estado con colores
            if state == "TRACKING":
                state_formatted = f"\033[1;32m{state}\033[0m"
            elif state == "SEARCHING":
                state_formatted = f"\033[1;31m{state}\033[0m" 
            elif state == "BACKUP":
                state_formatted = f"\033[1;33m{state}\033[0m"
            else:
                state_formatted = state
                
            # Imprimir línea de estado
            print("\033[K{:<8.1f} {:<12} {:<10.2f} {:<10.2f} {:<12.1f} {:<20}".format(
                elapsed, state_formatted, throttle, yaw, fps, info))
            sys.stdout.flush()
            
            # Actualizar tiempo de última impresión
            self.last_status_print = current_time

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize the camera frame to the desired dimensions
            frame = cv2.resize(frame, (self.camera_width, self.camera_height))
            
            throttle, yaw, line_found = self.follow_line(frame)

            # Lógica para el modo de búsqueda con reversa
            if not line_found:
                self.line_lost_count += 1
                
                if self.line_lost_count >= self.max_line_lost_count:
                    self.in_search_mode = True
                    
                    # En modo búsqueda, retrocedemos con un patrón variable de giro
                    throttle = self.backup_speed
                    
                    # Actualizamos contador de búsqueda
                    self.search_time_counter += 1
                    if self.search_time_counter >= self.search_interval:
                        # Cambiar estrategia de búsqueda rotando entre las opciones
                        self.search_yaw_index = (self.search_yaw_index + 1) % len(self.search_yaw_options)
                        self.search_time_counter = 0
                    
                    # Aplicar el yaw actual de la estrategia de búsqueda
                    yaw = self.search_yaw_options[self.search_yaw_index]
                    
                    # Imprimir estado en terminal
                    info = f"Opción búsqueda: {self.search_yaw_index}/{len(self.search_yaw_options)-1}"
                    self._print_status("BACKUP", throttle, yaw, info)
                    
                    cv2.putText(frame, f"Searching: Backing up (yaw={yaw:.2f})", (10, 90), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                # Si encontramos la línea, reseteamos contadores
                self.line_lost_count = 0
                self.in_search_mode = False
                self.search_time_counter = 0
                self.search_yaw_index = 1  # Volvemos a la posición central
                self.last_yaw = yaw  # Guardamos el último yaw válido
                
                # Imprimir estado en terminal
                self._print_status("TRACKING", throttle, yaw)

            # Publicar comando de movimiento
            twist = Twist()
            twist.linear.x = float(throttle)
            twist.angular.z = float(yaw)
            self.publisher.publish(twist)

            # Resize debug frame before publishing
            debug_frame = cv2.resize(frame, (self.camera_width, self.camera_height))
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f"Error en callback: {str(e)}")
            # Comando de seguridad
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
            
            # Nueva función de puntuación para jerarquía de decisión
            def line_score(c):
                pt1, pt2, angle, cx, cy = self.get_contour_line(c)
                
                # Factor de centralidad (normalizado entre 0-1, donde 1 es perfecto)
                center_distance = abs(cx - frame_center_x)
                center_score = 1.0 - (center_distance / frame_center_x)
                
                # Factor de ángulo (normalizado entre 0-1, donde 1 es perfecto)
                max_angle = 80
                angle_deviation = min(abs(angle), max_angle) / max_angle
                angle_score = 1.0 - angle_deviation
                
                # Puntuación combinada ponderada
                total_score = (self.center_weight * center_score) + (self.angle_weight * angle_score)
                
                # Visualización de puntuación
                score_text = f"C:{center_score:.2f} A:{angle_score:.2f} T:{total_score:.2f}"
                cv2.putText(frame, score_text, (int(cx), int(cy) - 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                
                return total_score

            # Ordenar contornos por puntuación (mayor primero)
            scored_contours = [(c, line_score(c)) for c in contours]
            scored_contours.sort(key=lambda x: x[1], reverse=True)
            
            # Seleccionar la línea con mejor puntuación
            line_contour = scored_contours[0][0]
            best_score = scored_contours[0][1]

            # Visualización
            cv2.drawContours(frame, [line_contour], -1, (0, 255, 0), 2)
            for c, score in scored_contours[1:]:
                cv2.drawContours(frame, [c], -1, (0, 0, 255), 2)

            # Calcular centro y comandos de movimiento
            _, _, angle, cx, cy = self.get_contour_line(line_contour)
            normalized_x = (cx - frame_center_x) / frame_center_x
            
            # Dibujar línea central seleccionada
            cv2.line(frame, (int(cx), 0), (int(cx), frame_height), (255, 0, 0), 2)
            cv2.putText(frame, f"Selected: Score={best_score:.2f}", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Calcular comando de dirección
            yaw = self.yaw_pid(normalized_x)
            
            # Ajustar velocidad según alineación
            alignment = 1 - abs(normalized_x)
            align_thres = 0.3
            throttle = self.max_thr * ((alignment - align_thres) / (1 - align_thres)) if alignment >= align_thres else 0
        else:
            cv2.putText(frame, "Searching for line", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # Mostrar estado del modo de búsqueda
            search_status = "Activating search mode" if self.line_lost_count < self.max_line_lost_count else "Search mode active"
            cv2.putText(frame, search_status, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Imprimir estado en terminal
            info = f"Línea perdida: {self.line_lost_count}/{self.max_line_lost_count}"
            self._print_status("SEARCHING", throttle, yaw, info)

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
    
    # Imprimir mensaje de inicio bonito
    print("\n\033[1;36m" + "=" * 80)
    print(" " * 28 + "INICIANDO SISTEMA")
    print("=" * 80 + "\033[0m")
    
    node = FollowLineNode()
    try:
        print("\033[1;32m[INFO] Nodo iniciado correctamente. Ejecutando...\033[0m\n")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\033[1;33m[INFO] Interrupción de teclado detectada\033[0m")
    except Exception as e:
        print(f"\n\033[1;31m[ERROR] Se produjo un error: {str(e)}\033[0m")
    finally:
        print("\033[1;36m[INFO] Cerrando nodo y finalizando...\033[0m")
        node.destroy_node()
        rclpy.shutdown()
        
        # Imprimir estadísticas finales
        elapsed = time.time() - node.start_time
        print("\n" + "-" * 80)
        print(f"\033[1mTiempo total\033[0m: {elapsed:.2f} segundos")
        print(f"\033[1mFrames procesados\033[0m: {node.frame_count}")
        print(f"\033[1mFPS promedio\033[0m: {node.frame_count / elapsed if elapsed > 0 else 0:.2f}")
        print("-" * 80 + "\n")

if __name__ == '__main__':
    main() 