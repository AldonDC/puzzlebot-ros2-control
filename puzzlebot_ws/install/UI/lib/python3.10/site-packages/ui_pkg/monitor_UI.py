import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32
from cv_bridge import CvBridge

import cv2
import threading
import tkinter as tk
from tkinter import ttk
from PIL import Image as PILImage, ImageTk


class TopicMonitorNode(Node):
    def __init__(self, gui_callback):
        super().__init__('topic_monitor_node')
        self.bridge = CvBridge()
        self.gui_callback = gui_callback

        image_topics = ['/debug_image', '/debug_image_sign', '/debug_image_traffic']
        data_topics = [
            ('/detected_traffic_sign', String),
            ('/intersection', String),
            ('/max_vel', Float32),
            ('/navigation_enable', Bool),
            ('/traffic_light_color', String),
            ('/turn_command', String)
        ]

        for topic in image_topics:
            self.create_subscription(Image, topic, self.image_callback(topic), 10)

        for topic, msg_type in data_topics:
            self.create_subscription(msg_type, topic, self.data_callback(topic), 10)

    def image_callback(self, topic_name):
        def callback(msg):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.gui_callback('image', topic_name, cv_image)
            except Exception as e:
                self.get_logger().error(f"Image conversion failed for {topic_name}: {e}")
        return callback

    def data_callback(self, topic_name):
        def callback(msg):
            self.gui_callback('data', topic_name, str(msg.data))
        return callback


class ROSMonitorGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("🚗 ROS2 Traffic Dashboard")
        self.root.configure(bg='#0f0f0f')
        self.root.geometry("1400x900")

        self.labels = {}
        self.image_labels = {}

        self.colors = {
            'bg_main': '#0f0f0f', 'bg_panel': '#1a1a1a', 'bg_card': '#252525',
            'accent': '#00d4ff', 'accent_secondary': '#ff6b35', 'text_primary': '#ffffff',
            'text_secondary': '#b0b0b0', 'success': '#4caf50', 'warning': '#ff9800', 'error': '#f44336'
        }

        self.fonts = {
            'title': ('Segoe UI', 20, 'bold'), 'section': ('Segoe UI', 14, 'bold'),
            'label': ('Segoe UI', 11), 'value': ('Consolas', 11, 'bold'), 'status': ('Segoe UI', 10)
        }

        self.image_topics = {
            '/debug_image': {'title': '🎥 Cámara Principal', 'icon': '🎯'},
            '/debug_image_sign': {'title': '🚸 Detección Señales', 'icon': '⚠️'},
            '/debug_image_traffic': {'title': '🚦 Semáforos', 'icon': '🔴'}
        }

        self.data_topics = {
            '/detected_traffic_sign': {'title': 'Señal Detectada', 'icon': '🚸', 'color': self.colors['warning']},
            '/intersection': {'title': 'Intersección', 'icon': '🛣️', 'color': self.colors['accent']},
            '/max_vel': {'title': 'Velocidad Máxima', 'icon': '⚡', 'color': self.colors['success'], 'unit': ' km/h'},
            '/navigation_enable': {'title': 'Navegación Activa', 'icon': '🧭', 'color': self.colors['accent']},
            '/traffic_light_color': {'title': 'Color Semáforo', 'icon': '🚦', 'color': self.colors['accent_secondary']},
            '/turn_command': {'title': 'Comando Giro', 'icon': '↩️', 'color': self.colors['accent']}
        }

        self._build_gui()

    def _build_gui(self):
        header = tk.Frame(self.root, bg=self.colors['bg_main'])
        header.pack(fill='x', padx=20, pady=10)

        tk.Label(header, text="🚗 ROS2 Traffic Dashboard", font=self.fonts['title'],
                 bg=self.colors['bg_main'], fg=self.colors['text_primary']).pack(side='left')

        self.status_label = tk.Label(header, text="🟢 CONECTADO", font=self.fonts['status'],
                                     bg=self.colors['bg_main'], fg=self.colors['success'])
        self.status_label.pack(side='right')

        self.video_frame = tk.Frame(self.root, bg=self.colors['bg_panel'])
        self.video_frame.pack(fill='both', expand=True, padx=20, pady=10)

        for topic, config in self.image_topics.items():
            frame = tk.Frame(self.video_frame, bg=self.colors['bg_card'], bd=1, relief='sunken')
            frame.pack(side='left', expand=True, fill='both', padx=5)

            tk.Label(frame, text=f"{config['icon']} {config['title']}",
                     bg=self.colors['bg_card'], fg=self.colors['text_primary'],
                     font=self.fonts['label']).pack(fill='x', pady=5)

            label = tk.Label(frame, text=f"Esperando señal...\n{topic}",
                             bg='#000000', fg=self.colors['text_secondary'])
            label.pack(expand=True, fill='both', padx=5, pady=5)
            self.image_labels[topic] = label

        self.data_frame = tk.Frame(self.root, bg=self.colors['bg_panel'])
        self.data_frame.pack(fill='both', expand=False, padx=20, pady=(0, 20))

        rows = 2
        cols = 3
        idx = 0
        for r in range(rows):
            row = tk.Frame(self.data_frame, bg=self.colors['bg_panel'])
            row.pack(fill='x', pady=5)
            for c in range(cols):
                if idx >= len(self.data_topics):
                    break
                topic, config = list(self.data_topics.items())[idx]
                card = tk.Frame(row, bg=self.colors['bg_card'], bd=1, relief='ridge')
                card.pack(side='left', expand=True, fill='both', padx=5)

                title = f"{config['icon']} {config['title']}"
                label = tk.Label(card, text=title, bg=self.colors['bg_card'],
                                 fg=self.colors['text_primary'], font=self.fonts['label'])
                label.pack(anchor='w', padx=10, pady=(10, 0))

                value_label = tk.Label(card, text="-", bg=self.colors['bg_card'],
                                       fg=config['color'], font=self.fonts['value'])
                value_label.pack(anchor='w', padx=10, pady=(5, 10))

                self.labels[topic] = value_label
                idx += 1

    def update(self, msg_type, topic, content):
        if msg_type == 'image':
            img = cv2.cvtColor(content, cv2.COLOR_BGR2RGB)
            img = cv2.resize(img, (320, 240))
            img_pil = PILImage.fromarray(img)
            img_tk = ImageTk.PhotoImage(img_pil)
            label = self.image_labels.get(topic)
            if label:
                label.config(image=img_tk, text='')
                label.image = img_tk
        elif msg_type == 'data':
            label = self.labels.get(topic)
            if label:
                unit = self.data_topics[topic].get('unit', '')
                label.config(text=f"{content}{unit}")

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    gui = ROSMonitorGUI()

    node = TopicMonitorNode(gui.update)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    gui.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
