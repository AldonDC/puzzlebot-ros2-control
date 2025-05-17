<div align="center">

# 🤖 PuzzleBot ROS 2 Framework

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-green?style=flat-square)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10-yellow?style=flat-square&logo=python)](https://www.python.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange?style=flat-square&logo=ubuntu)](https://ubuntu.com/)

**Control avanzado para robots móviles PuzzleBot usando ROS 2**  
*Seguimiento de líneas, detección de señales/semáforos y navegación autónoma*

[🚀 Inicio rápido](#-inicio-rápido) •
[📦 Instalación](#-instalación) •
[📋 Características](#-características) •
[🔄 Arquitectura](#-arquitectura-del-sistema) •
[📝 Documentación](#-documentación) •
[⚙️ Configuración](#-configuración-avanzada)

</div>

---

## 🌟 Características principales

<table>
  <tr>
    <td width="25%" align="center">
      <img src="https://github.com/user-attachments/assets/1b3d9abd-4280-43cb-b9d5-08e9ce55da1e" width="100"><br>
      <b>Seguimiento de líneas</b><br>
      Algoritmos avanzados de visión con filtrado HSV adaptativo
    </td>
    <td width="25%" align="center">
      <img src="https://github.com/user-attachments/assets/4a9c7b0c-78bf-42f3-98ae-9eed8ad6b7c3" width="100"><br>
      <b>Detección de semáforos</b><br>
      Reconocimiento en tiempo real con clasificación por color
    </td>
    <td width="25%" align="center">
      <img src="https://github.com/user-attachments/assets/e2c3be9b-60c6-41eb-b0dd-2c01a6c47e19" width="100"><br>
      <b>Detección de señales</b><br>
      Identificación y respuesta a señales de tráfico
    </td>
    <td width="25%" align="center">
      <img src="https://github.com/user-attachments/assets/c21e5d31-a8b7-409d-b631-8c41c8e8768a" width="100"><br>
      <b>Navegación autónoma</b><br>
      Control PID optimizado para trayectorias precisas
    </td>
  </tr>
</table>

## 🚀 Inicio rápido

```bash
# Clonar el repositorio
git clone https://github.com/AldonDC/puzzlebot-ros2.git
cd puzzlebot-ros2

# Configuración automática (una sola vez)
chmod +x scripts/puzzlebot_pro.sh
./scripts/puzzlebot_pro.sh

# ¡Comienza a trabajar con el PuzzleBot!
puzzlebot
```

## 📦 Instalación

### Requisitos previos

- Ubuntu 20.04/22.04
- ROS 2 Humble/Foxy
- Acceso a PuzzleBot con Jetson Nano

### Instalación automática

Nuestro framework incluye scripts de configuración automática que detectan tu entorno y configuran todo lo necesario:

```bash
# Dar permisos de ejecución
chmod +x scripts/puzzlebot_pro.sh

# Ejecutar configuración
./scripts/puzzlebot_pro.sh
```

El script:
- Detecta automáticamente la IP de tu laptop en la red 10.42.0.x
- Configura las variables de entorno ROS_DOMAIN_ID y ROS_IP
- Crea alias útiles para uso diario
- Verifica la conexión con el PuzzleBot

## 📋 Características

### Nodos principales

#### Control

| Nodo | Descripción | Tópicos publicados | Tópicos suscritos |
|------|-------------|-------------------|-------------------|
| `line_follower_controller` | Sigue líneas mediante visión con filtrado HSV adaptativo | `/cmd_vel` | `/line_position` |
| `traffic_light_controller` | Detecta y responde a semáforos con transiciones suaves | `/cmd_vel` | `/traffic_light` |
| `sign_response_controller` | **NUEVO**: Responde a señales de tráfico detectadas | `/cmd_vel` | `/traffic_sign`, `/odom` |
| `pid_controller_node` | Control PID optimizado para movimiento preciso | `/cmd_vel` | `/target`, `/odom` |
| `path_generator_node` | Genera trayectorias para navegación autónoma | `/target` | `/odom` |
| `odometry_node` | Cálculo mejorado de posición con fusión de datos | `/odom` | `/encoders` |

#### Detección

| Nodo | Descripción | Tópicos publicados | Tópicos suscritos |
|------|-------------|-------------------|-------------------|
| `traffic_detector` | Detecta semáforos con algoritmos robustos | `/traffic_light` | `/image_raw` |
| `sign_detector` | **NUEVO**: Identifica señales de tráfico (STOP, GIVE WAY, etc.) | `/traffic_sign` | `/image_raw` |
| `angular_error_node` | Cálculo optimizado de error angular para navegación precisa | `/angular_error` | `/image_raw` |
| `debug_visualizer` | Visualización en tiempo real del procesamiento de imágenes | `/debug_image` | `/image_raw` |

### Comandos útiles

```bash
# Activar entorno PuzzleBot
puzzlebot

# Monitorear tópicos
puzzlemon

# Lanzar nodos específicos
./scripts/launch_puzzlebot.sh
```

## 🔄 Arquitectura del Sistema

### Diagrama de Comunicación entre Nodos

<div align="center">
  <img src="https://github.com/user-attachments/assets/ed0a4819-16aa-4392-89c5-3c64835ffd0c" alt="Diagrama de Comunicación entre Nodos" width="800">
</div>


### Flujo de Datos y Procesamiento

<table>
  <tr>
    <td width="50%" align="center">
      <img src="https://github.com/user-attachments/assets/8a1d9e65-74a6-4af1-9ab4-b0e0d43faf76" width="350"><br>
      <b>Procesamiento de Visión</b><br>
      Flujo para detección de señales, semáforos y seguimiento de línea
    </td>
    <td width="50%" align="center">
      <img src="https://github.com/user-attachments/assets/c4e5da3c-1a70-4baa-8a3e-42a0c5f5c2ba" width="350"><br>
      <b>Sistema de Control</b><br>
      Arquitectura PID y generación de comandos de velocidad
    </td>
  </tr>
</table>

## 🔧 Conexión con el PuzzleBot

El framework está optimizado para la red PuzzleBot donde:
- **Jetson**: IP fija en 10.42.0.2
- **Tu laptop**: IP automáticamente detectada en la red 10.42.0.x

### Explicación detallada de la configuración ROS 2

Para garantizar la comunicación correcta entre todos los miembros del equipo y el PuzzleBot, es importante entender cómo funciona:

**En la laptop de cada miembro del equipo:**
```bash
export ROS_DOMAIN_ID=0        # Mismo valor para todos
export ROS_IP=10.42.0.X       # La IP de cada laptop (diferente para cada uno)
                             # Por ejemplo: 10.42.0.91 para un miembro, 10.42.0.92 para otro
```

**En la Jetson (PuzzleBot):**
```bash
export ROS_DOMAIN_ID=0        # Mismo valor que en las laptops
export ROS_IP=10.42.0.2       # IP fija de la Jetson
```

**Puntos clave:**

1. **ROS_DOMAIN_ID** determina qué nodos ROS 2 pueden "verse" entre sí. Todos los dispositivos con el mismo DOMAIN_ID pueden comunicarse.

2. **ROS_IP** le dice a ROS 2 qué dirección IP usar para la comunicación. Cada dispositivo usa su propia IP.

3. **No es necesario** configurar la IP de los otros dispositivos. ROS 2 descubre automáticamente a todos los nodos en el mismo DOMAIN_ID.

El script `puzzlebot_pro.sh` configura automáticamente estas variables para ti, detectando tu IP en la red 10.42.0.x y configurando el DOMAIN_ID adecuado.

### Nota importante para el equipo

Para garantizar la comunicación entre todos los miembros del equipo y el PuzzleBot:

1. Todos deben usar **ROS_DOMAIN_ID=0** al configurar con el script puzzlebot_pro.sh
2. Todos deben estar conectados a la misma red WiFi del PuzzleBot
3. La Jetson del PuzzleBot tiene la IP fija 10.42.0.2

## 📝 Documentación

### Estructura del repositorio

```
puzzlebot_ws/
├── assets/                         # Imágenes y recursos para documentación
├── build/                          # Archivos de compilación (generados)
├── control_pkg/                    # Paquete de controladores
│   ├── control_pkg/
│   │   ├── __init__.py
│   │   ├── line_follower_controller.py    # Mejorado con filtrado HSV adaptativo
│   │   ├── traffic_light_controller.py    # Actualizado con transiciones suaves
│   │   ├── sign_response_controller.py    # NUEVO: Respuesta a señales de tráfico
│   │   ├── pid_controller_node.py         # Control PID optimizado (COMPONENTE CRÍTICO)
│   │   ├── path_generator_node.py         # Generación de trayectorias básicas
│   │   └── odometry_node.py               # Mejorado con fusión de datos
│   ├── resource/                   # Recursos del paquete
│   ├── test/                       # Pruebas unitarias
│   ├── package.xml
│   └── setup.*
│
├── detector_pkg/                   # Paquete de algoritmos de detección
│   ├── detector_pkg/
│   │   ├── __init__.py
│   │   ├── traffic_detector.py           # Algoritmo robusto de detección
│   │   ├── sign_detector.py              # NUEVO: Detector de señales de tráfico
│   │   ├── angular_error_node.py         # Cálculo optimizado de error angular para corrección en eje Z
│   │   └── debug_visualizer.py           # Visualización para debug
│   ├── resource/                   # Recursos del paquete
│   ├── test/                       # Pruebas unitarias
│   ├── package.xml
│   └── setup.*
│
├── install/                        # Archivos de instalación (generados)
├── log/                            # Registros de ejecución
├── scripts/                        # Scripts de configuración y utilidades
│   └── puzzlebot_pro.sh            # Configuración automática
│
├── .gitignore                      # Archivos ignorados por Git
├── LICENSE                         # Licencia MIT
└── README.md                       # Esta documentación
```

### Uso de colores en terminal

El código utiliza una clase `TerminalColors` para mejorar la visualización en terminal:

```python
class TerminalColors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
```

## ⚙️ Configuración Avanzada

### Personalización del entorno

Para configuraciones personalizadas, edita:
```bash
~/.puzzlebot/config.sh
```

### Parámetros ajustables

<details>
<summary><b>Controlador PID (Componente Crítico)</b></summary>
<ul>
  <li><b>Kp, Ki, Kd</b>: Modifica estos valores en el nodo <code>pid_controller_node.py</code> para ajustar la respuesta del robot</li>
  <li><b>Max velocidad</b>: Ajusta la velocidad máxima en diferentes controladores según las capacidades de tu hardware</li>
  <li><b>Frecuencia</b>: Cambia la frecuencia de actualización según tus necesidades de rendimiento y capacidad de procesamiento</li>
</ul>
</details>

<details>
<summary><b>Parámetros de visión</b></summary>
<ul>
  <li><b>Rangos HSV</b>: Personaliza los rangos de color en <code>traffic_detector.py</code> y <code>sign_detector.py</code> para diferentes condiciones de iluminación</li>
  <li><b>Umbral de detección</b>: Ajusta la sensibilidad de los algoritmos de detección</li>
  <li><b>Resolución</b>: Modifica la resolución de procesamiento para balancear rendimiento y precisión</li>
</ul>
</details>

<details>
<summary><b>Reconocimiento de señales</b></summary>
<ul>
  <li><b>Plantillas de señales</b>: El sistema incluye plantillas para las señales más comunes (STOP, GIVE WAY, direccionales, etc.)</li>
  <li><b>Umbral de coincidencia</b>: Ajusta la precisión del reconocimiento de señales</li>
  <li><b>Prioridad de señales</b>: Configura qué señales tienen precedencia cuando múltiples son detectadas</li>
</ul>
</details>

### Solución de problemas

<details>
<summary><b>No se detecta la red del PuzzleBot</b></summary>
<ul>
  <li>Asegúrate de que el PuzzleBot esté encendido</li>
  <li>Conéctate al hotspot WiFi del PuzzleBot</li>
  <li>Verifica en la configuración de red que tienes una IP en el rango 10.42.0.x</li>
  <li>Ejecuta <code>ifconfig</code> para verificar tus interfaces de red</li>
</ul>
</details>

<details>
<summary><b>No hay comunicación entre dispositivos</b></summary>
<ul>
  <li>Ejecuta <code>ping 10.42.0.2</code> para verificar la conexión</li>
  <li>Confirma que tanto la laptop como la Jetson tienen el mismo ROS_DOMAIN_ID</li>
  <li>Verifica que no haya firewalls bloqueando la comunicación: <code>sudo ufw status</code></li>
  <li>Reinicia el PuzzleBot si los problemas persisten</li>
</ul>
</details>

<details>
<summary><b>Errores en los nodos</b></summary>
<ul>
  <li>Verifica los logs en <code>~/.ros/log/</code> o en el directorio <code>log/</code> del workspace</li>
  <li>Usa <code>ros2 topic echo /topic_name</code> para verificar si se están publicando mensajes</li>
  <li>Comprueba que todos los paquetes están correctamente compilados con <code>colcon build</code></li>
  <li>Verifica dependencias con <code>rosdep check --from-paths src</code></li>
</ul>
</details>

## 👥 Equipo de desarrollo

<table>
  <tr>
    <td align="center">
      <a href="https://github.com/AldonDC">
        <img src="https://github.com/identicons/AldonDC.png" width="100px;" alt=""/><br />
        <sub><b>Alfonso Solís Díaz</b></sub>
      </a>
      <br />
      <sub></sub>
    </td>
  </tr>
</table>

## 🔍 Mejoras recientes

### Versión 1.3.0 (Mayo 2025)
- **Nuevo**: Implementación del nodo `sign_detector.py` para reconocimiento de señales de tráfico
- **Nuevo**: Controlador `sign_response_controller.py` para responder adecuadamente a las señales detectadas
- **Mejorado**: Controlador PID optimizado para navegación más precisa (componente crítico del sistema)
- **Optimizado**: Cálculo de error angular con corrección específica en el eje Z para seguimiento de líneas más limpio
- **Actualizado**: Integración de señales y semáforos en el sistema de control de navegación
- **Añadido**: Capacidad de detección de múltiples señales de tráfico simultáneamente

## 🙏 Agradecimientos

Este proyecto ha sido desarrollado en colaboración con [**Manchester Robotics**](https://manchester-robotics.com/), quienes proporcionaron la plataforma **PuzzleBot** y un valioso soporte técnico.  
Agradecemos profundamente su compromiso con la educación en robótica y su contribución significativa a este proyecto académico.

<div align="center">
  <a href="https://manchester-robotics.com/" target="_blank">
    <img src="https://github.com/user-attachments/assets/30f7bcf1-f9f1-4dc2-bea5-43de47b9a1da" alt="Manchester Robotics Logo" width="300px">
  </a>
</div>

## 📄 Licencia

Este proyecto está bajo la Licencia MIT - ver el archivo [LICENSE](LICENSE) para más detalles.

<div align="center">
  <br>
  <a href="#-puzzlebot-ros2-framework">⬆️ Volver arriba ⬆️</a>
</div>
