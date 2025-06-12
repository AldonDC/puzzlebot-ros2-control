#!/bin/bash
# ================================================================
# 🤖 PUZZLEBOT - CONFIGURACIÓN PROFESIONAL
# Script de configuración automática para PuzzleBot
# Detecta automáticamente la IP y configura el entorno ROS 2
# ================================================================

# Colores y estilos para terminal
RESET="\033[0m"
BOLD="\033[1m"
RED="\033[1;31m"
GREEN="\033[1;32m"
YELLOW="\033[1;33m"
BLUE="\033[1;34m"
PURPLE="\033[1;35m"
CYAN="\033[1;36m"
WHITE="\033[1;37m"

# Limpiar pantalla e imprimir banner
clear
echo -e "${BOLD}${BLUE}╔═════════════════════════════════════════════════════════╗${RESET}"
echo -e "${BOLD}${BLUE}║                                                         ║${RESET}"
echo -e "${BOLD}${BLUE}║  ${GREEN}🤖 PUZZLEBOT - CONFIGURACIÓN PROFESIONAL${BLUE}             ║${RESET}"
echo -e "${BOLD}${BLUE}║                                                         ║${RESET}"
echo -e "${BOLD}${BLUE}╚═════════════════════════════════════════════════════════╝${RESET}"
echo 

# Variables globales
PUZZLEBOT_NETWORK="10.42.0"
PUZZLEBOT_DEFAULT_IP="10.42.0.2"
PUZZLEBOT_USER="puzzlebot"
DEFAULT_ROS_DOMAIN_ID=0

# Función para imprimir mensajes con formato
print_status() {
    case $1 in
        "info")
            echo -e "${CYAN}ℹ️  ${RESET}$2"
            ;;
        "success")
            echo -e "${GREEN}✅ ${RESET}$2"
            ;;
        "warning")
            echo -e "${YELLOW}⚠️  ${RESET}$2"
            ;;
        "error")
            echo -e "${RED}❌ ${RESET}$2"
            ;;
        "step")
            echo -e "${BOLD}${PURPLE}$2${RESET}"
            ;;
        *)
            echo -e "$2"
            ;;
    esac
}

# Función para mostrar título de sección
section_title() {
    echo
    echo -e "${BOLD}${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    echo -e "${BOLD}${WHITE}  $1${RESET}"
    echo -e "${BOLD}${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    echo
}

# Detectar la interfaz y la IP del usuario
detect_puzzlebot_network() {
    section_title "DETECCIÓN DE RED"
    
    print_status "step" "Buscando conexión a red PuzzleBot (${PUZZLEBOT_NETWORK}.x)..."
    
    # Buscar interfaces con IPs en la red 10.42.0.x
    PUZZLEBOT_INTERFACE=$(ip -o -4 addr show | grep -o "${PUZZLEBOT_NETWORK}\.[0-9]\{1,3\}" -B1 | grep -o "wl[^ ]*" | head -1)
    PUZZLEBOT_LOCAL_IP=$(ip -o -4 addr show | grep -o "${PUZZLEBOT_NETWORK}\.[0-9]\{1,3\}" | head -1)
    
    if [ -z "$PUZZLEBOT_INTERFACE" ] || [ -z "$PUZZLEBOT_LOCAL_IP" ]; then
        print_status "error" "No se detectó conexión a la red PuzzleBot"
        
        print_status "info" "Interfaces de red disponibles:"
        ip -o -4 addr show | grep -v "127.0.0.1" | awk '{print "   - " $2 ": " $4}'
        
        echo
        print_status "warning" "Para continuar necesitas:"
        echo "   1. Conectarte a la red WiFi del PuzzleBot"
        echo "   2. Ejecutar este script nuevamente"
        echo
        
        read -p "$(echo -e ${YELLOW}⚙️  ¿Continuar con configuración manual? [s/N]: ${RESET})" CONTINUE_MANUAL
        
        if [[ ! $CONTINUE_MANUAL =~ ^[Ss]$ ]]; then
            print_status "info" "Configuración cancelada. Conéctate a la red PuzzleBot primero."
            exit 0
        fi
        
        echo
        read -p "$(echo -e ${CYAN}📱 Introduce tu IP en la red PuzzleBot: ${RESET})" PUZZLEBOT_LOCAL_IP
        read -p "$(echo -e ${CYAN}🔌 Introduce el nombre de tu interfaz de red: ${RESET})" PUZZLEBOT_INTERFACE
    else
        print_status "success" "Red PuzzleBot detectada"
        print_status "info" "Interfaz: ${CYAN}${PUZZLEBOT_INTERFACE}${RESET}"
        print_status "info" "Tu IP: ${CYAN}${PUZZLEBOT_LOCAL_IP}${RESET}"
    fi
    
    USER_MACHINE=$(hostname)
    
    echo
    print_status "step" "¿Es esta información correcta?"
    echo -e "   ${YELLOW}Interfaz:${RESET} ${PUZZLEBOT_INTERFACE}"
    echo -e "   ${YELLOW}Tu IP:${RESET} ${PUZZLEBOT_LOCAL_IP}"
    echo -e "   ${YELLOW}Computadora:${RESET} ${USER_MACHINE}"
    echo
    
    read -p "$(echo -e ${YELLOW}⚙️  Confirmar [S/n]: ${RESET})" CONFIRM
    
    if [[ $CONFIRM =~ ^[Nn]$ ]]; then
        echo
        read -p "$(echo -e ${CYAN}📱 Introduce tu IP correcta: ${RESET})" PUZZLEBOT_LOCAL_IP
        read -p "$(echo -e ${CYAN}🔌 Introduce el nombre correcto de tu interfaz: ${RESET})" PUZZLEBOT_INTERFACE
    fi
}

# Verificar conexión con el PuzzleBot
verify_puzzlebot_connection() {
    section_title "VERIFICACIÓN DE CONEXIÓN"
    
    print_status "step" "Verificando conexión con PuzzleBot (${PUZZLEBOT_DEFAULT_IP})..."
    
    # Preguntar si la IP por defecto del PuzzleBot es correcta
    read -p "$(echo -e ${YELLOW}⚙️  La IP del PuzzleBot es ${CYAN}${PUZZLEBOT_DEFAULT_IP}${YELLOW}? [S/n]: ${RESET})" CONFIRM_IP
    
    if [[ $CONFIRM_IP =~ ^[Nn]$ ]]; then
        echo
        read -p "$(echo -e ${CYAN}📱 Introduce la IP correcta del PuzzleBot: ${RESET})" PUZZLEBOT_DEFAULT_IP
    fi
    
    # Verificar conexión con ping
    if ping -c 1 -W 2 $PUZZLEBOT_DEFAULT_IP &> /dev/null; then
        print_status "success" "Conexión exitosa con PuzzleBot"
        
        # Mostrar información adicional si es posible
        SSH_CHECK=$(ssh -o ConnectTimeout=2 -o BatchMode=yes ${PUZZLEBOT_USER}@${PUZZLEBOT_DEFAULT_IP} "echo OK" 2>&1)
        
        if [[ $SSH_CHECK == "OK" ]]; then
            print_status "success" "SSH configurado correctamente"
            
            # Obtener información del PuzzleBot
            JETSON_HOSTNAME=$(ssh -o ConnectTimeout=2 ${PUZZLEBOT_USER}@${PUZZLEBOT_DEFAULT_IP} "hostname" 2>/dev/null)
            JETSON_KERNEL=$(ssh -o ConnectTimeout=2 ${PUZZLEBOT_USER}@${PUZZLEBOT_DEFAULT_IP} "uname -r" 2>/dev/null)
            JETSON_MEM=$(ssh -o ConnectTimeout=2 ${PUZZLEBOT_USER}@${PUZZLEBOT_DEFAULT_IP} "free -h | grep Mem | awk '{print \$3\"/\"\$2}'" 2>/dev/null)
            
            echo
            print_status "info" "Información del PuzzleBot:"
            echo -e "   ${YELLOW}Hostname:${RESET} ${JETSON_HOSTNAME}"
            echo -e "   ${YELLOW}Kernel:${RESET} ${JETSON_KERNEL}"
            echo -e "   ${YELLOW}Memoria:${RESET} ${JETSON_MEM}"
        else
            print_status "warning" "SSH no configurado o requiere contraseña"
            print_status "info" "Comando para conectar por SSH: ${YELLOW}ssh ${PUZZLEBOT_USER}@${PUZZLEBOT_DEFAULT_IP}${RESET}"
        fi
    else
        print_status "error" "No se pudo conectar con el PuzzleBot"
        print_status "warning" "Posibles causas:"
        echo "   - El PuzzleBot no está encendido"
        echo "   - La IP del PuzzleBot no es ${PUZZLEBOT_DEFAULT_IP}"
        echo "   - Problemas con la conexión WiFi"
        echo
        
        read -p "$(echo -e ${YELLOW}⚙️  ¿Continuar de todos modos? [s/N]: ${RESET})" CONTINUE_ANYWAY
        
        if [[ ! $CONTINUE_ANYWAY =~ ^[Ss]$ ]]; then
            print_status "info" "Configuración cancelada. Verifica la conexión e intenta nuevamente."
            exit 0
        fi
    fi
}

# Configurar el entorno ROS
configure_ros_environment() {
    section_title "CONFIGURACIÓN ROS 2"
    
    print_status "step" "Configurando variables de entorno ROS 2..."
    
    # Preguntar por el ROS_DOMAIN_ID
    read -p "$(echo -e ${YELLOW}⚙️  ROS_DOMAIN_ID [${DEFAULT_ROS_DOMAIN_ID}]: ${RESET})" ROS_DOMAIN_ID
    ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-${DEFAULT_ROS_DOMAIN_ID}}
    
    # Crear directorio de configuración si no existe
    CONFIG_DIR="${HOME}/.puzzlebot"
    mkdir -p "${CONFIG_DIR}"
    
    # Crear archivo de configuración
    CONFIG_FILE="${CONFIG_DIR}/config.sh"
    
    cat > "${CONFIG_FILE}" << EOF
#!/bin/bash
# ========================================================
# 🤖 PUZZLEBOT - CONFIGURACIÓN AUTOMÁTICA
# Generado el $(date)
# ========================================================

# Colores para terminal
RESET="\033[0m"
BOLD="\033[1m"
RED="\033[1;31m"
GREEN="\033[1;32m"
YELLOW="\033[1;33m"
BLUE="\033[1;34m"
CYAN="\033[1;36m"
WHITE="\033[1;37m"

# Configuración ROS 2
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
export ROS_IP=${PUZZLEBOT_LOCAL_IP}

# Limpiar pantalla y mostrar banner
clear
echo -e "\${BOLD}\${BLUE}╔═════════════════════════════════════════════════════════╗\${RESET}"
echo -e "\${BOLD}\${BLUE}║                                                         ║\${RESET}"
echo -e "\${BOLD}\${BLUE}║  \${GREEN}🤖 PUZZLEBOT - CONECTADO Y LISTO\${BLUE}                   ║\${RESET}"
echo -e "\${BOLD}\${BLUE}║                                                         ║\${RESET}"
echo -e "\${BOLD}\${BLUE}╚═════════════════════════════════════════════════════════╝\${RESET}"
echo

# Información de configuración
echo -e "\${BOLD}\${WHITE}CONFIGURACIÓN ROS 2\${RESET}"
echo -e "\${YELLOW}ROS_DOMAIN_ID=\${RESET}${ROS_DOMAIN_ID}"
echo -e "\${YELLOW}ROS_IP=\${RESET}${PUZZLEBOT_LOCAL_IP}"
echo

# Información de red
echo -e "\${BOLD}\${WHITE}INFORMACIÓN DE RED\${RESET}"
echo -e "\${YELLOW}Tu interfaz:\${RESET} ${PUZZLEBOT_INTERFACE}"
echo -e "\${YELLOW}Tu IP:\${RESET} ${PUZZLEBOT_LOCAL_IP}"
echo -e "\${YELLOW}IP PuzzleBot:\${RESET} ${PUZZLEBOT_DEFAULT_IP}"
echo -e "\${YELLOW}Usuario PuzzleBot:\${RESET} ${PUZZLEBOT_USER}"
echo

# Verificar conexión
echo -e "\${BOLD}\${WHITE}ESTADO DE CONEXIÓN\${RESET}"
if ping -c 1 -W 2 ${PUZZLEBOT_DEFAULT_IP} &> /dev/null; then
    echo -e "\${GREEN}✅ PuzzleBot conectado\${RESET}"
    echo -e "\${YELLOW}SSH:\${RESET} ssh ${PUZZLEBOT_USER}@${PUZZLEBOT_DEFAULT_IP}"
    
    # Verificar nodos ROS 2 si están disponibles
    if command -v ros2 &> /dev/null; then
        echo
        echo -e "\${BOLD}\${WHITE}NODOS ROS 2 ACTIVOS\${RESET}"
        NODES=\$(ros2 node list 2>/dev/null)
        if [ -z "\$NODES" ]; then
            echo -e "\${YELLOW}No se detectaron nodos ROS 2 activos\${RESET}"
        else
            echo -e "\$NODES" | sed 's/^/  /'
        fi
        
        echo
        echo -e "\${BOLD}\${WHITE}TÓPICOS ROS 2 DISPONIBLES\${RESET}"
        TOPICS=\$(ros2 topic list 2>/dev/null)
        if [ -z "\$TOPICS" ]; then
            echo -e "\${YELLOW}No se detectaron tópicos ROS 2\${RESET}"
        else
            echo -e "\$TOPICS" | grep -v "^/parameter" | grep -v "^/rosout" | sed 's/^/  /'
        fi
    fi
else
    echo -e "\${RED}❌ No se detecta el PuzzleBot\${RESET}"
    echo -e "\${YELLOW}Verifica la conexión e intenta nuevamente\${RESET}"
fi

echo
echo -e "\${BOLD}\${WHITE}COMANDOS ÚTILES\${RESET}"
echo -e "  \${CYAN}ros2 topic list\${RESET} - Ver tópicos disponibles"
echo -e "  \${CYAN}ros2 node list\${RESET} - Ver nodos en ejecución"
echo -e "  \${CYAN}ros2 topic echo <topic>\${RESET} - Ver mensajes en un tópico"
echo -e "  \${CYAN}ros2 interface show <msg_type>\${RESET} - Ver estructura de un mensaje"
echo
echo -e "\${GREEN}🚀 PuzzleBot listo para trabajar\${RESET}"
EOF

    chmod +x "${CONFIG_FILE}"
    
    # Crear alias
    if grep -q "alias puzzlebot=" ~/.bashrc; then
        # Actualizar alias existente
        sed -i "s|alias puzzlebot=.*|alias puzzlebot='source ${CONFIG_FILE}'|" ~/.bashrc
        print_status "success" "Alias 'puzzlebot' actualizado en ~/.bashrc"
    else
        # Crear nuevo alias
        echo "" >> ~/.bashrc
        echo "# Alias para PuzzleBot (añadido el $(date))" >> ~/.bashrc
        echo "alias puzzlebot='source ${CONFIG_FILE}'" >> ~/.bashrc
        print_status "success" "Alias 'puzzlebot' añadido a ~/.bashrc"
    fi
    
    print_status "success" "Configuración ROS 2 completada"
    print_status "info" "Para activar la configuración:"
    echo -e "   ${CYAN}source ${CONFIG_FILE}${RESET}"
    echo -e "   o simplemente escribe ${CYAN}puzzlebot${RESET} en una nueva terminal"
}

# Crear herramientas adicionales
create_tools() {
    section_title "HERRAMIENTAS ADICIONALES"
    
    print_status "step" "Creando herramientas útiles..."
    
    # Crear script para monitoreo de tópicos
    MONITOR_SCRIPT="${HOME}/.puzzlebot/monitor.sh"
    
    cat > "${MONITOR_SCRIPT}" << EOF
#!/bin/bash
# PuzzleBot Monitor - Herramienta para monitorear tópicos ROS 2

RESET="\033[0m"
BOLD="\033[1m"
GREEN="\033[1;32m"
YELLOW="\033[1;33m"
BLUE="\033[1;34m"

if [ ! -f "${CONFIG_DIR}/config.sh" ]; then
    echo -e "\033[1;31mError: Debes configurar PuzzleBot primero\033[0m"
    echo -e "Ejecuta: puzzlebot_pro.sh"
    exit 1
fi

source "${CONFIG_DIR}/config.sh" > /dev/null

clear
echo -e "\${BOLD}\${BLUE}╔════════════════════════════════════════╗\${RESET}"
echo -e "\${BOLD}\${BLUE}║  \${GREEN}🤖 PUZZLEBOT - MONITOR\${BLUE}             ║\${RESET}"
echo -e "\${BOLD}\${BLUE}╚════════════════════════════════════════╝\${RESET}"
echo

# Obtener lista de tópicos
TOPICS=\$(ros2 topic list 2>/dev/null | grep -v "^/parameter" | grep -v "^/rosout")

if [ -z "\$TOPICS" ]; then
    echo -e "\${YELLOW}No se detectaron tópicos ROS 2\${RESET}"
    exit 0
fi

# Mostrar y numerar tópicos
echo -e "\${BOLD}Tópicos disponibles:\${RESET}"
IFS=\$'\n'
i=1
for topic in \$TOPICS; do
    echo -e "  \${YELLOW}\${i}.\${RESET} \${topic}"
    i=\$((i+1))
done
echo

# Solicitar selección
read -p "Selecciona un tópico para monitorear (número): " SELECTION

if [[ \$SELECTION =~ ^[0-9]+\$ ]]; then
    i=1
    for topic in \$TOPICS; do
        if [ \$i -eq \$SELECTION ]; then
            # Obtener tipo de mensaje
            MSG_TYPE=\$(ros2 topic info \$topic 2>/dev/null | grep "Type" | awk '{print \$2}')
            
            echo -e "\${BOLD}Monitoreando tópico: \${GREEN}\${topic}\${RESET} (\${MSG_TYPE})"
            echo -e "Presiona Ctrl+C para salir\\n"
            
            # Monitorear tópico
            ros2 topic echo \$topic
            exit 0
        fi
        i=\$((i+1))
    done
    
    echo -e "\${YELLOW}Selección inválida\${RESET}"
else
    echo -e "\${YELLOW}Selección inválida\${RESET}"
fi
EOF

    chmod +x "${MONITOR_SCRIPT}"
    
    # Crear alias para el monitor
    if ! grep -q "alias puzzlemon=" ~/.bashrc; then
        echo "alias puzzlemon='${MONITOR_SCRIPT}'" >> ~/.bashrc
        print_status "success" "Alias 'puzzlemon' añadido para monitorear tópicos"
    fi
    
    print_status "success" "Herramientas adicionales creadas"
}

# Función para finalizar la instalación
finalize_installation() {
    section_title "INSTALACIÓN COMPLETADA"
    
    print_status "success" "¡Configuración de PuzzleBot completada con éxito!"
    print_status "info" "A partir de ahora, simplemente escribe:"
    echo -e "   ${CYAN}puzzlebot${RESET} - Para activar la configuración"
    echo -e "   ${CYAN}puzzlemon${RESET} - Para monitorear tópicos"
    
    echo
    print_status "step" "¿Deseas activar la configuración ahora?"
    read -p "$(echo -e ${YELLOW}⚙️  Activar [S/n]: ${RESET})" ACTIVATE_NOW
    
    if [[ ! $ACTIVATE_NOW =~ ^[Nn]$ ]]; then
        source "${CONFIG_DIR}/config.sh"
    else
        echo
        print_status "info" "Para activar más tarde, escribe: ${CYAN}puzzlebot${RESET}"
        echo -e "${BOLD}${GREEN}¡Listo para trabajar con PuzzleBot!${RESET} 🚀"
    fi
}

# Función principal
main() {
    detect_puzzlebot_network
    verify_puzzlebot_connection
    configure_ros_environment
    create_tools
    finalize_installation
}

# Ejecutar función principal
main