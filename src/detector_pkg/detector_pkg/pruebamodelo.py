import os
from ultralytics import YOLO
import sys

def load_and_inspect_model(model_path, model_name):
    """Carga un modelo YOLO y muestra sus clases"""
    print(f"\n{'='*60}")
    print(f"🔍 INSPECCIONANDO: {model_name}")
    print(f"📁 Ruta: {model_path}")
    print(f"{'='*60}")
    
    try:
        # Verificar si el archivo existe
        if not os.path.exists(model_path):
            print(f"❌ ERROR: Archivo '{model_path}' no encontrado")
            print(f"💡 Asegúrate de que el archivo esté en el directorio actual")
            return None
        
        # Cargar el modelo
        print("⏳ Cargando modelo...")
        model = YOLO(model_path)
        print("✅ Modelo cargado exitosamente")
        
        # Obtener información del modelo
        if hasattr(model, 'names') and model.names:
            class_names = model.names
            print(f"\n📋 CLASES DETECTADAS EN {model_name}:")
            print(f"📊 Total de clases: {len(class_names)}")
            print("-" * 50)
            
            # Mostrar todas las clases con índice
            for class_id, class_name in class_names.items():
                print(f"  {class_id:2d}: {class_name}")
            
            return class_names
        else:
            print("❌ No se pudieron obtener los nombres de las clases")
            return None
            
    except Exception as e:
        print(f"❌ ERROR al cargar modelo: {str(e)}")
        return None

def compare_models(primary_classes, direction_classes):
    """Compara las clases de ambos modelos"""
    print(f"\n{'='*60}")
    print("🔄 COMPARACIÓN DE MODELOS")
    print(f"{'='*60}")
    
    if primary_classes is None or direction_classes is None:
        print("❌ No se pueden comparar modelos - Error en la carga")
        return
    
    # Convertir a sets para comparación
    primary_set = set(primary_classes.values())
    direction_set = set(direction_classes.values())
    
    # Clases en común
    common_classes = primary_set.intersection(direction_set)
    
    # Clases únicas de cada modelo
    primary_unique = primary_set - direction_set
    direction_unique = direction_set - primary_set
    
    print(f"📊 ESTADÍSTICAS:")
    print(f"  ├─ Modelo Principal: {len(primary_classes)} clases")
    print(f"  ├─ Modelo Direcciones: {len(direction_classes)} clases")
    print(f"  ├─ Clases en común: {len(common_classes)}")
    print(f"  ├─ Solo en Principal: {len(primary_unique)}")
    print(f"  └─ Solo en Direcciones: {len(direction_unique)}")
    
    if common_classes:
        print(f"\n🔄 CLASES EN COMÚN:")
        for class_name in sorted(common_classes):
            print(f"  • {class_name}")
    
    if primary_unique:
        print(f"\n🔵 SOLO EN MODELO PRINCIPAL:")
        for class_name in sorted(primary_unique):
            print(f"  • {class_name}")
    
    if direction_unique:
        print(f"\n🟢 SOLO EN MODELO DIRECCIONES:")
        for class_name in sorted(direction_unique):
            print(f"  • {class_name}")

def suggest_hierarchy_setup(primary_classes, direction_classes):
    """Sugiere configuración de jerarquía"""
    print(f"\n{'='*60}")
    print("💡 SUGERENCIAS PARA JERARQUÍA")
    print(f"{'='*60}")
    
    if primary_classes is None or direction_classes is None:
        return
    
    # Identificar señales de dirección
    direction_keywords = ['turn', 'left', 'right', 'arrow']
    
    primary_directions = []
    direction_directions = []
    
    for class_name in primary_classes.values():
        if any(keyword in class_name.lower() for keyword in direction_keywords):
            primary_directions.append(class_name)
    
    for class_name in direction_classes.values():
        if any(keyword in class_name.lower() for keyword in direction_keywords):
            direction_directions.append(class_name)
    
    print("🎯 CONFIGURACIÓN RECOMENDADA:")
    print("\n🔵 MODELO PRINCIPAL (Modelo.pt) - Señales generales:")
    non_direction_classes = [name for name in primary_classes.values() 
                           if not any(keyword in name.lower() for keyword in direction_keywords)]
    
    for class_name in sorted(non_direction_classes):
        print(f"  ✅ {class_name}")
    
    if primary_directions:
        print("\n  ⚠️  Direcciones encontradas (considerar mover a Modelo3.pt):")
        for class_name in sorted(primary_directions):
            print(f"  ⚡ {class_name}")
    
    print("\n🟢 MODELO DIRECCIONES (Modelo3.pt) - Solo direcciones:")
    for class_name in sorted(direction_directions):
        print(f"  ✅ {class_name}")

def main():
    print("🚗 INSPECTOR DE MODELOS YOLO - PUZZLEBOT")
    print("🎯 Analizando modelos para sistema jerárquico")
    
    # Rutas de los modelos
    primary_model_path = "/home/alfonso/puzzlebot-ros2-control/src/detector_pkg/detector_pkg/Modelo.pt"      # Modelo principal
    direction_model_path = "/home/alfonso/puzzlebot-ros2-control/src/detector_pkg/detector_pkg/Modelo3.pt"   # Modelo de direcciones
    
    # Cargar e inspeccionar modelo principal
    primary_classes = load_and_inspect_model(
        primary_model_path, 
        "MODELO PRINCIPAL (Modelo.pt)"
    )
    
    # Cargar e inspeccionar modelo de direcciones
    direction_classes = load_and_inspect_model(
        direction_model_path, 
        "MODELO DIRECCIONES (Modelo3.pt)"
    )
    
    # Comparar modelos
    compare_models(primary_classes, direction_classes)
    
    # Sugerir configuración de jerarquía
    suggest_hierarchy_setup(primary_classes, direction_classes)
    
    print(f"\n{'='*60}")
    print("✅ INSPECCIÓN COMPLETADA")
    print("💡 Usa esta información para configurar tu sistema jerárquico")
    print(f"{'='*60}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n🛑 Inspección cancelada por usuario")
    except Exception as e:
        print(f"\n❌ ERROR: {str(e)}")
        print("💡 Asegúrate de tener los archivos Modelo.pt y Modelo3.pt en el directorio actual")
    
    input("\n📱 Presiona Enter para salir...")