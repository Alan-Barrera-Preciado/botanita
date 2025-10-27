#!/usr/bin/env python3

import rospy
import serial
import time
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry # Usaremos Odometría como ejemplo para la velocidad
from geometry_msgs.msg import Twist # También podemos usar Twist si es más conveniente

# ===============================================
# CONFIGURACIÓN SERIAL Y ESTADO GLOBAL
# ===============================================

# ⚠️ Configuración de Serial
SERIAL_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200 
ser = None
last_gif_number = -1

# Variables de Estado Globales
global current_detections # Almacena la detección con mayor prioridad/confianza
global current_emotion    # Almacena la emoción dominante
global robot_speed        # Velocidad lineal (obtenida de odometría/Twist)

current_detections = {"name": "none", "confidence": 0.0}
current_emotion = {"name": "neutral", "id": 6.0, "confidence": 1.0}
robot_speed = 0.0

# -------------------------------------------------------------
# Mapeos de ID a Clases
# -------------------------------------------------------------

# Mapeo de IDs de Detecciones (Ajusta estos IDs a los que realmente manda tu sistema YOLO)
DETECTION_CLASSES = {
    1.0: "face", 2.0: "fu", 3.0: "palm", 4.0: "peace-sign", 
    5.0: "phone", 6.0: "phone-camera", 7.0: "thumbs-down", 8.0: "thumbs-up"
}

# Mapeo de IDs de Emociones
EMOTION_CLASSES = {
    1.0: "anger", 2.0: "content", 3.0: "disgust", 4.0: "fear",
    5.0: "happy", 6.0: "neutral", 7.0: "sad", 8.0: "surprise"
}

# Mapeo de Clase a ID de GIF (AJUSTA ESTOS NÚMEROS A TUS ARCHIVOS GIF)
GIF_MAP = {
    "neutral": 1,
    "content": 2, "happy": 2, # Mismo GIF para contento y feliz
    "surprise": 3,
    "anger": 4,
    "disgust": 5,
    "fear": 6,
    "sad": 9,
    
    # Gestos/Objetos con Alta Prioridad
    "fu": 10,
    "phone": 11, "phone-camera": 11,
    "thumbs-up": 12,
    "thumbs-down": 13,
    
    # Estados de Movimiento (Modificadores)
    "moving_neutral": 7,
}

# -------------------------------------------------------------
# LÓGICA DE SELECCIÓN DE GIF CON PRIORIDAD
# -------------------------------------------------------------

def select_gif_logic():
    """
    Determina el número de GIF basado en la prioridad: Gesto > Emoción > Velocidad.
    """
    global current_detections, current_emotion, robot_speed

    # 1. PRIORIDAD ALTA: GESTOS Y OBJETOS ESPECÍFICOS (Solo si la confianza es alta)
    det_name = current_detections["name"]
    det_conf = current_detections["confidence"]

    # Mapear los nombres de alta prioridad
    if det_conf > 0.7: # Umbral de confianza
        if det_name in ["fu", "thumbs-up", "thumbs-down", "phone", "phone-camera"]:
            return GIF_MAP.get(det_name, GIF_MAP["neutral"])


    # 2. MODIFICADOR DE VELOCIDAD
    emotion_key = current_emotion["name"]
    
    if abs(robot_speed) > 0.1: # El robot se está moviendo
        if emotion_key == "neutral":
            return GIF_MAP.get("moving_neutral", GIF_MAP["neutral"])
        # Se pueden añadir otras reglas de movimiento aquí (ej. moving_happy)
    
    # 3. PRIORIDAD BAJA: EMOCIÓN DOMINANTE
    return GIF_MAP.get(emotion_key, GIF_MAP["neutral"])

# ===============================================
# CALLBACKS DE SUSCRIPCIÓN Y DECODIFICACIÓN
# ===============================================

def detections_callback(data):
    """
    Callback para el topic de detecciones de YOLO.
    Decodifica el array: [ID, Confianza, x1, y1, x2, y2, ID, Confianza, ...]
    """
    global current_detections
    
    # La data.data es el array de flotantes
    array = data.data
    
    # El tamaño de cada bloque de detección es 6: [ID, Confianza, x1, y1, x2, y2]
    BLOCK_SIZE = 6
    if len(array) % BLOCK_SIZE != 0:
        rospy.logwarn("Detections array tiene un tamaño inválido.")
        current_detections = {"name": "none", "confidence": 0.0}
        return

    highest_conf_det = {"name": "none", "confidence": 0.0}
    
    for i in range(0, len(array), BLOCK_SIZE):
        det_id = array[i]
        confidence = array[i+1]
        
        # Mapear el ID a la clase y priorizar por confianza
        if det_id in DETECTION_CLASSES and confidence > highest_conf_det["confidence"]:
             highest_conf_det["name"] = DETECTION_CLASSES[det_id]
             highest_conf_det["confidence"] = confidence
             
    current_detections = highest_conf_det
    # rospy.loginfo(f"Detección dominante: {current_detections}")


def emotions_callback(data):
    """
    Callback para el topic de emociones.
    Decodifica el array: [ID, x1, y1, x2, y2, ID, x1, y1, x2, y2, ...]
    Prioriza la emoción más frecuente (o la no-neutral más frecuente).
    """
    global current_emotion
    
    array = data.data
    
    # El tamaño de cada bloque de emoción es 5: [ID, x1, y1, x2, y2]
    BLOCK_SIZE = 5
    if len(array) % BLOCK_SIZE != 0:
        rospy.logwarn("Emotions array tiene un tamaño inválido.")
        current_emotion = {"name": "neutral", "id": 6.0, "confidence": 1.0}
        return
        
    emotion_counts = {}
    
    for i in range(0, len(array), BLOCK_SIZE):
        emo_id = array[i]
        
        if emo_id in EMOTION_CLASSES:
            emo_name = EMOTION_CLASSES[emo_id]
            emotion_counts[emo_name] = emotion_counts.get(emo_name, 0) + 1

    # Determinar la emoción dominante
    dominant_emotion_name = "neutral"
    max_count = 0
    
    for emo_name, count in emotion_counts.items():
        # Damos preferencia a emociones no-neutrales
        if emo_name != "neutral" and count > max_count:
            max_count = count
            dominant_emotion_name = emo_name
        # Si no hay otras emociones detectadas, y hay detecciones neutrales, se queda neutral
        elif max_count == 0 and emo_name == "neutral" and count > 0:
            dominant_emotion_name = "neutral" 
            
    # Solo actualizamos si se detectó algo, sino mantenemos el estado anterior
    if dominant_emotion_name != "neutral" or len(emotion_counts) > 0:
         current_emotion["name"] = dominant_emotion_name
    
    # rospy.loginfo(f"Emoción dominante: {current_emotion['name']}")


def speed_callback(data):
    """
    Callback para el topic de velocidad. Asumo que se usa nav_msgs/Odometry o geometry_msgs/Twist.
    Si usas Odometry: data.twist.twist.linear.x
    Si usas Twist: data.linear.x
    """
    global robot_speed
    
    # Asumo que recibes geometry_msgs/Twist (comando o estimación simple)
    # Si recibes Odometry, cambia a data.twist.twist.linear.x
    robot_speed = abs(data.linear.x)
    # rospy.loginfo(f"Velocidad recibida: {robot_speed}")


# ===============================================
# NODO PRINCIPAL Y BUCLE DE SERIAL
# ===============================================

def gif_controller_node():
    global ser, last_gif_number
    
    rospy.init_node('gif_controller', anonymous=True)
    
    # 1. Configuración de la comunicación serial
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2) 
        rospy.loginfo(f"Puerto serial {SERIAL_PORT} conectado.")
    except serial.SerialException as e:
        rospy.logerr(f"No se pudo abrir el puerto serial {SERIAL_PORT}. Asegúrate de que el ESP32 esté conectado y el puerto sea correcto. Error: {e}")
        return

    # 2. Suscripciones a topics
    rospy.Subscriber('/detecciones_emociones', Float32MultiArray, emotions_callback)
    rospy.Subscriber('/detecciones_yolo', Float32MultiArray, detections_callback)
    
    # ⚠️ AJUSTA ESTO: Usamos geometry_msgs/Twist como ejemplo, si usas Odometría, cambia el tipo y topic.
    rospy.Subscriber('/cmd_vel', Twist, speed_callback) 
    
    rospy.loginfo("Nodo GIF Controller iniciado y suscrito a topics.")
    
    # 3. Bucle de control (publicación serial)
    rate = rospy.Rate(10) # 10 Hz para un control responsivo
    while not rospy.is_shutdown():
        
        new_gif_number = select_gif_logic()
        
        # Enviar al ESP32 solo si el GIF ha cambiado y si el número es válido
        if new_gif_number != last_gif_number and new_gif_number is not None:
            try:
                # El ESP32 espera el número seguido de un salto de línea (\n)
                serial_output = f"{new_gif_number}\n"
                ser.write(serial_output.encode('utf-8'))
                last_gif_number = new_gif_number
                rospy.loginfo(f"Cambiando GIF a: {new_gif_number}.gif")
            except serial.SerialException as e:
                rospy.logerr(f"Error al escribir en serial: {e}")
            except Exception as e:
                rospy.logwarn(f"Error al enviar serial: {e}. GIF intentado: {new_gif_number}")


        rate.sleep()

if __name__ == '__main__':
    try:
        gif_controller_node()
    except rospy.ROSInterruptException:
        if ser and ser.is_open:
            ser.close()
        rospy.loginfo("Nodo GIF Controller finalizado.")
