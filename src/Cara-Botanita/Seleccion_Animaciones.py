#!/usr/bin/env python3

# -------- Librerias ----------

import rospy
import serial
import time
import random
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# -------- Configurar comunicacion serial ----------

SERIAL_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200 
ser = None
last_gif_number = -1

# -------- Variables de deteccion, emociones y animaciones ----------

global current_detections     
global current_emotion        
global last_gif_change_time   
global last_vision_data_time  

current_detections = {"name": "none", "confidence": 0.0}
current_emotion = {"name": "neutral", "id": 5.0, "confidence": 1.0}

last_gif_change_time = time.time()
last_vision_data_time = time.time() 

manual_gif = 0 

# -------- Detecciones de la camara recibidas por ROS ----------

DETECTION_CLASSES = {
    0.0:"cora", 1.0: "face", 2.0: "fu", 3.0: "palm", 4.0: "peace-sign", 
    5.0: "phone", 6.0: "phone-camera", 7.0: "thumbs-down", 8.0: "thumbs-up"
}

# -------- Detecciones de emociones recibidas por ROS ----------

EMOTION_CLASSES = {
    1.0: "anger", 2.0: "disgust", 3.0: "fear", 
    4.0: "happy", 5.0: "neutral", 6.0: "sad", 7.0: "surprise"
}

# -------- Conectar detecciones de la camara a animaciones ----------

GIF_MAP = {
    "neutral": 1,         # BASE
    "disgust": 7,         # DISGUSTO
    "anger": 9,           # ENOJO
    "happy": 10,          # FELIZ
    "fear": 14,           # MIEDO
    "sad": 20,            # TRISTE
    "fu": 11,             # GROSERIA
    "phone": [15, 16, 17, 18],        # POSE ALEATORIA
    "phone-camera": [15, 16, 17, 18], # POSE ALEATORIA 
    "dormida": 8,         # DORMIDA
    "palm": 19,           # SERIEDAD
    "cora": 2,            # AMOR
}

# -------- Establecer prioridad de las detecciones ----------

def select_gif_logic():
    global current_detections, current_emotion, last_vision_data_time, manual_gif

    if manual_gif != 0:
        return manual_gif
        
    INACTIVITY_TIMEOUT = 15.0
    if (time.time() - last_vision_data_time) > INACTIVITY_TIMEOUT:
        return GIF_MAP["dormida"]

    det_name = current_detections["name"]
    det_conf = current_detections["confidence"]
    
    if det_conf > 0.55:
        if det_name in ["phone", "phone-camera"]:
            return random.choice(GIF_MAP.get(det_name))
            
        if det_name == "fu":
            return GIF_MAP["fu"]

        if det_name == "cora":
            return GIF_MAP["cora"]

        if det_name == "palm":
            return GIF_MAP["palm"]

    emotion_key = current_emotion["name"]
    gif_choice = GIF_MAP.get(emotion_key, GIF_MAP["neutral"])

    if isinstance(gif_choice, list):
        return random.choice(gif_choice)
        
    return gif_choice

# -------- Recibir mensajes por ROS de las detecciones ----------

def detections_callback(data):
    global current_detections, last_vision_data_time
    array = data.data
    
    if len(array) > 0:
        last_vision_data_time = time.time()
    
    BLOCK_SIZE = 6 
    if len(array) % BLOCK_SIZE != 0 or len(array) == 0:
        current_detections = {"name": "none", "confidence": 0.0}
        if len(array) > 0:
             rospy.logwarn("Detections array tiene un tamaño invalido.")
        return

    highest_conf_det = {"name": "none", "confidence": 0.0}
    
    for i in range(0, len(array), BLOCK_SIZE):
        det_id = array[i]
        confidence = array[i+1]
        if det_id in DETECTION_CLASSES and confidence > highest_conf_det["confidence"]:
            highest_conf_det["name"] = DETECTION_CLASSES[det_id]
            highest_conf_det["confidence"] = confidence
    current_detections = highest_conf_det

# -------- Recibir mensajes por ROS de las detecciones de emociones ----------

def emotions_callback(data):
    global current_emotion, last_vision_data_time
    array = data.data
    if len(array) > 0:
        last_vision_data_time = time.time()
    BLOCK_SIZE = 5
    if len(array) % BLOCK_SIZE != 0 or len(array) == 0:
        if len(array) > 0: 
            rospy.logwarn("Emotions array tiene un tamaño invalido.")
        return
        
    emotion_counts = {}
    
    for i in range(0, len(array), BLOCK_SIZE):
        emo_id = array[i]
        if emo_id in EMOTION_CLASSES:
            emo_name = EMOTION_CLASSES[emo_id]
            emotion_counts[emo_name] = emotion_counts.get(emo_name, 0) + 1
    dominant_emotion_name = "neutral"
    max_count = 0
    for emo_name, count in emotion_counts.items():
        if emo_name != "neutral" and count > max_count:
            max_count = count
            dominant_emotion_name = emo_name
        elif max_count == 0 and emo_name == "neutral" and count > 0:
            dominant_emotion_name = "neutral" 
    current_emotion["name"] = dominant_emotion_name

# -------- Recibir mensajes por ROS de las emociones enviadas por otros nodos de ROS ----------

def manual_status_callback(data):
    global manual_gif
    try:
        new_status = int(data.data) 
    except ValueError:
        rospy.logwarn("Received non-integer data on manual control topic.")
        return
    manual_gif = new_status
    rospy.loginfo(f"Estado manual actualizado a GIF: {manual_gif}")

# -------- Emocion neutral al detener el codigo ----------

def safe_shutdown():
    global ser
    if ser and ser.is_open:
        try:
            base_gif_id = GIF_MAP["neutral"]
            serial_output = f"{base_gif_id}\n"
            ser.write(serial_output.encode('utf-8'))
            rospy.loginfo(f"Comando de finalización enviado: {base_gif_id}.gif")
        except Exception as e:
            rospy.logwarn(f"Error al enviar comando base antes de cerrar serial: {e}")
        ser.close()
        
    rospy.loginfo("Nodo GIF Controller finalizado.")

def gif_controller_node():
    global ser, last_gif_number, last_gif_change_time
    if not rospy.core.is_initialized():
        rospy.init_node('gif_controller', anonymous=True)
    rospy.on_shutdown(safe_shutdown) 
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2) 
        rospy.loginfo(f"Puerto serial {SERIAL_PORT} conectado.")
    except serial.SerialException as e:
        rospy.logerr(f"No se pudo abrir el puerto serial {SERIAL_PORT}. Asegurate de que el ESP32 este conectado y el puerto sea correcto. Error: {e}")
        raise
        return

# -------- Suscribirse a topics de ROS ----------

    rospy.Subscriber('/detecciones_emociones', Float32MultiArray, emotions_callback)
    rospy.Subscriber('/detecciones_yolo', Float32MultiArray, detections_callback)
    rospy.Subscriber('/robot_status', Float32, manual_status_callback) 
    rospy.loginfo("Nodo GIF Controller iniciado y suscrito a topics.")

    rate = rospy.Rate(5)
    GIF_CHANGE_DELAY = 7.0

# -------- Seleccion de animacion a mostrar ----------

    while not rospy.is_shutdown():
        
        new_gif_number = select_gif_logic()
        current_time = time.time()
        if new_gif_number != last_gif_number and \
           (current_time - last_gif_change_time) > GIF_CHANGE_DELAY and \
           new_gif_number is not None:
            
            try:
                serial_output = f"{new_gif_number}\n"
                ser.write(serial_output.encode('utf-8'))
                last_gif_number = new_gif_number
                last_gif_change_time = current_time
                rospy.loginfo(f"Cambiando GIF a: {new_gif_number}.gif")
            except serial.SerialException as e:
                rospy.logerr(f"Error al escribir en serial: {e}. Deteniendo nodo.")
                rospy.signal_shutdown("Error de comunicacion serial.")
                break 
            except Exception as e:
                rospy.logwarn(f"Error inesperado al enviar serial: {e}. GIF intentado: {new_gif_number}")
        rate.sleep()

if __name__ == '__main__':
    try:
        gif_controller_node()
    except serial.SerialException:
        rospy.loginfo("El nodo termino debido a un error de conexion serial (problema de apertura del puerto).")
    except Exception as e:
        rospy.logerr(f"Excepcion inesperada en el nodo principal: {e}")