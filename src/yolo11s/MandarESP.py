#!/usr/bin/env python3

import rospy
import serial
import time
import random # Necesario para la selección aleatoria de GIFs
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist # Usado solo para definir el tipo si fuera necesario, pero la suscripción se elimina
from std_msgs.msg import Float32 # Necesario para el topic de control manual

# ===============================================
# CONFIGURACIÓN SERIAL Y ESTADO GLOBAL
# ===============================================

# ⚠️ Configuración de Serial
SERIAL_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200 
ser = None
last_gif_number = -1

# Variables de Estado Globales
global current_detections     # Almacena la detección con mayor prioridad/confianza
global current_emotion        # Almacena la emoción dominante
global last_gif_change_time   # Tiempo de la última vez que se envió un GIF
global last_vision_data_time  # Tiempo de la última vez que se recibió data de /detecciones_yolo o /detecciones_emociones
global manual_gif             # Control manual externo

current_detections = {"name": "none", "confidence": 0.0}
# NOTA: Neutral ahora es ID 5.0 según las nuevas clases
current_emotion = {"name": "neutral", "id": 5.0, "confidence": 1.0}

# Inicializamos el tiempo para permitir el primer cambio de GIF
last_gif_change_time = time.time()
last_vision_data_time = time.time() 

# Variable de control manual (0: control por visión, >0: número de GIF manual)
manual_gif = 0 

# -------------------------------------------------------------
# Mapeos de ID a Clases y GIF
# -------------------------------------------------------------

# Mapeo de IDs de Detecciones (Ajusta estos IDs a los que realmente manda tu sistema YOLO)
DETECTION_CLASSES = {
    1.0: "face", 2.0: "fu", 3.0: "palm", 4.0: "peace-sign", 
    5.0: "phone", 6.0: "phone-camera", 7.0: "thumbs-down", 8.0: "thumbs-up"
}

# Mapeo de IDs de Emociones (¡ACTUALIZADO SEGÚN TU SOLICITUD!)
EMOTION_CLASSES = {
    1.0: "anger", 2.0: "disgust", 3.0: "fear", 
    4.0: "happy", 5.0: "neutral", 6.0: "sad", 7.0: "surprise"
}

# Mapeo de Clase a ID de GIF (Ajustado a tu nueva lista)
# Usaremos listas para los GIFs con múltiples versiones
GIF_MAP = {
    "neutral": 1,         # BASE 01
    "disgust": [8, 9],    # DISGUSTOV1/V2 08, 09
    "anger": 11,          # ENOJO 11
    "happy": 12,          # FELIZ 12 (NOTA: "content" se eliminó de aquí)
    "fear": 18,           # MIEDO 18
    "sad": 24,            # TRISTE 24
    
    # Gestos/Objetos con Alta Prioridad
    "fu": [13, 14],       # GROSERIAV1/V2 13, 14
    "phone": [19, 20, 21, 22, 23],        # FOTO_X (Celular)
    "phone-camera": [19, 20, 21, 22, 23], # FOTO_X (Celular) 
    
    # Estados especiales (No automáticos)
    "dormida": 10,        # DORMIDA 10 (Usado por lógica de inactividad)
}

# -------------------------------------------------------------
# LÓGICA DE SELECCIÓN DE GIF CON PRIORIDAD
# -------------------------------------------------------------

def select_gif_logic():
    """
    Determina el número de GIF basado en la prioridad: Manual > Inactividad > Celular > Gesto > Emoción.
    """
    global current_detections, current_emotion, last_vision_data_time, manual_gif

    # 0. PRIORIDAD MÁXIMA: CONTROL MANUAL EXTERNO
    if manual_gif != 0:
        return manual_gif
        
    # 1. PRIORIDAD: INACTIVIDAD (DORMIDA)
    INACTIVITY_TIMEOUT = 15.0 # Segundos
    if (time.time() - last_vision_data_time) > INACTIVITY_TIMEOUT:
        return GIF_MAP["dormida"]

    # 2. PRIORIDAD ALTA: CELULAR (phone/phone-camera)
    det_name = current_detections["name"]
    det_conf = current_detections["confidence"]
    
    if det_conf > 0.6: # Umbral de confianza
        if det_name in ["phone", "phone-camera"]:
            # Elige un GIF aleatorio entre 19, 20, 21, 22, 23
            return random.choice(GIF_MAP.get(det_name))
            
    # 3. PRIORIDAD MEDIA: GESTOS (Dedo Medio)
        if det_name == "fu":
            # Elige un GIF aleatorio entre 13 y 14
            return random.choice(GIF_MAP.get(det_name))
            
    # 4. PRIORIDAD BAJA: EMOCIÓN DOMINANTE (IMITACIÓN)
    emotion_key = current_emotion["name"]
    gif_choice = GIF_MAP.get(emotion_key, GIF_MAP["neutral"])

    # Si la emoción tiene múltiples opciones (ej. Disgusto), elige una aleatoria
    if isinstance(gif_choice, list):
        return random.choice(gif_choice)
        
    return gif_choice

# ===============================================
# CALLBACKS DE SUSCRIPCIÓN Y DECODIFICACIÓN
# ===============================================

def detections_callback(data):
    """
    Callback para el topic de detecciones de YOLO.
    Decodifica el array y actualiza el tiempo de actividad SOLO si hay detecciones.
    """
    global current_detections, last_vision_data_time
    
    array = data.data
    
    # CORRECCIÓN: Solo actualizamos el tiempo si el array NO está vacío (hubo una detección)
    if len(array) > 0:
        last_vision_data_time = time.time()
    
    BLOCK_SIZE = 6 
    if len(array) % BLOCK_SIZE != 0 or len(array) == 0:
        # Si el array es inválido o vacío, forzamos "none" para la detección dominante
        current_detections = {"name": "none", "confidence": 0.0}
        if len(array) > 0: # Si es inválido (no múltiplo de BLOCK_SIZE)
             rospy.logwarn("Detections array tiene un tamaño inválido.")
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
    Decodifica el array, prioriza la emoción no-neutral más frecuente y actualiza el tiempo de actividad SOLO si hay detecciones.
    """
    global current_emotion, last_vision_data_time
    
    array = data.data
    
    # CORRECCIÓN: Solo actualizamos el tiempo si el array NO está vacío (hubo una detección)
    if len(array) > 0:
        last_vision_data_time = time.time()
    
    BLOCK_SIZE = 5
    if len(array) % BLOCK_SIZE != 0 or len(array) == 0:
        # Si el array es inválido o vacío, no hacemos nada con current_emotion 
        # (se mantiene la última emoción dominante o neutral por defecto)
        if len(array) > 0: # Si es inválido (no múltiplo de BLOCK_SIZE)
            rospy.logwarn("Emotions array tiene un tamaño inválido.")
        return
        
    emotion_counts = {}
    
    for i in range(0, len(array), BLOCK_SIZE):
        emo_id = array[i]
        
        if emo_id in EMOTION_CLASSES:
            emo_name = EMOTION_CLASSES[emo_id]
            emotion_counts[emo_name] = emotion_counts.get(emo_name, 0) + 1

    # Determinar la emoción dominante (priorizando no-neutrales)
    dominant_emotion_name = "neutral"
    max_count = 0
    
    for emo_name, count in emotion_counts.items():
        if emo_name != "neutral" and count > max_count:
            max_count = count
            dominant_emotion_name = emo_name
        # Si solo hay detecciones neutrales
        elif max_count == 0 and emo_name == "neutral" and count > 0:
            dominant_emotion_name = "neutral" 
            
    # Solo actualizamos si se detectó algo (ya sabemos que len(array)>0 por la corrección)
    current_emotion["name"] = dominant_emotion_name
    
    # rospy.loginfo(f"Emoción dominante: {current_emotion['name']}")


# -------------------------------------------------------------
# ESTRUCTURA PARA GESTIONAR ESTADOS EXTERNOS (IF SE ACTIVA...)
# -------------------------------------------------------------

def manual_status_callback(data):
    """
    Callback para el topic de control manual externo.
    Permite a otros nodos forzar un GIF específico.
    Data esperada: un número entero (Float32) que representa el GIF manual.
    """
    global manual_gif
    # 0.0: Control por visión. 4.0: Cargando. 16.0: MapeoV1, etc.
    try:
        new_status = int(data.data) 
    except ValueError:
        rospy.logwarn("Received non-integer data on manual control topic.")
        return
    
    # Mapeos específicos de estados
    if new_status == 15: # Indecisión por obstáculo
        manual_gif = 15
    elif new_status == 4: # Cargando
        manual_gif = 4
    elif new_status == 7: # Despertar (Inicio de todo)
        manual_gif = 7
    # Lógica para MapeoV1/V2 y ConduccionV1/V2
    elif new_status == 16:
        manual_gif = 16 # MAPEOV1
    elif new_status == 17:
        manual_gif = 17 # MAPEOV2
    elif new_status == 5:
        manual_gif = 5 # CONDUCCIONV1
    elif new_status == 6:
        manual_gif = 6 # CONDUCCIONV2
    elif new_status == 0:
        # Para volver al control de visión, otro nodo debería enviar 0.0
        manual_gif = 0
    else:
        # Si se envía un número desconocido, simplemente lo asignamos o ignoramos
        manual_gif = new_status
        
    rospy.loginfo(f"Estado manual actualizado a GIF: {manual_gif}")

# -------------------------------------------------------------
# FUNCIÓN DE APAGADO SEGURO
# -------------------------------------------------------------

def safe_shutdown():
    """
    Envía el comando del GIF base (neutral) al ESP32 y cierra el puerto serial.
    Esta función se llama al detectar Ctrl+C o la interrupción de ROS (a través de rospy.on_shutdown).
    """
    global ser
    if ser and ser.is_open:
        try:
            # 1. Enviar el comando para el GIF base (neutral: 1)
            base_gif_id = GIF_MAP["neutral"]
            serial_output = f"{base_gif_id}\n"
            ser.write(serial_output.encode('utf-8'))
            rospy.loginfo(f"Comando de finalización enviado: {base_gif_id}.gif")
        except Exception as e:
            # Si falla el envío, solo logueamos, no detenemos el cierre
            rospy.logwarn(f"Error al enviar comando base antes de cerrar serial: {e}")
            
        # 2. Cerrar el puerto serial
        ser.close()
        
    rospy.loginfo("Nodo GIF Controller finalizado.")


# ===============================================
# NODO PRINCIPAL Y BUCLE DE SERIAL
# ===============================================

def gif_controller_node():
    global ser, last_gif_number, last_gif_change_time
    
    # Inicialización del nodo ROS
    if not rospy.core.is_initialized():
        rospy.init_node('gif_controller', anonymous=True)
        
    # ⚠️ REGISTRO DE APAGADO SEGURO
    # Esto asegura que safe_shutdown se llama justo antes de que ROS termine (por Ctrl+C o cualquier otra señal).
    rospy.on_shutdown(safe_shutdown) 

    # 1. Configuración de la comunicación serial
    try:
        # Aseguramos un tiempo de espera para que el ESP32 se reinicie
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2) 
        rospy.loginfo(f"Puerto serial {SERIAL_PORT} conectado.")
    except serial.SerialException as e:
        rospy.logerr(f"No se pudo abrir el puerto serial {SERIAL_PORT}. Asegúrate de que el ESP32 esté conectado y el puerto sea correcto. Error: {e}")
        # Si no se puede abrir el puerto serial, se debe permitir que la excepción se propague
        raise # Propagar la excepción para que el bloque except del main la capture
        return # Nunca se alcanzará, pero se deja por seguridad

    # 2. Suscripciones a topics
    rospy.Subscriber('/detecciones_emociones', Float32MultiArray, emotions_callback)
    rospy.Subscriber('/detecciones_yolo', Float32MultiArray, detections_callback)
    
    # Suscripción para control de estados externos (IF SE activa...)
    # Usaremos '/robot_status' con el tipo std_msgs/Float32
    rospy.Subscriber('/robot_status', Float32, manual_status_callback) 
    
    rospy.loginfo("Nodo GIF Controller iniciado y suscrito a topics.")
    
    # 3. Bucle de control (publicación serial)
    rate = rospy.Rate(10) # 10 Hz para un control responsivo
    
    # Constante para el retraso (debounce)
    GIF_CHANGE_DELAY = 5.0 # 5 segundos de retraso
    
    while not rospy.is_shutdown():
        
        new_gif_number = select_gif_logic()
        
        current_time = time.time()
        
        # Solo cambiar si: 
        # a) El GIF es diferente al último enviado.
        # b) Ha pasado el tiempo de debounce (5 segundos).
        # c) El número de GIF es válido.
        if new_gif_number != last_gif_number and \
           (current_time - last_gif_change_time) > GIF_CHANGE_DELAY and \
           new_gif_number is not None:
            
            try:
                # El ESP32 espera el número seguido de un salto de línea (\n)
                serial_output = f"{new_gif_number}\n"
                ser.write(serial_output.encode('utf-8'))
                last_gif_number = new_gif_number
                last_gif_change_time = current_time # Actualizar el tiempo del último cambio
                rospy.loginfo(f"Cambiando GIF a: {new_gif_number}.gif")
            except serial.SerialException as e:
                rospy.logerr(f"Error al escribir en serial: {e}. Deteniendo nodo.")
                # Si hay un error serio de serial, señalamos a ROS para que se apague
                rospy.signal_shutdown("Error de comunicación serial.")
                break 
            except Exception as e:
                rospy.logwarn(f"Error inesperado al enviar serial: {e}. GIF intentado: {new_gif_number}")


        rate.sleep()

if __name__ == '__main__':
    try:
        # La función gif_controller_node ahora registra safe_shutdown con rospy.on_shutdown()
        gif_controller_node()
    except serial.SerialException:
        # Maneja casos donde el error de serial ocurre antes del bucle principal
        rospy.loginfo("El nodo terminó debido a un error de conexión serial (problema de apertura del puerto).")
    except Exception as e:
        # Capturamos cualquier otra excepción no manejada por ROS
        rospy.logerr(f"Excepción inesperada en el nodo principal: {e}")

