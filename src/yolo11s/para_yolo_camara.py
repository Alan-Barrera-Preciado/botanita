#!/usr/bin/env python3
import cv2
import numpy as np
import onnxruntime as ort
import os
import sys

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

rospy.init_node('detecciones_camara')

yolo_detections_pub = rospy.Publisher('/detecciones_yolo', Float32MultiArray, queue_size=10)

emotion_detections_pub = rospy.Publisher('/detecciones_emociones', Float32MultiArray, queue_size=10)

YOLO_MODEL_PATH = 'best.onnx'
YOLO_INPUT_SIZE = (320,320) # width, heigth

# get yolo clasees
with open('yolo-classes.txt') as file:
  YOLO_CLASSES = file.read().split('\n')

# load ONNX model
ONNX_MODEL = ort.InferenceSession(YOLO_MODEL_PATH)

# load test image
CAM_SHAPE = (640, 480)
#CAM_SHAPE = (1280, 720)
# CAM_SHAPE = (1920, 1080)


def imgInputPreProcees(rawImage, inputSize):
  imgInput = cv2.cvtColor(rawImage, cv2.COLOR_BGR2RGB) # transform to RGB 
  imgInput = cv2.resize(imgInput, inputSize)
  # transform shame to standart "channels-first"
  imgInput = imgInput.transpose(2,0,1) # (inputSize[0], inputSize[1], 3) -> (3, inputSize[0], inputSize[1])
  imgInput = imgInput.reshape(1, 3, inputSize[0], inputSize[1]) # "1" is for batch-size
  # normalize it and change to expected type format
  imgInput = imgInput/255.0
  imgInput = imgInput.astype(np.float32)

  return imgInput

# selecting the class with the highest confidence score for each detection
# discard detections where all confidence scores are below than a chosen threshold (0.5)
def filter_Detections(results, thresh = 0.6):
    # if model is trained on 1 class only
    #detections = []
    if len(results[0]) == 5:
        # filter out the detections with confidence > thresh
        considerable_detections = [detection for detection in results if detection[4] > thresh]
        considerable_detections = np.array(considerable_detections)
        return considerable_detections

    # if model is trained on multiple classes
    else:
        A = []
        for detection in results:

            class_id = detection[4:].argmax()
            confidence_score = detection[4:].max()
            

            new_detection = np.append(detection[:4],[class_id,confidence_score])

            A.append(new_detection)

        A = np.array(A)

        # filter out the detections with confidence > thresh
        considerable_detections = [detection for detection in A if detection[-1] > thresh]
        considerable_detections = np.array(considerable_detections)

        return considerable_detections #, detections
  
# Non-Maximum Suppression algorithm (NMS) 
def NMS(boxes, conf_scores, iou_thresh = 0.55):

    #  boxes [[x1,y1, x2,y2], [x1,y1, x2,y2], ...]

    x1 = boxes[:,0]
    y1 = boxes[:,1]
    x2 = boxes[:,2]
    y2 = boxes[:,3]

    areas = (x2-x1)*(y2-y1)

    order = conf_scores.argsort()

    keep = []
    keep_confidences = []

    while len(order) > 0:
        idx = order[-1] # box with highest confidence
        A = boxes[idx]
        conf = conf_scores[idx]

        order = order[:-1]

        xx1 = np.take(x1, indices= order)
        yy1 = np.take(y1, indices= order)
        xx2 = np.take(x2, indices= order)
        yy2 = np.take(y2, indices= order)

        keep.append(A)
        keep_confidences.append(conf)

        # iou = inter/union

        xx1 = np.maximum(x1[idx], xx1)
        yy1 = np.maximum(y1[idx], yy1)
        xx2 = np.minimum(x2[idx], xx2)
        yy2 = np.minimum(y2[idx], yy2)

        w = np.maximum(xx2-xx1, 0)
        h = np.maximum(yy2-yy1, 0)

        intersection = w*h

        # union = areaA + other_areas - intesection
        other_areas = np.take(areas, indices= order)
        union = areas[idx] + other_areas - intersection

        iou = intersection/union

        boleans = iou < iou_thresh

        order = order[boleans]

        # order = [2,0,1]  boleans = [True, False, True]
        # order = [2,1]

    return keep, keep_confidences


# function to rescale bounding boxes 
def rescale_back(results,img_w,img_h, inputSize):
    cx, cy, w, h, class_id, confidence = results[:,0], results[:,1], results[:,2], results[:,3], results[:,4], results[:,-1]
    cx = cx/float(inputSize) * img_w
    cy = cy/float(inputSize) * img_h
    w = w/float(inputSize) * img_w
    h = h/float(inputSize) * img_h
    x1 = cx - w/2
    y1 = cy - h/2
    x2 = cx + w/2
    y2 = cy + h/2

    boxes = np.column_stack((x1, y1, x2, y2, class_id))
    keep, keep_confidences = NMS(boxes,confidence)
    # print(np.array(keep).shape)
    return keep, keep_confidences
# print(faces)

#############

CUSTOM_MODEL_PATH = 'emociones_gray_s_v3.onnx'
CUSTOM_MODEL = ort.InferenceSession(CUSTOM_MODEL_PATH)

# Cargar las clases de emociones
with open('classes_custom.txt') as file:
    EMOTION_CLASSES = file.read().splitlines() # .splitlines() es más robusto

# Definir el tamaño de entrada para el modelo de emociones
EMOTION_INPUT_SIZE = (96, 96) # ancho, alto

def preprocess_emotion_input_int8(face_crop, input_size):
    """
    Pre-procesa un recorte de cara para un modelo INT8 que espera una IMAGEN A COLOR.
    Convierte a RGB, redimensiona, ajusta la forma a channels-last y desplaza el rango.
    """
    if face_crop.size == 0:
        return None

    # 1. NO convertir a escala de grises. Es buena práctica convertir de BGR a RGB,
    # ya que la mayoría de los modelos se entrenan con RGB.
    rgb_face = cv2.cvtColor(face_crop, cv2.COLOR_BGR2RGB)
    
    # 2. Redimensionar al tamaño de entrada del modelo (96x96)
    resized_face = cv2.resize(rgb_face, input_size)
    
    # 3. Añadir la dimensión de lote (batch). La forma ahora es (96, 96, 3).
    # El modelo espera (1, 96, 96, 3)
    reshaped_face = resized_face.reshape(1, input_size[1], input_size[0], 3)
    
    # 4. Desplazar el rango de [0, 255] a [-128, 127] y convertir a int8
    input_tensor = (reshaped_face.astype(np.float32) - 128.0).astype(np.int8)

    return input_tensor

def preprocess_emotion_grayscale_int8(face_crop, input_size):
    """
    Pre-procesa un recorte de cara para un modelo INT8 que espera ESCALA DE GRISES.
    Convierte a gris, redimensiona y ajusta la forma a (1, H, W, 1).
    """
    if face_crop.size == 0:
        return None

    # 1. Convertir a escala de grises (¡Este paso es clave ahora!)
    gray_face = cv2.cvtColor(face_crop, cv2.COLOR_BGR2GRAY)
    
    # 2. Redimensionar al tamaño de entrada del modelo (48x48)
    resized_face = cv2.resize(gray_face, input_size)
    
    # 3. Añadir las dimensiones de lote y canal. La forma ahora es (48, 48).
    # El modelo espera (1, 48, 48, 1)
    reshaped_face = resized_face.reshape(1, input_size[1], input_size[0], 1)
    
    # 4. Desplazar el rango a [-128, 127] y convertir a int8
    input_tensor = (reshaped_face.astype(np.float32) - 128.0).astype(np.int8)

    return input_tensor

#############

#cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(0, cv2.CAP_V4L2) 
if not cap.isOpened():
    print("Error: No se puede abrir la cámara.")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_SHAPE[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_SHAPE[1])
print(f"Cámara iniciada con resolución: {int(cap.get(3))}x{int(cap.get(4))}")

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        print("No se pudo recibir el fotograma. Saliendo...")
        break    

    yolo_numeric_data = []
    emotions_numeric_data = []

    yoloInput = imgInputPreProcees(frame, YOLO_INPUT_SIZE)
    yoloOutput = ONNX_MODEL.run(None, {"images": yoloInput})
    
    results = yoloOutput[0].transpose()
    results = filter_Detections(results)

    if results.size > 0:
        rescaled_results, confidences = rescale_back(results, CAM_SHAPE[0], CAM_SHAPE[1], YOLO_INPUT_SIZE[0])

        for res, conf in zip(rescaled_results, confidences):
            x1, y1, x2, y2, cls_id = res
            cls_id = int(cls_id)
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # Etiqueta por defecto (solo clase)
            final_label = f"{YOLO_CLASSES[cls_id]} {conf:.2f}"
            
            # Formato: [cls_id, conf, x1, y1, x2, y2] (6 campos)
            yolo_numeric_data.extend([float(cls_id), conf, float(x1), float(y1), float(x2), float(y2)])

            # --- INICIO DE LA MODIFICACIÓN ---
            # Si la detección es una 'cara', ejecuta el segundo modelo
            if YOLO_CLASSES[cls_id] == 'face':
                # Recorta la cara de la imagen original
                face_crop = frame[y1:y2, x1:x2]
                
                # Pre-procesa el recorte para el modelo de emociones
                emotion_input = preprocess_emotion_grayscale_int8(face_crop, EMOTION_INPUT_SIZE)
                # emotion_input = preprocess_emotion_input_int8(face_crop, EMOTION_INPUT_SIZE)
                
                if emotion_input is not None:
                    # Ejecuta la inferencia del modelo de emociones
                    # OJO: Cambia 'input_name' por el nombre real de la entrada de tu modelo
                    emotion_output = CUSTOM_MODEL.run(None, {'serving_default_args_0:0': emotion_input})
                    
                    # Obtiene el ID de la emoción predicha (la de mayor probabilidad)
                    emotion_id = np.argmax(emotion_output[0])

                    emotions_numeric_data.extend([float(emotion_id), float(x1), float(y1), float(x2), float(y2)])

                    emotion_label = EMOTION_CLASSES[emotion_id]
                    emotions_labels.append(emotion_label)
                    
                    # Actualiza la etiqueta para incluir la emoción
                    final_label = f"face - {emotion_label}"
            # --- FIN DE LA MODIFICACIÓN ---

            # Dibuja la caja y la etiqueta final
            #cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            #cv2.putText(frame, final_label, (x1, y1 - 10),
            #            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # print(final_label)
    # print(emotions_labels)
    
    # 1. Mensaje YOLO
    yolo_msg = Float32MultiArray()
    yolo_msg.layout.dim.append(MultiArrayDimension())
    yolo_msg.layout.dim[0].label = "detections"
    yolo_msg.layout.dim[0].size = len(yolo_numeric_data) // 6  # Número de detecciones
    yolo_msg.layout.dim[0].stride = 6                          # Campos por detección
    yolo_msg.data = yolo_numeric_data
    yolo_detections_pub.publish(yolo_msg)

    # 2. Mensaje de Emociones
    emotion_msg = Float32MultiArray()
    emotion_msg.layout.dim.append(MultiArrayDimension())
    emotion_msg.layout.dim[0].label = "emotions"
    emotion_msg.layout.dim[0].size = len(emotions_numeric_data) // 5 # Número de emociones
    emotion_msg.layout.dim[0].stride = 5                           # Campos por emoción
    emotion_msg.data = emotions_numeric_data
    emotion_detections_pub.publish(emotion_msg)

    rate.sleep()

    yolo_labels = []
    emotions_labels = []



    #cv2.imshow('Deteccion de Emociones en Tiempo Real', frame)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
