#!/usr/bin/env python3

# -------- Librerias ----------

import cv2
import numpy as np
import onnxruntime as ort
import os
import sys
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

rospy.init_node('detecciones_camara')

# -------- Publicadores de ROS ----------

yolo_detections_pub = rospy.Publisher('/detecciones_yolo', Float32MultiArray, queue_size=10)
emotion_detections_pub = rospy.Publisher('/detecciones_emociones', Float32MultiArray, queue_size=10)

# -------- Configuracion de modelos ----------

YOLO_MODEL_PATH = 'best_new.onnx'
YOLO_INPUT_SIZE = (320, 320)

# -------- Configurar las clases de YOLO ----------

try:
    with open('yolo-classes.txt') as file:
        YOLO_CLASSES = file.read().split('\n')
except FileNotFoundError:
    rospy.logerr("Error: Archivo 'yolo-classes.txt' no encontrado.")
    sys.exit(1)

# -------- Cargar modelo de YOLO ----------

try:
    ONNX_MODEL = ort.InferenceSession(YOLO_MODEL_PATH)
except ort.OnnxRuntimeError as e:
    rospy.logerr(f"Error al cargar el modelo YOLO ONNX: {e}")
    sys.exit(1)

# -------- Cargar emociones ----------

CUSTOM_MODEL_PATH = 'emociones_gray_s_v5.onnx'

try:
    CUSTOM_MODEL = ort.InferenceSession(CUSTOM_MODEL_PATH)
except ort.OnnxRuntimeError as e:
    rospy.logerr(f"Error al cargar el modelo de EMOCIONES ONNX: {e}")
    sys.exit(1)

EMOTION_INPUT_NAME = CUSTOM_MODEL.get_inputs()[0].name
rospy.loginfo(f"Nombre de entrada del modelo de emociones detectado: {EMOTION_INPUT_NAME}")

try:
    with open('classes_custom.txt') as file:
        EMOTION_CLASSES = file.read().splitlines()
except FileNotFoundError:
    rospy.logerr("Error: Archivo 'classes_custom.txt' no encontrado.")
    sys.exit(1)

EMOTION_INPUT_SIZE = (75, 75) 
CAM_SHAPE = (640, 480)

# -------- Funciones de pre-procesamiento y post-procesamiento ----------

def imgInputPreProcees(rawImage, inputSize):
    imgInput = cv2.cvtColor(rawImage, cv2.COLOR_BGR2RGB)
    imgInput = cv2.resize(imgInput, inputSize)

    imgInput = imgInput.transpose(2,0,1)
    imgInput = imgInput.reshape(1, 3, inputSize[0], inputSize[1])

    imgInput = imgInput/255.0
    imgInput = imgInput.astype(np.float32)
    return imgInput

# -------- Filtrar las detecciones de la camara ----------

def filter_Detections(results, thresh = 0.55):
    if len(results[0]) == 5:
        considerable_detections = [detection for detection in results if detection[4] > thresh]
        considerable_detections = np.array(considerable_detections)
        return considerable_detections
    else:
        A = []
        for detection in results:
            class_id = detection[4:].argmax()
            confidence_score = detection[4:].max()
            new_detection = np.append(detection[:4], [class_id, confidence_score])
            A.append(new_detection)

        A = np.array(A)

        considerable_detections = [detection for detection in A if detection[-1] > thresh]
        considerable_detections = np.array(considerable_detections)
        return considerable_detections
  
def NMS(boxes, conf_scores, iou_thresh = 0.55):
    x1 = boxes[:,0]
    y1 = boxes[:,1]
    x2 = boxes[:,2]
    y2 = boxes[:,3]
    areas = (x2-x1)*(y2-y1)
    order = conf_scores.argsort()
    keep = []
    keep_confidences = []

    while len(order) > 0:
        idx = order[-1]
        A = boxes[idx]
        conf = conf_scores[idx]
        order = order[:-1]

        xx1 = np.take(x1, indices= order)
        yy1 = np.take(y1, indices= order)
        xx2 = np.take(x2, indices= order)
        yy2 = np.take(y2, indices= order)

        keep.append(A)
        keep_confidences.append(conf)

        xx1 = np.maximum(x1[idx], xx1)
        yy1 = np.maximum(y1[idx], yy1)
        xx2 = np.minimum(x2[idx], xx2)
        yy2 = np.minimum(y2[idx], yy2)

        w = np.maximum(xx2-xx1, 0)
        h = np.maximum(yy2-yy1, 0)

        intersection = w*h
        other_areas = np.take(areas, indices= order)
        union = areas[idx] + other_areas - intersection
        iou = intersection/union
        boleans = iou < iou_thresh
        order = order[boleans]

    return keep, keep_confidences

# -------- Resaltar detecciones de la camara ----------

def rescale_back(results, img_w, img_h, inputSize):
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
    keep, keep_confidences = NMS(boxes, confidence)
    return keep, keep_confidences

def preprocess_emotion_grayscale_int8(face_crop, input_size):
    if face_crop.size == 0:
        return None

    gray_face = cv2.cvtColor(face_crop, cv2.COLOR_BGR2GRAY)
    resized_face = cv2.resize(gray_face, input_size)
    reshaped_face = resized_face.reshape(1, input_size[1], input_size[0], 1)
    input_tensor = (reshaped_face.astype(np.float32) - 128.0).astype(np.int8)

    return input_tensor

# -------- Inicializacion de la camara ----------

cap = cv2.VideoCapture(0, cv2.CAP_V4L2) 
if not cap.isOpened():
    rospy.logerr("Error: No se puede abrir la cámara.")
    sys.exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_SHAPE[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_SHAPE[1])
rospy.loginfo(f"Cámara iniciada con resolución: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")

rate = rospy.Rate(5)

# -------- Bucle principal ----------

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        rospy.logwarn("No se pudo recibir el fotograma. Intentando de nuevo...")
        rate.sleep()
        continue    

    yolo_numeric_data = []
    emotions_numeric_data = []
    emotions_labels = [] 
    yolo_labels = [] 

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
            
            yolo_numeric_data.extend([float(cls_id), conf, float(x1), float(y1), float(x2), float(y2)])
            yolo_labels.append(YOLO_CLASSES[cls_id])

            if YOLO_CLASSES[cls_id] == 'face':
                face_crop = frame[y1:y2, x1:x2]
                
                emotion_input = preprocess_emotion_grayscale_int8(face_crop, EMOTION_INPUT_SIZE)
                
                if emotion_input is not None:
                    try:
                        emotion_output = CUSTOM_MODEL.run(None, {EMOTION_INPUT_NAME: emotion_input})
                        
                        emotion_id = np.argmax(emotion_output[0])
                        emotion_label = EMOTION_CLASSES[emotion_id]

                        emotions_numeric_data.extend([float(emotion_id), float(x1), float(y1), float(x2), float(y2)])
                        emotions_labels.append(emotion_label)

                    except ort.OnnxRuntimeError as e:
                        rospy.logwarn(f"Error durante la inferencia de emociones: {e}")
                        
# -------- Mostrar la camara y sus detecciones ----------
            
            # final_label = f"YOLO: {YOLO_CLASSES[cls_id]} | Emo: {'/'.join(emotions_labels)}"
            # cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # cv2.putText(frame, final_label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# -------- Publicar detecciones por ROS ----------

    yolo_msg = Float32MultiArray()
    yolo_msg.layout.dim.append(MultiArrayDimension())
    yolo_msg.layout.dim[0].label = "detections"
    yolo_msg.layout.dim[0].size = len(yolo_numeric_data) // 6  
    yolo_msg.layout.dim[0].stride = 6                          
    yolo_msg.data = yolo_numeric_data
    yolo_detections_pub.publish(yolo_msg)

    emotion_msg = Float32MultiArray()
    emotion_msg.layout.dim.append(MultiArrayDimension())
    emotion_msg.layout.dim[0].label = "emotions"
    emotion_msg.layout.dim[0].size = len(emotions_numeric_data) // 5 
    emotion_msg.layout.dim[0].stride = 5                               
    emotion_msg.data = emotions_numeric_data
    emotion_detections_pub.publish(emotion_msg)

    # cv2.imshow('Deteccion', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    rate.sleep()

cap.release()
cv2.destroyAllWindows()