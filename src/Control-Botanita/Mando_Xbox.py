#!/usr/bin/env python3

# -------- Librerias ----------

import rospy
import rospkg
import subprocess
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import String
import time
import os

# -------- Rutas para guardar informacion ----------

rospack = rospkg.RosPack()
pkg_path = rospack.get_path("botanita")
Map_dir = os.path.join(pkg_path, "src/Mapas")

# -------- Nodo de ROS ----------

rospy.init_node('referencias_motores')
ros_env = os.environ.copy()
ros_env["ROS_MASTER_URI"] = rospy.get_param("/ros_master_uri", "http://localhost:11311")

# -------- Publicadores de mensajes de ROS ----------

pub = rospy.Publisher('/vel_referencia', Float32MultiArray, queue_size=10)
status = rospy.Publisher('/robot_status', Float32, queue_size=10)
map = rospy.Publisher('/syscommand', String, queue_size=10)

# -------- Configuracion inicial ----------

rate = rospy.Rate(20)
Cara = 5.0
Ref_Izq = 0
Ref_Der = 0
Vel_Lineal = 0
Vel_Angular = 0
Expresiones = ["Neutral", "Amor", "Antojo", "Cargando", "Control", "Despertar", "Disgusto", "Dormida", "Enojo", "Feliz", "Groseria", "Indecision", "Mapeo", "Miedo", "Pose Creeper", "Pose UwU", "Pose Roblox", "Pose Mewing", "Seriedad", "Triste"]

# -------- Al presionar un boton del mando de Xbox ----------

def joy_callback(data):
    global Ref_Izq, Ref_Der, Vel_Lineal, Vel_Angular, Cara

    if data.axes[7] > 0.8:                 # Cruz Arriba - Aumentar velocidad lineal
        Vel_Lineal = Vel_Lineal + 5
    if data.axes[7] < -0.8:                # Cruz Abajo - Disminuir velocidad lineal
        Vel_Lineal = Vel_Lineal - 5
    if data.axes[6] > 0.8:                 # Cruz Derecha - Aumentar velocidad angular
        Vel_Angular = Vel_Angular + 0.1
    if data.axes[6] < -0.8:                # Cruz Izquierda - Disminuir velocidad angular
        Vel_Angular = Vel_Angular - 0.1
    if data.buttons[3] > 0.8:              # Boton Y - Detener
        Vel_Angular = 0
        Vel_Lineal = 0
    if data.buttons[2] > 0.8:              # Boton X - Guardar mapa generado hasta el momento por HectorSLAM
        Map_Name = rospy.get_param("~Map_Name", "Mapa")
        map.publish("savegeotiff")
        timestamp = time.strftime("%H%M%S")
        Map_Name_Time = f"{Map_Name}_{timestamp}"
        subprocess.Popen(f"rosrun map_server map_saver -f {Map_dir}/{Map_Name_Time}", shell=True, env=ros_env)
    if data.buttons[5] > 0.8:              # Boton RB - Avanzar en lista de emociones
        Cara = Cara + 1.0
        if Cara > 20:
            Cara = 1
        print("Expresion seleccionada: ", Expresiones[int(Cara)-1])
    if data.buttons[4] > 0.8:              # Boton LB - Retroceder en lista de emociones
        Cara = Cara - 1.0
        if Cara < 1:
            Cara = 20
        print("Expresion seleccionada: ", Expresiones[int(Cara)-1])
    if data.buttons[0] > 0.8:              # Boton A - Mandar cara seleccionada
        Estatus = Float32()
        Estatus.data = Cara
        status.publish(Estatus.data)

# -------- Velocidades lineal y angular a velocidades angulares en cada rueda ----------

    Ref_Izq = -max(min((2*Vel_Lineal - Vel_Angular * 42)/(16), 12), -12)
    Ref_Der = max(min((2*Vel_Lineal + Vel_Angular * 42)/(16), 12), -12)

    Ref = Float32MultiArray()
    Ref.data = [Ref_Izq, Ref_Der]
    pub.publish(Ref)

# -------- Suscribirse a topic de mando de Xbox ----------

rospy.Subscriber("/joy", Joy, joy_callback)
rospy.spin()
