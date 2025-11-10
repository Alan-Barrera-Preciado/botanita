#!/usr/bin/env python3

import rospy
import os
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import pandas as pd
import numpy as np
from math import cos, sin
import rospkg
import math

rospy.init_node('splines')
pub = rospy.Publisher('/vel_referencia', Float32MultiArray, queue_size=10)

def callback(pose_msg):
    x = pose_msg.pose.position.x
    y = pose_msg.pose.position.y
    
    q = pose_msg.pose.orientation
    (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    rospy.loginfo(f"Posicion: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.2f}")
    
    
S = 15
D = 0.30
kpx, kpy = 4.5, 4.5
kdx, kdy = 1.5, 1.5
kix, kiy = 0.5, 0.5
dt = 0.0

Kp = np.array([[kpx, 0], [0, kpy]])
Kd = np.array([[kdx, 0], [0, kdy]])
Ki = np.array([[kix, 0], [0, kiy]])    

error = np.array([0.0, 0.0])
error_integral = np.array([0.0, 0.0])
error_anterior = np.array([0.0, 0.0])
error_derivativo = np.array([0.0, 0.0])

rate = rospy.Rate(20)

pose_x = 0.0
pose_y = 0.0
pose_yaw = 0.0

tiempos = []
x_actual = []
y_actual = []
x_deseada = []
y_deseada = []

datos = {
    "t": [],
    "x_actual": [],
    "y_actual": [],
    "x_deseada": [],
    "y_deseada": [],
    "Ref_Izq": [],
    "Ref_Der": [],
    "Vel_Lineal": [],
    "Vel_Angular": [],
    "error_x": [],
    "error_y": []
}

def odom_callback(msg):
    global pose_x, pose_y, pose_yaw
    pose_x = msg.pose.position.x - 0.3
    pose_y = msg.pose.position.y
    q = msg.pose.orientation
    (_, _, pose_yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

rospy.Subscriber('/slam_out_pose', PoseStamped, odom_callback)

while not rospy.is_shutdown():

    xdh = 2.0
    ydh = 0.0

    Theta = pose_yaw
    xh = pose_x + D * cos(pose_yaw)
    yh = pose_y + D * sin(pose_yaw)

    A = np.array([
        [cos(Theta), sin(Theta)],
        [-(1/D)*sin(Theta), (1/D)*cos(Theta)]
    ])

    error = np.array([xdh - xh, ydh - yh])
    error_integral += error * 0.05
    error_derivativo = (error - error_anterior) / 0.05
    error_anterior = error
   
    Error_U = Kp @ error + Ki @ error_integral + Kd @ error_derivativo
    u = A @ Error_U
    
    Vel_Lineal = u[0]
    Vel_Angular = u[1]

    Ref_Izq = -max(min((2*Vel_Lineal - Vel_Angular * 42) / 16, 6), -6)
    Ref_Der =  max(min((2*Vel_Lineal + Vel_Angular * 42) / 16, 6), -6)

    if dt > S:
       Ref_Izq = 0
       Ref_Der = 0

    Ref = Float32MultiArray()
    Ref.data = [Ref_Izq, Ref_Der]
    pub.publish(Ref)

    if dt > S:
        rospy.signal_shutdown("Trayectoria completada")
        break

    dt += 0.05
    
    datos["t"].append(dt)
    datos["x_actual"].append(xh)
    datos["y_actual"].append(yh)
    datos["x_deseada"].append(xdh)
    datos["y_deseada"].append(ydh)
    datos["Ref_Izq"].append(Ref_Izq)
    datos["Ref_Der"].append(Ref_Der)
    datos["Vel_Lineal"].append(Vel_Lineal)
    datos["Vel_Angular"].append(Vel_Angular)
    datos["error_x"].append(error[0])
    datos["error_y"].append(error[1])
    
    rate.sleep()

df = pd.DataFrame(datos)

# Crear carpeta de salida dentro del paquete (si no existe)
ROS_PACKAGE_NAME = "botanita" 
rospack = rospkg.RosPack()
pkg_path = rospack.get_path(ROS_PACKAGE_NAME)
resultados_dir = os.path.join(pkg_path, "src", "Resultados")
os.makedirs(resultados_dir, exist_ok=True)

Result_Name = rospy.get_param("~Result_Name", "Resultados")
if not Result_Name.lower().endswith('.csv'):
    Result_Name = f"{Result_Name}.csv"
csv_path = os.path.join(resultados_dir, Result_Name)
df.to_csv(csv_path, index=False)

rospy.loginfo(f"Datos de trayectoria guardados en: {csv_path}")

xd = np.array(df["x_deseada"])
yd = np.array(df["y_deseada"])
xa = np.array(df["x_actual"])
ya = np.array(df["y_actual"])

plt.figure()
plt.plot(xd, yd, 'r--', label="Deseada")
plt.plot(xa, ya, 'b-', label="Real")
plt.legend()
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Trayectoria generada vs deseada")
plt.grid()
plt.show()
