#!/usr/bin/env python3

import rospy
import os
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import pandas as pd
import numpy as np
from math import cos, sin
import rospkg
import math

rospy.init_node('splines')
pub = rospy.Publisher('/vel_referencia', Float32MultiArray, queue_size=10)

def Obtener_Splines(T, Puntos):
    n = len(T)
    h = np.diff(T)

    diag_sup_inf = h[1:n-2]
    diag_prin = 2 * (h[0:n-2] + h[1:n-1])

    M = np.diag(diag_prin) + np.diag(diag_sup_inf, k=1) + np.diag(diag_sup_inf, k=-1)
    f = np.diff(Puntos) / np.diff(T)
    B = 6 * np.diff(f)

    g_inner = np.linalg.solve(M, B)
    g = np.concatenate(([0], g_inner, [0]))

    a = np.diff(g) / (6 * h)
    b = g[:-1] / 2
    c = f - h * (2 * g[:-1] + g[1:]) / 6
    d = Puntos[:-1]

    return np.column_stack((a, b, c, d))


def Evaluar_Splines(S, T, time):
    n = len(T)
    if time > T[-1]:
        k = n - 2
    else:
        k = np.where((T[:-1] <= time) & (time <= T[1:]))[0][0]

    dt = time - T[k]
    qk = S[k, :] @ np.array([dt**3, dt**2, dt, 1])
    qpk = S[k, :] @ np.array([3*dt**2, 2*dt, 1, 0])

    return qk, qpk

ROS_PACKAGE_NAME = "botanita" 
rospack = rospkg.RosPack()
pkg_path = rospack.get_path(ROS_PACKAGE_NAME)
Ruta_dir = os.path.join(pkg_path, "src", "Trayectorias") 
Route_Name = rospy.get_param("~Route_Name", "Ruta")
if not Route_Name.lower().endswith('.csv'):
        Route_Name = f"{Route_Name}.csv"
Ruta_csv = os.path.join(Ruta_dir, Route_Name)
Ruta = pd.read_csv(Ruta_csv)

CantidadPuntos = len(Ruta.x)
tf = 0.05
S = 10
T = np.linspace(0, S, CantidadPuntos)
SpX = Obtener_Splines(T, Ruta.x)
SpY = Obtener_Splines(T, Ruta.y)

D = 0.15
kpx, kpy = 0.05, 0.05
kdx, kdy = 0.015, 0.015
kix, kiy = 0.001, 0.001
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
theta_est = 0.0

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

def theta_callback(msg):
    global theta_est
    theta_est = msg.data
    
def odom_callback(pose_msg):
    global pose_x, pose_y
    x = pose_msg.pose.pose.position.x
    y = pose_msg.pose.pose.position.y
    pose_x = x - D
    pose_y = y

rospy.Subscriber('/odom_estimada', Odometry, odom_callback)
rospy.Subscriber('/theta_estimada', Float32, theta_callback)

while not rospy.is_shutdown():
    xdh, xdh_p = Evaluar_Splines(SpX, T, dt)
    ydh, ydh_p = Evaluar_Splines(SpY, T, dt)

    Theta = 0 # theta_est
    xh = pose_x + D * cos(Theta)
    yh = pose_y + D * sin(Theta)

    A = np.array([
        [cos(Theta), sin(Theta)],
        [-(1/D)*sin(Theta), (1/D)*cos(Theta)]
    ])

    error = np.array([xdh - xh, ydh - yh])
    error_integral += error * 0.01
    error_derivativo = (error - error_anterior) / 0.01
    error_anterior = error
   
    Error_U = Kp @ error + Ki @ error_integral + Kd @ error_derivativo
    u = A @ (np.array([xdh_p, ydh_p]) + Error_U)
    
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
resultados_dir = os.path.join(pkg_path, "src", "Resultados")
os.makedirs(resultados_dir, exist_ok=True)

Result_Name = rospy.get_param("~Result_Name", "Resultados")
if not Route_Name.lower().endswith('.csv'):
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
