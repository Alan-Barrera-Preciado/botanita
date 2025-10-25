#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import pandas as pd
import numpy as np
from math import cos, sin
import math

rospy.init_node('splines')
pub = rospy.Publisher('/vel_referencia', Float32MultiArray, queue_size=10)

def callback(pose_msg):
    x = pose_msg.pose.position.x
    y = pose_msg.pose.position.y
    
    q = pose_msg.pose.orientation
    (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    rospy.loginfo(f"Posicion: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.2f}")

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

Ruta = pd.read_csv('ruta_simple.csv')

CantidadPuntos = len(Ruta.x)
tf = 0.05
S = 10
T = np.linspace(0, S, CantidadPuntos)
SpX = Obtener_Splines(T, Ruta.x)
SpY = Obtener_Splines(T, Ruta.y)

D = 0.10
kpx, kpy = 0.7, 0.7
kdx, kdy = 0.5, 0.5
kix, kiy = 0.0, 0.0
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

def odom_callback(msg):
    global pose_x, pose_y, pose_yaw
    pose_x = msg.pose.position.x
    pose_y = msg.pose.position.y
    q = msg.pose.orientation
    (_, _, pose_yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

rospy.Subscriber('/slam_out_pose', PoseStamped, odom_callback)

while not rospy.is_shutdown():
    xdh, xdh_p = Evaluar_Splines(SpX, T, dt)
    ydh, ydh_p = Evaluar_Splines(SpY, T, dt)

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
    u = A @ (np.array([xdh_p, ydh_p]) + Error_U)
    
    print(Error_U)
    
    Vel_Lineal = u[0]
    Vel_Angular = u[1]

    Ref_Izq = -max(min((2*Vel_Lineal - Vel_Angular * 42) / 16, 4), -4)
    Ref_Der =  max(min((2*Vel_Lineal + Vel_Angular * 42) / 16, 4), -4)

    if dt > S:
       Ref_Izq = 0
       Ref_Der = 0

    Ref = Float32MultiArray()
    Ref.data = [Ref_Izq, Ref_Der]
    pub.publish(Ref)

    if dt > S:
        rospy.signal_shutdown("Trayectoria completada")
        break

    xdh = float(np.squeeze(xdh))
    ydh = float(np.squeeze(ydh))

    xh = float(np.squeeze(xh))
    yh = float(np.squeeze(yh))


    tiempos.append(float(dt))
    x_actual.append(float(xh))
    y_actual.append(float(yh))
    x_deseada.append(float(xdh))
    y_deseada.append(float(ydh))

    dt += 0.05
    rate.sleep()

df = pd.DataFrame({
    'tiempo': tiempos,
    'x_actual': x_actual,
    'y_actual': y_actual,
    'x_deseada': x_deseada,
    'y_deseada': y_deseada
})



plt.plot(df['x_deseada'], df['y_deseada'], 'r--', label='Deseada')
plt.plot(df['x_actual'], df['y_actual'], 'b-', label='Actual')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.legend()
plt.title('Seguimiento de trayectoria')
plt.grid(True)
plt.show()
