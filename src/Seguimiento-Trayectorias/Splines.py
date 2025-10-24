#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import pandas as pd
import numpy as np
from math import cos, sin

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

Ruta = pd.read_csv('ruta_metros.csv')

CantidadPuntos = len(Ruta.x)
tf = 0.05
S = tf * CantidadPuntos
T = np.linspace(0, S, CantidadPuntos)
SpX = Obtener_Splines(T, Ruta.x)
SpY = Obtener_Splines(T, Ruta.y)

D = 0.10
kx, ky = 0.05, 0.05
dt = 0.0
rate = rospy.Rate(20)

odom_data = Odometry()

def odom_callback(msg):
    global odom_data
    odom_data = msg

rospy.Subscriber('/odom', Odometry, odom_callback)

while not rospy.is_shutdown():
    xdh, xdh_p = Evaluar_Splines(SpX, T, dt)
    ydh, ydh_p = Evaluar_Splines(SpY, T, dt)

    Theta = odom_data.twist.twist.angular.z
    xh = odom_data.pose.pose.position.x + D * cos(Theta)
    yh = odom_data.pose.pose.position.y + D * sin(Theta)

    A = np.array([
        [cos(Theta), sin(Theta)],
        [-(1/D)*sin(Theta), (1/D)*cos(Theta)]
    ])

    error = np.array([xdh - xh, ydh - yh])
    K = np.array([[kx, 0], [0, ky]])
    
    u = A @ (np.array([xdh_p, ydh_p]) * 0 + K @ error)

    Vel_Lineal = u[0]
    Vel_Angular = u[1]

    Ref_Izq = -max(min((2*Vel_Lineal - Vel_Angular * 42) / 16, 12), -12)
    Ref_Der =  max(min((2*Vel_Lineal + Vel_Angular * 42) / 16, 12), -12)

    Ref = Float32MultiArray()
    Ref.data = [Ref_Izq, Ref_Der]
    pub.publish(Ref)

    dt += 0.05
    rate.sleep()

