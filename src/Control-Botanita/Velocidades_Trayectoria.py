#!/usr/bin/env python3
"""
Seguidor de trayectoria para robot diferencial usando Splines Cúbicos reales
con SciPy + control en el punto adelantado (tipo Kanayama/Samson).

ROS: Noetic (Python 3)
Tópicos:
  - Subscribir: /Odom            (nav_msgs/Odometry)
  - Publicar:  /vel_llanta       (std_msgs/Float32MultiArray)  -> [vel_izq, vel_der]

Requisitos:
  - sudo apt-get install python3-scipy

Estructura:
  - Clase SeguidorTrayectoriaSpline: inicializa ROS, construye splines una sola vez,
    guarda el último estado de odometría y ejecuta un lazo de control a dt fijo con rospy.Timer.

Notas sobre los Splines cúbicos (resumen):
  - Son polinomios por tramos de grado 3 que interpolan los puntos (t_k, q_k),
    asegurando continuidad en valor, 1ª y 2ª derivada en los nudos.
  - Con SciPy usamos scipy.interpolate.CubicSpline para construir X(t) y Y(t).
  - Podemos elegir las condiciones de frontera (bc_type):
        * 'natural'   -> 2ª derivada = 0 en los extremos (suaviza los bordes)
        * 'clamped'   -> fijar las pendientes inicial y final (1ª derivada dada)
        * 'not-a-knot'-> default de SciPy (suaviza eliminando nudos innecesarios)
  - La derivada del spline se obtiene con .derivative(); la evaluamos para obtener x_dot(t), y_dot(t),
    que sirven como "feedforward" (velocidad deseada de la trayectoria) en el control.

Control implementado (punto adelantado D):
  - Definimos el punto de control del robot en (x_h, y_h) = (x + D cos(theta), y + D sin(theta)).
  - Sea (x_h_ref, y_h_ref) el punto de referencia en la trayectoria parametrizada por tiempo t.
  - Usamos la matriz Jacobiana inversa del mapeo [v, w] -> [x_h_dot, y_h_dot] para calcular
    [v, w] = J^{-1} * ( [x_h_ref_dot, y_h_ref_dot] + K * e ), con e = [x_h_ref - x_h, y_h_ref - y_h].
  - Este esquema suma un término proporcional sobre el error al término feedforward de la trayectoria.

Salida a ruedas:
  - Se convierte (v, w) a velocidades angulares de ruedas (rad/s):
      wL = (v - w*L/2)/R,   wR = (v + w*L/2)/R
    donde L es el ancho entre ruedas (track) y R el radio de la rueda.
  - Se limita a un máximo configurable y se publica.

Ajustes recomendados:
  - D ~ 0.1–0.25 m; Kx, Ky ~ 0.5–1.5 para comenzar.
  - Si la velocidad base es muy alta o hay oscilación, incrementa D o reduce las ganancias.
  - Para recorrer la ruta en bucle, activa loop_traj=True (por defecto activado).
"""

#!/usr/bin/env python3
import math
import time
import numpy as np

import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

# SciPy para splines cúbicos reales
try:
    from scipy.interpolate import CubicSpline
except Exception as e:
    raise RuntimeError(
        "No se pudo importar SciPy. Instala con: sudo apt-get install python3-scipy\n"
        f"Detalle: {e}"
    )

# TF para convertir cuaternión -> yaw
import tf.transformations as tft


class SeguidorTrayectoriaSpline:
    def __init__(self):
        # --- Nodo ROS ---
        rospy.init_node('seguidor_trayectoria_spline', anonymous=True)

        # --- Parámetros (puedes sobreescribir vía rosparam) ---
        self.dt = rospy.get_param('~dt', 0.05)                 # periodo de control [s]
        self.D = rospy.get_param('~D', 0.12)                   # distancia del punto adelantado [m]
        self.L = rospy.get_param('~L', 0.42)                   # distancia entre ruedas [m]
        self.R = rospy.get_param('~R', 0.08)                   # radio de rueda [m]
        self.kx = rospy.get_param('~kx', 0.8)                  # ganancia eje x_h
        self.ky = rospy.get_param('~ky', 0.8)                  # ganancia eje y_h
        self.wheel_limit = rospy.get_param('~wheel_limit', 12.0)  # límite abs de vel rueda [rad/s]
        self.loop_traj = rospy.get_param('~loop_traj', True)   # True: repetir trayectoria al finalizar
        self.bc_type = rospy.get_param('~bc_type', 'natural')  # 'natural' | 'clamped' | 'not-a-knot'

        # Si bc_type == 'clamped', puedes fijar las pendientes inicial/final (dx/dt, dy/dt)
        # en los extremos como tu deseo de velocidad: por defecto 0.
        self.dx0 = rospy.get_param('~dx0', 0.0)
        self.dxN = rospy.get_param('~dxN', 0.0)
        self.dy0 = rospy.get_param('~dy0', 0.0)
        self.dyN = rospy.get_param('~dyN', 0.0)

        # --- Publicador ---
        self.pub = rospy.Publisher('/vel_llanta', Float32MultiArray, queue_size=10)

        # --- Estado de odometría ---
        self.has_odom = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # --- Definición de la trayectoria por puntos (X, Y) ---
        # Puedes reemplazar por tu lista de waypoints.
        self.X = np.array([0.0, 1.0, 1.0, 0.0, -1.0, -1.0, 0.0], dtype=float)
        self.Y = np.array([0.0, 1.0, -1.0, 0.0,  1.0, -1.0, 0.0], dtype=float)

        # Tiempo total deseado para recorrer la trayectoria (parámetro de tiempo)
        self.S = rospy.get_param('~T_total', 120.0)  # [s]
        self.t_knots = np.linspace(0.0, self.S, len(self.X))  # t_k estrictamente creciente

        # --- Construcción de los Splines cúbicos ---
        #   X(t), Y(t) y sus derivadas dX/dt, dY/dt
        self._build_splines()

        # Tiempo de inicio para parametrización temporal
        self.t0 = time.time()

        # Suscripción a odometría
        rospy.Subscriber('/Odom', Odometry, self._odom_cb)

        # Lazo de control periódico
        rospy.Timer(rospy.Duration(self.dt), self._control_step)

        rospy.loginfo("[spline] Nodo iniciado. bc_type=%s, loop_traj=%s", self.bc_type, self.loop_traj)

    # -------------------------
    #  Construcción de Splines
    # -------------------------
    def _build_splines(self):
        t = self.t_knots
        X = self.X
        Y = self.Y

        # Validaciones básicas
        if len(t) != len(X) or len(t) != len(Y):
            raise ValueError("t_knots, X e Y deben tener la misma longitud")
        if np.any(np.diff(t) <= 0):
            raise ValueError("t_knots debe ser estrictamente creciente")

        # Selección de condición de frontera
        if self.bc_type == 'clamped':
            # 'clamped' requiere par de pendientes (inicio, fin)
            bc_x = ((1, self.dx0), (1, self.dxN))
            bc_y = ((1, self.dy0), (1, self.dyN))
        else:
            # 'natural' o 'not-a-knot' sólo requieren la cadena
            bc_x = self.bc_type
            bc_y = self.bc_type

        # Splines X(t), Y(t)
        self.sx = CubicSpline(t, X, bc_type=bc_x)
        self.sy = CubicSpline(t, Y, bc_type=bc_y)

        # Derivadas dX/dt, dY/dt (feedforward de velocidad)
        self.sx_d = self.sx.derivative()
        self.sy_d = self.sy.derivative()

    # -------------------------
    #  Callback de Odometría
    # -------------------------
    def _odom_cb(self, msg: Odometry):
        # Posición
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Orientación -> yaw
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, self.yaw = tft.euler_from_quaternion(quat)

        self.has_odom = True

    # -------------------------
    #  Paso de Control periódico
    # -------------------------
    def _control_step(self, event):
        if not self.has_odom:
            return  # todavía no hay odometría

        # Tiempo paramétrico actual desde inicio
        tk = time.time() - self.t0

        # Si excede el final, o cicla o se "clampa" al último valor
        if tk > self.t_knots[-1]:
            if self.loop_traj:
                # En bucle: modulo del total
                tk = tk % self.t_knots[-1]
            else:
                tk = self.t_knots[-1]

        # Referencias por spline (posición y derivada) en el tiempo tk
        x_ref = float(self.sx(tk))
        y_ref = float(self.sy(tk))
        x_dot = float(self.sx_d(tk))
        y_dot = float(self.sy_d(tk))

        # Punto adelantado real del robot
        x_h = self.x + self.D * math.cos(self.yaw)
        y_h = self.y + self.D * math.sin(self.yaw)

        # Error en el punto adelantado
        ex = x_ref - x_h
        ey = y_ref - y_h

        # Jacobiano inverso para pasar de [x_h_dot, y_h_dot] a [v, w]
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        D = self.D if abs(self.D) > 1e-6 else 1e-6  # evitar división por cero

        # Matriz 2x2
        # [  c    s ] [v]   = [x_h_dot]
        # [ -s/D  c/D] [w]  = [y_h_dot]
        # => [v, w]^T = A * ( [x_dot, y_dot]^T + K * [ex, ey]^T )
        A11, A12 = c, s
        A21, A22 = -s / D, c / D

        # Ganancias diagonales
        v_in = x_dot + self.kx * ex
        w_in = y_dot + self.ky * ey

        # Producto A * [v_in, w_in]
        v = A11 * v_in + A12 * w_in
        w = A21 * v_in + A22 * w_in

        # Cinemática diferencial -> velocidades de ruedas (rad/s)
        wL = (v - (w * self.L) / 2.0) / self.R
        wR = (v + (w * self.L) / 2.0) / self.R

        # Limitación
        wL = max(min(wL, self.wheel_limit), -self.wheel_limit)
        wR = max(min(wR, self.wheel_limit), -self.wheel_limit)

        # Publicar
        msg = Float32MultiArray()
        msg.data = [wL, wR]
        self.pub.publish(msg)

        # Logeo suave para depurar sin saturar consola
        rospy.loginfo_throttle(1.0, f"tk={tk:.2f}  ref=({x_ref:.2f},{y_ref:.2f})  e=({ex:.2f},{ey:.2f})  v={v:.2f} w={w:.2f}  wL={wL:.2f} wR={wR:.2f}")


if __name__ == '__main__':
    try:
        SeguidorTrayectoriaSpline()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

