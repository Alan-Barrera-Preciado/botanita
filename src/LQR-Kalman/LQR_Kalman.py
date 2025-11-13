#!/usr/bin/env python3
import serial
import pandas as pd
import time
import matplotlib.pyplot as plt
import numpy as np
import math
from scipy.signal import cont2discrete
from scipy.linalg import solve_discrete_are
import rospkg

import os
from datetime import datetime

import rospy
from std_msgs.msg import Float32MultiArray

rospy.init_node('controlador_motores')
pub = rospy.Publisher('/rads_motores', Float32MultiArray, queue_size=10)

rospack = rospkg.RosPack()
pkg_path = rospack.get_path("botanita")
dataset_dir = os.path.join(pkg_path, "src/LQR-Kalman/datasets")

Kalman_Izq = [[1e-9, 5e-14], [3e-10, 5e-9]]
Kalman_Der = [[1e-9, 5e-14], [3e-10, 5e-9]]

LQR_Izq = [5, 250, 10]
LQR_Der = [5, 250, 10]

def _next_count_filename(path):
    base, ext = os.path.splitext(path)
    i = 1
    while True:
        candidate = f"{base}_{i}{ext}"
        if not os.path.exists(candidate):
            return candidate
        i += 1

def mergeData(name="datos", mode='timestamp', outdir='.', verbose=True):
    if not name.lower().endswith('.csv'):
        name = f"{name}.csv"
    t_izq, z_izq, x_est_izq, u_izq, ref_vel_izq = bot.motorIzquierdo.grafica.guardadosToArray()
    _,      z_der, x_est_der, u_der, ref_vel_der = bot.motorDerecho.grafica.guardadosToArray()

    df = pd.DataFrame({
        't': t_izq,

        'i_izq': z_izq[:, 0],
        'vel_izq':   z_izq[:, 1],
        'i_izq_est': x_est_izq[:, 0],
        'vel_izq_est':   x_est_izq[:, 1],
        'u_izq': u_izq,
        'ref_vel_izq': ref_vel_izq,

        'i_der': z_der[:, 0],
        'vel_der':   z_der[:, 1],
        'i_der_est': x_est_der[:, 0],
        'vel_der_est':   x_est_der[:, 1],
        'u_der': u_der,
        'ref_vel_der': ref_vel_der
    })

    outdir = os.path.expanduser(outdir)
    os.makedirs(outdir, exist_ok=True)
    outpath = os.path.join(outdir, name)

    if mode == 'timestamp':
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        base, ext = os.path.splitext(outpath)
        outpath = f"{base}_{ts}{ext}"
        if os.path.exists(outpath):
            outpath = _next_count_filename(outpath)
    elif mode == 'count':
        if os.path.exists(outpath):
            outpath = _next_count_filename(outpath)
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        base, ext = os.path.splitext(outpath)
        outpath = f"{base}_{ts}{ext}"

    df.to_csv(outpath, index=False)
    if verbose:
        print(f"Datos guardados en: {outpath}")
    return outpath

class Sistema:
    def __init__(self, Ac, Bc, Cc, Dc, dt=None):
        self.Ac = Ac
        self.Bc = Bc
        self.Cc = Cc
        self.Dc = Dc
        self.dt = dt if dt is not None else 0.05
        self.discretizar()
        
    def discretizar(self):
	    self.F, self.G, self.H, self.D, _ = cont2discrete((self.Ac, self.Bc, self.Cc, self.Dc), self.dt, method='zoh')
        
class FiltroKalman:
    def __init__(self, sys: Sistema):
        self.sys = sys
        self.Q_kalman = 0*np.eye(2)
        self.R_kalman = 0*np.eye(2)
        self.inicializar()
        
        self.ruido_proceso = 0
        self.ruido_Medicion = np.array([[0.1], [0.1]])
        
    def inicializar(self):
        self.x_real = np.zeros((2, 1))
        
        self.x_est = np.zeros((2, 1))
        self.x_est = self.sys.F @ self.x_est + self.sys.G * 0 
        self.P = np.array([[0.2,0],[0, 2.3]])
        self.P = self.sys.F @ self.P @ self.sys.F.T + self.Q_kalman
    
    def actualizar(self, z):
        H = self.sys.H          
        S = H @ self.P @ H.T + self.R_kalman
        self.K = self.P @ H.T @ np.linalg.inv(S)
        y = z
        self.x_est = self.x_est + self.K @ (y - H @ self.x_est)
        I = np.eye(self.P.shape[0])
        self.P = (I - self.K @ H) @ self.P
        
    def predecir(self, u, z):
        F, G = self.sys.F, self.sys.G
        self.x_est = F @ self.x_est + G * u
        self.P = F @ self.P @ F.T + self.Q_kalman
         
    def setGananciasQR(self, Q_vec, R_vec):
        self.Q_kalman = np.diag(Q_vec)
        self.R_kalman = np.diag(R_vec)
        self.inicializar()
        
class ControladorLQR:
    def __init__(self, sys: Sistema, Q_lqr=None, R_lqr=None):
        self.sys = sys
        self.Q_lqr = 0.01 * np.eye(2)
        self.R_lqr = 0.01 * np.array([[1.0]])
        self.calcularGanancias()
        
    def calcularGanancias(self):
        A, B = self.sys.F, self.sys.G
        P = solve_discrete_are(A, B, self.Q_lqr, self.R_lqr)
        self.K = np.linalg.inv(B.T @ P @ B + self.R_lqr) @ (B.T @ P @ A)
        Ctrack = np.array([[0.0, 1.0]]) 
        M = np.linalg.inv(np.eye(A.shape[0]) - (A - B @ self.K)) @ B
        self.Kr = np.linalg.inv(Ctrack @ M)
		
    def controlar(self, ref, x_est):
        u = (-self.K @ x_est + self.Kr * ref).item()
        return u
    
    def setPenalizacionesQR(self, Q_vec, R):
        self.Q_lqr =np.diag(Q_vec)
        self.R_lqr = R * np.array([[1.0]])
        self.calcularGanancias()
        
class GraficasSistema:
    def __init__(self):
        self.z_plot = []
        self.x_est_plot = []
        self.t_plot = []
        self.ref_plot = []
        self.u_plot = []
        
        self.formaArray = 0

    def guardar(self, z, x_est, t, ref, u=None):
        self.z_plot.append(z.flatten())
        self.x_est_plot.append(x_est.flatten())
        self.ref_plot.append(ref)
        self.t_plot.append(t)
        if u is not None:            
            self.u_plot.append(u)

    def guardadosToArray(self):
        self.z_plot = np.array(self.z_plot)
        self.x_est_plot = np.array(self.x_est_plot)
        self.t_plot = np.array(self.t_plot)
        self.u_plot = np.array(self.u_plot)
        
        self.formaArray = 1

        return self.t_plot, self.z_plot, self.x_est_plot, self.u_plot, self.ref_plot
               
class MotorKalmanLQR:
    def __init__(self, sys: Sistema):
        self.sys = sys
        self.kalman = FiltroKalman(self.sys)
        self.lqr = ControladorLQR(self.sys)
        self.grafica = GraficasSistema()
        self.u=0
        
    def pasoLectura(self, t, ref, z):
        self.kalman.actualizar(z)
        
        self.u = self.lqr.controlar(ref, self.kalman.x_est)
        u_sat = float(np.clip(self.u, -12.0, 12.0))         
        self.kalman.predecir(u_sat, z)
        self.grafica.guardar(np.array([[z[0]],[z[1]]]), np.array([[abs(self.kalman.x_est[0])],[self.kalman.x_est[1]]]), t, ref, u=u_sat)
        pwm = int(np.clip(u_sat / 12.0, -1.0, 1.0) * 255)
        return pwm, u_sat

class RobotMobilDiff:
    def __init__(self, motorIzquierdo: MotorKalmanLQR, motorDerecho: MotorKalmanLQR, dt, pub_rads):
        self.dt = dt
        self.motorIzquierdo = motorIzquierdo
        self.motorDerecho = motorDerecho
        self.pub = pub_rads
            
    def referenciaMotoresCustom(self, t, ref_Izq, ref_Der, z_Izq, z_Der):
        pwm_izq, uSat_I = self.motorIzquierdo.pasoLectura(t, ref_Izq, z_Izq)
        pwm_Der, uSat_D = self.motorDerecho.pasoLectura(t, ref_Der, z_Der)        
        return pwm_izq, pwm_Der, uSat_I, uSat_D  
        
    def publicarRads(self,Rads_Izq, Rads_Der):
        RadianesSegundo = Float32MultiArray()
        RadianesSegundo.data = [Rads_Izq, Rads_Der]
        self.pub.publish(RadianesSegundo)



dt = 0.01

Rm, Lm, Jm, Bm = 1.26450238e+01, 3.53068540e-01, 3.46318818e-02, 1.14027020e-02
Km = Kb = 0.57145056

Ac = np.array([
    [-Rm/Lm, -Kb/Lm],
    [ Km/Jm, -Bm/Jm]])                                                                  
Bc = np.array([[1 / Lm], [0]])
Cc = np.array([[1, 0],
               [0, 1]]) 
Dc = np.zeros((2, 1))

motor_Izq = MotorKalmanLQR(Sistema(Ac, Bc, Cc, Dc, dt))
motor_Izq.kalman.setGananciasQR([Kalman_Izq[0][0], Kalman_Izq[0][1]], [Kalman_Izq[1][0], Kalman_Izq[1][1]]) 
motor_Izq.lqr.setPenalizacionesQR([LQR_Izq[0], LQR_Izq[1]], LQR_Izq[2]) 

Rm, Lm, Jm, Bm = 1.26450238e+01, 3.53068540e-01, 3.46318818e-02, 1.14027020e-02
Km = Kb = 0.57145056

Ac = np.array([
    [-Rm/Lm, -Kb/Lm],
    [ Km/Jm, -Bm/Jm]])
Bc = np.array([[1 / Lm], [0]])
Cc = np.array([[1, 0],
               [0, 1]])
Dc = np.zeros((2, 1))

motor_Der = MotorKalmanLQR(Sistema(Ac, Bc, Cc, Dc, dt))
motor_Der.kalman.setGananciasQR([Kalman_Der[0][0], Kalman_Der[0][1]], [Kalman_Der[1][0], Kalman_Der[1][1]])
motor_Der.lqr.setPenalizacionesQR([LQR_Der[0], LQR_Der[1]], LQR_Der[2])

bot = RobotMobilDiff(motor_Izq, motor_Der, dt, pub)

def callback_ref(msg):
    global Ref_Izq, Ref_Der
    Ref_Izq = msg.data[0]
    Ref_Der = msg.data[1]

def main(dt):
    global Ref_Izq, Ref_Der
    
    rospy.Subscriber('/vel_referencia', Float32MultiArray, callback_ref)
    rate = rospy.Rate(5)
    PORT = "/dev/esp32"
    BAUDRATE = 115200

    serialPort = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=2)
    time.sleep(2)

    START1 = 0xAA
    START2 = 0x55
    HANDSHAKE_CMD = 0x01
    MEASURE_CMD = 0x02
    HANDSHAKE_ACK = 0xCC

    serialPort.reset_input_buffer()
    time.sleep(0.5)

    if not do_handshake(serialPort):
        raise Exception("No se pudo establecer el handshake con el ESP32")

    print("Ya se hizo el handshake")

    Do_CSV = rospy.get_param("~Do_CSV", False)
    rospy.loginfo(f"CSV logging {'enabled' if Do_CSV else 'disabled'}")

    t = 0
    Ref_Izq, Ref_Der = 0.0, 0.0

    while not rospy.is_shutdown():
        send_frame(serialPort, MEASURE_CMD)
        raw_data = serialPort.readline().decode(errors='ignore').strip()
        partes = raw_data.split(',')
        if len(partes) == 4:
            try:
                corrienteD = float(partes[0])
                rpmD = float(partes[1])
                corrienteI = float(partes[2])
                rpmI = float(partes[3])

                ref_Izq = Ref_Izq
                ref_Der = Ref_Der  

                rpmD = rpmD * np.sign(bot.motorDerecho.u)
                rpmI = rpmI * np.sign(bot.motorIzquierdo.u)

                z_Izq = np.array([
                        [corrienteI / 1000.0],
                        [rpmI * 0.1047197551]
                      ])
                z_Der = np.array([
                        [corrienteD / 1000.0],
                        [rpmD * 0.1047197551]
                      ])

                ref_Der = 3 * np.cos(t/30)
                ref_Izq = 2 + np.sin(t/20)

                pwm_Izq, pwm_Der, uSat_I, uSat_D = bot.referenciaMotoresCustom(t, ref_Izq, ref_Der, z_Izq, z_Der)     
              
                if pwm_Izq < 0:
                     rpmI = rpmI*-1
                if pwm_Der < 0:
                     rpmD = rpmD*-1            
                serialPort.write(f"{pwm_Izq},{pwm_Der}\n".encode())
                t += dt
                
            except rospy.ROSInterruptException:
                break
        
        Radianes_Estimados_Izq = bot.motorIzquierdo.kalman.x_est[1]
        Radianes_Estimados_Der = bot.motorDerecho.kalman.x_est[1]
        if abs(Radianes_Estimados_Izq) < 0.003:
            Radianes_Estimados_Izq = 0.0
        if abs(Radianes_Estimados_Der) < 0.003:
            Radianes_Estimados_Der = 0.0
        bot.publicarRads(Radianes_Estimados_Izq, Radianes_Estimados_Der)
            
    serialPort.close()

    if Do_CSV:
        CSV_Name = rospy.get_param("~CSV_Name", "datos")
        ruta = mergeData(CSV_Name, mode='count', outdir=dataset_dir)

def send_frame(serialPort, cmd: int):
    START1 = 0xAA
    START2 = 0x55
    serialPort.write(bytes([START1, START2, cmd]))

def hexdump(b: bytes) -> str:
    return ' '.join(f'{x:02X}' for x in b)

def do_handshake(serialPort, retries=8, wait_after_open=1.0):
    START1 = 0xAA
    START2 = 0x55
    HANDSHAKE_CMD = 0x01
    MEASURE_CMD = 0x02
    HANDSHAKE_ACK = 0xCC
    time.sleep(wait_after_open)
    serialPort.reset_input_buffer()
    for attempt in range(1, retries + 1):
        print(f"[Handshake] Intento {attempt}/{retries} ...")
        send_frame(serialPort, HANDSHAKE_CMD)
        time.sleep(0.05)
        resp = serialPort.read(64)
        print("[Handshake] Raw recv:", resp, "HEX:", hexdump(resp))
        if resp:
            if bytes([HANDSHAKE_ACK]) in resp:
                print("[Handshake] ACK detectado en respuesta.")
                return True
            if bytes([START1, START2, HANDSHAKE_ACK]) in resp:
                print("[Handshake] Frame completo detectado.")
                return True
        time.sleep(0.2)

    print("[Handshake] Fallido tras reintentos.")
    return False

if __name__ == '__main__':
    main(dt)