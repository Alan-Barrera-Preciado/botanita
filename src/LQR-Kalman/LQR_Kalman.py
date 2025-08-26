#!/usr/bin/env python3

import serial
import pandas as pd
import time
import matplotlib.pyplot as plt
import numpy as np
import math
from scipy.signal import cont2discrete
from scipy.linalg import solve_discrete_are

import rospy
from std_msgs.msg import Float32MultiArray

# ===== OBJETOS PARA OBJETO MOTOR =====

class Sistema:
    def __init__(self, Ac, Bc, Cc, Dc, dt=None):
        self.Ac = Ac
        self.Bc = Bc
        self.Cc = Cc
        self.Dc = Dc
        self.dt = dt if dt is not None else 0.05
        self.discretizar()
        
    def discretizar(self):
	    # Discretización por retención de orden cero (ZOH)
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
        self.P = np.eye(2)
        self.P = self.sys.F @ self.P @ self.sys.F.T + self.Q_kalman
    
    def actualizar(self, z):
        H = self.sys.H            # Dimensiones 2x2 (identidad si Cc=I)
        # Ganancia de Kalman
        S = H @ self.P @ H.T + self.R_kalman
        self.K = self.P @ H.T @ np.linalg.inv(S)
        # Innovación con medición (NO usar x_real = z)
        y = z
        self.x_est = self.x_est + self.K @ (y - H @ self.x_est)
        # Actualización de covarianza
        I = np.eye(self.P.shape[0])
        self.P = (I - self.K @ H) @ self.P
        
    def predecir(self, u, z): # x_real=medicion # se puede quitar z por que ya se pasa a self.xreal en act.
        # Evolución del sistema (dinámica + ruido)
        F, G = self.sys.F, self.sys.G
        # self.x_real = F @ self.x_real + G * u + self.ruido_proceso * np.random.randn(2, 1) # solo en sim
        # self.x_real = z # ya esta en actualizar
        # Predicción del filtro
        self.x_est = F @ self.x_est + G * u
        self.P = F @ self.P @ F.T + self.Q_kalman
                
    #test
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
		# Usar matrices DISCRETAS para LQR discreto
        A, B = self.sys.F, self.sys.G

		# Riccati discreto
        P = solve_discrete_are(A, B, self.Q_lqr, self.R_lqr)

		# K discreto: (B' P B + R)^(-1) (B' P A)
        self.K = np.linalg.inv(B.T @ P @ B + self.R_lqr) @ (B.T @ P @ A)

		# Ganancia de referencia Kr para SEGUIMIENTO DE VELOCIDAD (salida y = [0 1] x)
        Ctrack = np.array([[0.0, 1.0]])  # seleccionar la velocidad como salida a seguir

		# Kr = (C (I - A + B K)^(-1) B)^(-1)
        M = np.linalg.inv(np.eye(A.shape[0]) - (A - B @ self.K)) @ B
        self.Kr = np.linalg.inv(Ctrack @ M)  # escalar 1x1 si SISO
		
    def controlar(self, ref, x_est):
        u = (-self.K @ x_est + self.Kr * ref).item()
        return u
    
    # test
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


    def graficar(self, titulo="", save=0):
        if self.formaArray == 0:
            self.guardadosToArray()
        
        plt.figure(figsize=(10, 8))
        
        # Velocidad
        plt.subplot(2, 1, 1)
        plt.plot(self.t_plot, self.z_plot[:, 1], 'bx', markersize=2, label='Velocidad medida')
        plt.plot(self.t_plot, self.x_est_plot[:, 1], 'k-', label='Velocidad estimada')
        plt.plot(self.t_plot, self.ref_plot, 'g--', label='Velocidad referencia')
        plt.ylabel('Velocidad [rad/s]')
        plt.title(f"Velocidad Angular - {titulo}")
        plt.grid(True)
        plt.legend()

        # Corriente
        plt.subplot(2, 1, 2)
        plt.plot(self.t_plot, self.z_plot[:, 0], 'rx', markersize=2, label='Corriente medida')
        plt.plot(self.t_plot, self.x_est_plot[:, 0], 'k-', label='Corriente estimada')
        plt.xlabel('Tiempo [s]')
        plt.ylabel('Corriente [A]')
        plt.title(f"Corriente - {titulo}")
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        # plt.show()
        plt.savefig(f"{titulo}_vel_corr.png")
    
        if len(self.u_plot) != 0:
            plt.figure()
            
            print(np.shape(self.u_plot))
            print(np.shape(self.t_plot))
            
            # accion de control
            plt.plot(self.t_plot, self.u_plot, 'r--', label='Accion de control')
            plt.ylabel('[V]')
            plt.title(f"Accion de control - {titulo}")
            plt.grid(True)
            plt.legend()
            
            #plt.show()
            plt.savefig(f"{titulo}_u.png")

            
    def guardarExcel(self, name=""):
        if self.formaArray == 0:
            self.guardadosToArray()
            
        df = pd.DataFrame({
            't': self.t_plot,
            'rad_s': self.z_plot[:, 0],
            'CorrienteD_mA': self.z_plot[:, 1],
        })
        
        df.to_csv(f"{name}.csv", index=False)
        print("Datos guardados en " + name)
        
        
        
class MotorKalmanLQR:
    def __init__(self, sys: Sistema):
        self.sys = sys
        self.kalman = FiltroKalman(self.sys) # Q_gain | R_gain
        self.lqr = ControladorLQR(self.sys) # Q_penalizacion | R_penalizacion
        self.grafica = GraficasSistema()
        
    def pasoLectura(self, t, ref, z):
        # z = self.kalman.simular_medicion()
        self.kalman.actualizar(z)
        
        #u = 5
        u = self.lqr.controlar(ref, self.kalman.x_est)  # voltios teóricos
        u_sat = float(np.clip(u, -12.0, 12.0))          # saturación de hardware

	    	# Predicción de Kalman con la acción realmente aplicada
        self.kalman.predecir(u_sat, z)

		    # Guardado para gráficas
        self.grafica.guardar(z, self.kalman.x_est, t, ref, u=u_sat)

		    # PWM (SÍMBOLO del signo y magnitud)
        pwm = int(np.clip(u_sat / 12.0, -1.0, 1.0) * 255)

        if pwm < 10:
            rompe_fr = 0
        else:
            rompe_fr = 10

        return pwm+rompe_fr, u_sat # pwm+15 para romper friccion estatica inicial

class RobotMobilDiff:
    def __init__(self, motorIzquierdo: MotorKalmanLQR, motorDerecho: MotorKalmanLQR, dt):
        self.dt = dt
        self.motorIzquierdo = motorIzquierdo
        self.motorDerecho = motorDerecho
            
    def referenciaMotoresCustom(self, t, ref_Izq, ref_Der, z_Izq, z_Der):
        pwm_izq, uSat_I = self.motorIzquierdo.pasoLectura(t, ref_Izq, z_Izq)
        pwm_Der, uSat_D = self.motorDerecho.pasoLectura(t, ref_Der, z_Der)        
        return pwm_izq, pwm_Der, uSat_I, uSat_D  
        
    def publicarRPM(self,RPM_Izq, RPM_Der):
        RPM = Float32MultiArray()
        RPM.data = [RPM_Izq, RPM_Der]
        pub.publish(RPM)

dt = 0.025 # dt para ambos sistemas

########################## MOTOR 1 (Izquierdo) ##########################
Rm, Lm, Jm, Bm = 1.26450238e+01, 3.53068540e-01, 3.46318818e-02, 1.14027020e-02 # JALAN
Km = Kb = 2.98696654e-01

# Matrices del sistema continuo (x = [i, w])
Ac = np.array([
    [-Rm/Lm, -Kb/Lm],
    [ Km/Jm, -Bm/Jm]])                                                                  
Bc = np.array([[1 / Lm], [0]])
Cc = np.array([[1, 0],
               [0, 1]]) # Medimos corriente y velocidad (en el mismo orden del estado)
Dc = np.zeros((2, 1)) # Dc debe tener 2 filas (una por salida)

# Declaracion del sistema de MOTOR 1 controlado con LQR y kalman
motor_Izq = MotorKalmanLQR(Sistema(Ac, Bc, Cc, Dc, dt))
# configuraciones Kalman
motor_Izq.kalman.setGananciasQR([1e-15, 5e-15], [1e-20, 9.5e-14]) # Ganancias Q R
# configuraciones LQR penalizacion ([I, V], | R) 
motor_Izq.lqr.setPenalizacionesQR([1, 125], 45) # Penalizacion Q (referencia) | Penalizacion R (accion control)

########################## MOTOR 2 (Derecho) ##########################
Rm, Lm, Jm, Bm = 1.26450238e+01, 3.53068540e-01, 3.46318818e-02, 1.14027020e-02 # JALAN
Km = Kb = 0.57145056

Ac = np.array([
    [-Rm/Lm, -Kb/Lm],
    [ Km/Jm, -Bm/Jm]])
Bc = np.array([[1 / Lm], [0]])# Matrices del sistema continuo (x = [i, w])
Cc = np.array([[1, 0],
               [0, 1]]) # Medimos corriente y velocidad (en el mismo orden del estado)
Dc = np.zeros((2, 1)) # Dc debe tener 2 filas (una por salida)

# Declaracion del sistema de MOTOR 2 controlado con LQR y kalman
motor_Der = MotorKalmanLQR(Sistema(Ac, Bc, Cc, Dc, dt))

# configuraciones Kalman
motor_Der.kalman.setGananciasQR([1e-15, 5e-15], [1e-25, 13e-14]) # Ganancias Q R
# configuraciones LQR penalizacion ([I, V], | R) 
motor_Der.lqr.setPenalizacionesQR([1, 125], 45) # Penalizacion Q (referencia) | Penalizacion R (accion control)

############# Robot Diferencial #############

bot = RobotMobilDiff(motor_Izq, motor_Der, dt)

def callback_ref(msg):
    global Ref_Izq, Ref_Der
    Ref_Izq = msg.data[0]
    Ref_Der = msg.data[1]

def main(dt):
    global Ref_Izq, Ref_Der
    
    rospy.init_node('controlador_motores')
    rospy.Subscriber('/vel_referencia', Float32MultiArray, callback_ref)
    pub = rospy.Publisher('/rpm_medido', Float32MultiArray, queue_size=10)
    # dt = 0.025
    rate = rospy.Rate(1/dt) 
   
    # serialArduino = serial.Serial('/dev/ttyACM0', 115200)
    # time.sleep(2)

    # ############# CONFIGURACIÓN SERIAL #############
    PORT = "/dev/ttyUSB0"
    BAUDRATE = 115200
    # N = 50000  # Número de muestras

    # === Inicialización SERIAL ===
    serialPort = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=2)
    serialPort.reset_input_buffer()
    time.sleep(0.5)

    # === Handshake inicial ===
    serialPort.write(bytes([0]))  # HANDSHAKE
    time.sleep(0.5)
    respuesta = serialPort.readline().decode().strip()
    if respuesta != "OK":
        raise Exception("Handshake fallido")
    print("Handshake exitoso")

    t = 0
    Ref_Izq, Ref_Der = 0.0, 0.0

    while not rospy.is_shutdown():
        serialPort.write(bytes([1]))  # MEASURE_REQUEST
        serialPort.flush()
        
        raw_data = serialPort.readline().decode().strip()
        partes = raw_data.split(',')

        if len(partes) == 4:
            try:
                time.sleep(0.025)
                # separar de PARTES del serial
                corrienteD = float(partes[0])
                # voltajeD = float(partes[1])
                rpmD = float(partes[1])
                corrienteI = float(partes[2])
                # voltajeI = float(partes[4])
                rpmI = float(partes[3])
                
                # ref_Izq = 8 - 8*math.exp(-t)
                # ref_Izq = 3 + math.sin(t) + math.cos(t)
                # ref_Der = ref_Izq
                
                ref_Izq = Ref_Izq
                ref_Der = Ref_Der  

                # Orden: z = [i; w] = [corriente (A); velocidad (rad/s)]
                z_Izq = np.array([
                        [corrienteI / 1000.0],
                        [rpmI * 0.1047197551]
                      ])
                z_Der = np.array([
                        [corrienteD / 1000.0],
                        [rpmD * 0.1047197551]
                      ])
                            
                pwm_Izq, pwm_Der, uSat_I, uSat_D = bot.referenciaMotoresCustom(t, ref_Izq, ref_Der, z_Izq, z_Der)     
              
                '''
                if t >= 10:
                    serialPort.write("0,0\n".encode())
                    break
                '''
                bot.PublicarRPM(rpmI, rpmD)
                # Enviar control (PWM R, PWM L)                
                serialPort.write(f"{pwm_Izq},{pwm_Der}\n".encode())
                #print(f"[{t}] CorrienteD: {corrienteD:.2f} mA | RPMD: {rpmD:.2f}")
                #print(f"[{t}] CorrienteI: {corrienteI:.2f} mA | RPMI: {rpmI:.2f}")

                t += dt
            except rospy.ROSInterruptException:
                break

            
    serialPort.close()

    bot.motorDerecho.grafica.graficar("motorDerecho", 1)
    bot.motorIzquierdo.grafica.graficar("motorIzquierdo")
    plt.show()

if __name__ == '__main__':
    main(dt)
