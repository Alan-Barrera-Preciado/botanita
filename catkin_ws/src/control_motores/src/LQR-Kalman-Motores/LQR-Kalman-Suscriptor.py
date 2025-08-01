#!/usr/bin/env python3

import numpy as np                  #Para operaciones matematicas
import matplotlib.pyplot as plt     #Para graficar
import serial                       #Para comunicacion serial
import time                         #Para timeouts en comunicacion
import pandas as pd                 #Para leer/escribir excel
import rospy
from std_msgs.msg import Float32MultiArray

def callback_ref(msg):
    global Ref_Izq, Ref_Der
    Ref_Izq = msg.data[0]
    Ref_Der = msg.data[1]

def main():
    global Ref_Izq, Ref_Der
    
    rospy.init_node('Controlador_Motores')
    rospy.Suscriber('/Vel_Referencia', Float32MultiArray, callback_ref)
    rate = rospy.Rate(1/dt) 
   
    serialArduino = serial.Serial('/dev/ttyACM0', 115200)
    time.sleep(2)
    dt=0.05

# ----------------- Espacio de estados discretizado (Motor Izquierda 12v) -----------------

    Ad_Izq = np.array([[0.000, -0.0102], [0.0008, 0.7115]])
    Bd_Izq = np.array([[0.0771],[0.4348]])

# ----------------- Espacio de estados discretizado (Motor Derecha 24v) -----------------

    Ad_Der = np.array([[-0.0002, -0.0102], [0.0111, 0.7191]])
    Bd_Der = np.array([[0.0788],[0.3260]])

    C = np.array([0, 1])

# ----------------- Filtro de Kalman (Motor Izquierda 12v) -----------------

    Fk_Izq = Ad_Izq
    Gk_Izq = Bd_Izq
    Hk_Izq = np.eye(2,2)

    Pk_Izq = np.array([[0.15, 0],[0, 0.75]])
    Qk_Izq = np.array([[15, 0],[0, 1.5e-4]])
    Rk_Izq = np.array([[7.5e-7, 0],[0, 9.5e-4]])

    x_hat_Izq = np.array([[0],[0]])

# ----------------- Filtro de Kalman (Motor Derecha 24v) -----------------

    Fk_Der = Ad_Der
    Gk_Der = Bd_Der
    Hk_Der = np.eye(2,2)

    Pk_Der = np.array([[0.15, 0],[0, 1]])
    Qk_Der = np.array([[15, 0],[0, 8.5e-4]])
    Rk_Der = np.array([[5e-5, 0],[0, 8.5e-4]])

    x_hat_Der = np.array([[0],[0]])

# ----------------- LQR (Motor Izquierda 12v) -----------------

    K_Izq = np.array([0.0006, 1.9469])
    Kr_Izq = 2.6104

    u_Izq = 0

# ----------------- LQR (Motor Derecha 24v) -----------------

    K_Der = np.array([0.0058, 0.9447])
    Kr_Der = 1.8046

    u_Der = 0

# ----------------- Predicciones de los estados -----------------

    x_hat_Izq = Fk_Izq @ x_hat_Izq + Gk_Izq * u_Izq
    Pk_Izq = Fk_Izq @ Pk_Izq @ Fk_Izq.T + Qk_Izq

    x_hat_Der = Fk_Der @ x_hat_Der + Gk_Der * u_Der
    Pk_Der = Fk_Der @ Pk_Der @ Fk_Der.T + Qk_Der

    Ref_Izq = 0.0
    Ref_Der = 0.0

# ----------------- Graficas -----------------

    PWM_Izq_plot = []
    u_Izq_plot = []
    Corriente_Izq_plot = []
    Rad_Izq_plot = []
    xhat_Izq_plot = []
    x2hat_Izq_plot = []
    Error_Izq_plot = []
    Referencia_Izq_plot = []
    PWM_Der_plot = []
    u_Der_plot = []
    Corriente_Der_plot = []
    Rad_Der_plot = []
    xhat_Der_plot = []
    x2hat_Der_plot = []
    Error_Der_plot = []
    Referencia_Der_plot = []
    Tiempo_plot = []
    Tiempo = 0
    t = 0

    while not rospy.is_shutdown()
        try:
            RadRef_Izq = Ref_Izq
            RadRef_Der = Ref_Der
            PWM_Izq = max(-12, min(12, u_Izq))
            PWM_Izq =  PWM_Izq/12 * 255
            PWM_Izq = int(np.round(PWM_Izq).item())
            PWM_Der = max(-12, min(12, u_Der))
            PWM_Der =  PWM_Der/12 * 255
            PWM_Der = int(np.round(PWM_Der).item())

            cad = f"{PWM_Izq},{PWM_Der}\n"
            serialArduino.write(cad.encode('ascii'))

            Lectura = serialArduino.readline().decode('ascii').rstrip('\n')
            Motor_Izq, Motor_Der = Lectura.split(':')
            RPM_Izq, Corriente_Izq = map(float, Motor_Izq.split(','))
            RPM_Der, Corriente_Der = map(float, Motor_Der.split(','))
            Rad_Izq = RPM_Izq*0.10472
            Rad_Der = RPM_Der*0.10472
            Corriente_Izq = Corriente_Izq/1000
            Corriente_Der = Corriente_Der/1000

            x_Izq = np.array([[Corriente_Izq],[Rad_Izq]])
            Kk_Izq = Pk_Izq @ Hk_Izq.T @ np.linalg.pinv(Hk_Izq @ Pk_Izq @ Hk_Izq.T + Rk_Izq)
            x_hat_Izq = x_hat_Izq + Kk_Izq @  (x_Izq - Hk_Izq @ x_hat_Izq)
            Pk_Izq = Pk_Izq - Kk_Izq @ Hk_Izq @ Pk_Izq

            x_hat_Izq = Fk_Izq @ x_hat_Izq + Gk_Izq * u_Izq
            Pk_Izq = Fk_Izq @ Pk_Izq @ Fk_Izq.T + Qk_Izq

            Error_Izq = RadRef_Izq - x_Izq[1,0]

            x_Der = np.array([[Corriente_Der],[Rad_Der]])
            Kk_Der = Pk_Der @ Hk_Der.T @ np.linalg.pinv(Hk_Der @ Pk_Der @ Hk_Der.T + Rk_Der)
            x_hat_Der = x_hat_Der + Kk_Der @  (x_Der - Hk_Der @ x_hat_Der)
            Pk_Der = Pk_Der - Kk_Der @ Hk_Der @ Pk_Der

            u_Der = -K_Der @ x_hat_Der + Kr_Der * RadRef_Der
            u_Izq = -K_Izq @ x_hat_Izq + Kr_Izq * RadRef_Izq

            x_hat_Der = Fk_Der @ x_hat_Der + Gk_Der * u_Der
            Pk_Der = Fk_Der @ Pk_Der @ Fk_Der.T + Qk_Der

            Error_Der = RadRef_Der - x_Der[1,0]


            PWM_Izq_plot.append(PWM_Izq)
            u_Izq_plot.append(u_Izq)
            Rad_Izq_plot.append(Rad_Izq)
            Corriente_Izq_plot.append(Corriente_Izq)
            xhat_Izq_plot.append(x_hat_Izq[1,0])
            x2hat_Izq_plot.append(x_hat_Izq[0,0])
            Error_Izq_plot.append(Error_Izq)
            Referencia_Izq_plot.append(RadRef_Izq)

            PWM_Der_plot.append(PWM_Der)
            u_Der_plot.append(u_Der)
            Rad_Der_plot.append(Rad_Der)
            Corriente_Der_plot.append(Corriente_Der)
            xhat_Der_plot.append(x_hat_Der[1,0])
            x2hat_Der_plot.append(x_hat_Der[0,0])
            Error_Der_plot.append(Error_Der)
            Referencia_Der_plot.append(RadRef_Der)

            Tiempo = Tiempo + dt
            Tiempo_plot.append(Tiempo)
            t=t+1
            rate.sleep()

        except rospy.ROSInterruptException:
            break

    time.sleep(dt)
    cad = f"{0},{0}\n"
    serialArduino.write(cad.encode('ascii'))
    serialArduino.close()

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))

    ax1.plot(Tiempo_plot, xhat_Izq_plot, label='Estimacion Izquierda (Rad/s) ')
    ax1.plot(Tiempo_plot, Rad_Izq_plot, label='Medicion Izquierda (Rad/s) ')
    ax1.plot(Tiempo_plot, Referencia_Izq_plot, label='Referencia Izquierda (Rad/s) ')
    ax1.legend(loc='lower left')
    ax1.set_ylabel('Velocidad (Rad/s)')

    ax2.plot(Tiempo_plot, xhat_Der_plot, label='Estimacion Derecha (Rad/s) ')
    ax2.plot(Tiempo_plot, Rad_Der_plot, label='Medicion Derecha (Rad/s) ')
    ax2.plot(Tiempo_plot, Referencia_Der_plot, label='Referencia Derecha (Rad/s) ')
    ax2.legend(loc='lower left')
    ax2.set_ylabel('Velocidad (Rad/s)')

    fig, (ax3, ax4) = plt.subplots(2, 1, figsize=(8, 6))

    ax3.plot(Tiempo_plot, x2hat_Izq_plot, label='Estimacion corriente Izquierda (A)')
    ax3.plot(Tiempo_plot, Corriente_Izq_plot, label= 'Corriente medida Izquierda (A)')
    ax3.legend(loc='upper left')
    ax3.set_ylabel('Corriente (A)')

    ax4.plot(Tiempo_plot, x2hat_Der_plot, label='Estimacion corriente Derecha (A)')
    ax4.plot(Tiempo_plot, Corriente_Der_plot, label= 'Corriente medida Derecha (A)')
    ax4.legend(loc='upper left')
    ax4.set_ylabel('Corriente (A)')

    fig, (ax5, ax6) = plt.subplots(2, 1, figsize=(8, 6))

    ax5.plot(Tiempo_plot, u_Izq_plot, label='Control Izquierda (V)')
    ax5.legend(loc='upper left')
    ax5.set_ylabel('Voltaje (V)')
    ax5.set_xlabel('Tiempo')

    ax6.plot(Tiempo_plot, u_Der_plot, label='Control Derecha (V)')
    ax6.legend(loc='upper left')
    ax6.set_ylabel('Voltaje (V)')
    ax6.set_xlabel('Tiempo')

    fig, (ax7, ax8) = plt.subplots(2, 1, figsize=(8, 6))

    ax7.plot(Tiempo_plot, xhat_Izq_plot, label='Estimacion Izquierda (Rad/s) ')
    ax7.plot(Tiempo_plot, Rad_Izq_plot, label='Medicion Izquierda (Rad/s) ')
    ax7.plot(Tiempo_plot, xhat_Der_plot, label='Estimacion Derecha (Rad/s) ')
    ax7.plot(Tiempo_plot, Rad_Der_plot, label='Medicion Derecha (Rad/s) ')
    ax7.legend(loc='upper left')
    ax7.set_ylabel('Velocidad (Rad/s)')

    ax8.plot(Tiempo_plot, x2hat_Izq_plot, label='Estimacion corriente Izquierda (A)')
    ax8.plot(Tiempo_plot, Corriente_Izq_plot, label= 'Corriente medida Izquierda (A)')
    ax8.plot(Tiempo_plot, x2hat_Der_plot, label='Estimacion corriente Derecha (A)')
    ax8.plot(Tiempo_plot, Corriente_Der_plot, label= 'Corriente medida Derecha (A)')
    ax8.legend(loc='upper left')
    ax8.set_ylabel('Velocidad (Rad/s)')

    fig, (ax9, ax10) = plt.subplots(2, 1, figsize=(8, 6))

    ax9.plot(Tiempo_plot, Error_Izq_plot, label='Error Izquierda (Rad/s)')
    ax9.legend(loc='upper left')
    ax9.set_ylabel('Error')
    ax9.set_xlabel('Tiempo')

    ax10.plot(Tiempo_plot, Error_Der_plot, label='Error Derecha (Rad/s)')
    ax10.legend(loc='upper left')
    ax10.set_ylabel('Error')
    ax10.set_xlabel('Tiempo')

    plt.show()

if __name__ == '__main__':
    main()
