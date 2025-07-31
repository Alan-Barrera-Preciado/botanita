import numpy as np                  #Para operaciones matematicas
import matplotlib.pyplot as plt     #Para graficar
import serial                       #Para comunicacion serial
import time                         #Para timeouts en comunicacion
import pandas as pd                 #Para leer/escribir excel

serialArduino = serial.Serial('COM4', 115200)
time.sleep(2)
intervalo_comunicacion=0.05
tiempo_actual=0

vector_radianes_Izq=[]
vector_corriente_Izq=[]

vector_radianes_Der=[]
vector_corriente_Der=[]

vector_tiempo=[]

vector_radianes_Izq.append(0)
vector_corriente_Izq.append(0)

vector_radianes_Der.append(0)
vector_corriente_Der.append(0)

vector_tiempo.append(0)

N = 400

for k in range(N):
    cad = f"{1}\n"
    serialArduino.write(cad.encode('ascii'))
    Lectura = serialArduino.readline().decode('ascii').rstrip('\n')
    print(Lectura)
    Motor_Izq, Motor_Der = Lectura.split(':')
    Rad_Izq, Corriente_Izq = map(float, Motor_Izq.split(','))
    Rad_Der, Corriente_Der = map(float, Motor_Der.split(','))
    Corriente_Izq = Corriente_Izq/1000
    Corriente_Der = Corriente_Der/1000
    Rad_Izq = Rad_Izq*0.10472
    Rad_Der = Rad_Der*0.10472
    vector_radianes_Izq.append(Rad_Izq)
    vector_corriente_Izq.append(Corriente_Izq)
    vector_radianes_Der.append(Rad_Der)
    vector_corriente_Der.append(Corriente_Der)
    tiempo_actual=tiempo_actual+intervalo_comunicacion
    vector_tiempo.append(tiempo_actual)
    
    k=k+1

time.sleep(intervalo_comunicacion)
cad = f"{0}\n"
serialArduino.write(cad.encode('ascii'))
serialArduino.close()

data_Izq = {'t':vector_tiempo,'Radianes':vector_radianes_Izq, 'Corriente':vector_corriente_Izq}
data_Der = {'t':vector_tiempo,'Radianes':vector_radianes_Der, 'Corriente':vector_corriente_Der}

df_Izq = pd.DataFrame(data_Izq)
df_Izq.to_csv('df_12v.csv',index=False)

df_Der = pd.DataFrame(data_Der)
df_Der.to_csv('df_24v.csv',index=False)