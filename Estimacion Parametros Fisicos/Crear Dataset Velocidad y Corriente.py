import numpy as np                  #Para operaciones matematicas
import matplotlib.pyplot as plt     #Para graficar
import serial                       #Para comunicacion serial
import time                         #Para timeouts en comunicacion
import pandas as pd                 #Para leer/escribir excel

serialArduino = serial.Serial('COM4', 115200)
time.sleep(2)
intervalo_comunicacion=0.05
tiempo_actual=0

vector_rpm=[]
vector_tiempo=[]

vector_rpm.append(0)
vector_tiempo.append(0)

N = 400

for k in range(N):
    cad = f"{1}\n"
    serialArduino.write(cad.encode('ascii'))
    Lectura = serialArduino.readline().decode('ascii').rstrip('\n')
    RPM, Control = map(float, Lectura.split(','))
    vector_rpm.append(RPM)
    tiempo_actual=tiempo_actual+intervalo_comunicacion
    vector_tiempo.append(tiempo_actual)
    
    k=k+1

time.sleep(intervalo_comunicacion)
cad = f"{0}\n"
serialArduino.write(cad.encode('ascii'))
serialArduino.close()

data = {'t':vector_tiempo,'Radianes':vector_rpm}

df = pd.DataFrame(data)
df.to_csv('df_example_12v.csv',index=False)