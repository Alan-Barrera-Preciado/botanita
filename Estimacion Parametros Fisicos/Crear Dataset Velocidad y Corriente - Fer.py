import serial
import pandas as pd
import time
import matplotlib.pyplot as plt

# === CONFIGURACIÓN ===
PORT = "COM3"
BAUDRATE = 115200
# N = 50000  # Número de muestras

# === Inicialización ===
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

# === Recolección de datos ===
lista_corrienteD = [] 
lista_rpmD = []
lista_corrienteI = []
lista_rpmI = []
lista_indices = []

# pwmTest = [20,40,60,80,100,120,140,160,180,200]
pwmTest = [0,255,0,200,0,150,0,100,0,50]
j = 0

t = 0;
dt = 0.05
seg = 2
muestreoMaximo = 400
cambioPWM = int(seg / dt) # seg (2)/dt (0.05) = 40 iteraciones

pwm = pwmTest[0]
while True:
    serialPort.write(bytes([1]))  # MEASURE_REQUEST
    serialPort.flush()
    #time.sleep(0.01355) # esperar (al tanteo) # comentar si el dt lo da el ESP32
    time.sleep(dt) # se tardo 1:06 min
    
    raw_data = serialPort.readline().decode().strip()
    partes = raw_data.split(',')

    if len(partes) == 4:
        try:
            
            if(t == muestreoMaximo+1):
                pwm = 0
                serialPort.write(f"{pwm},{pwm}\n".encode())
                time.sleep(1)
                break
            
            if t % cambioPWM == 0 and j < len(pwmTest):
                pwm = pwmTest[j]
                j += 1
            
            corrienteD = float(partes[0])
            rpmD = float(partes[1])
            corrienteI = float(partes[2])
            rpmI = float(partes[3])

            lista_corrienteD.append(corrienteD/1000)
            lista_rpmD.append(rpmD*0.10472)
            lista_corrienteI.append(corrienteI/1000)
            lista_rpmI.append(rpmI*0.10472)
            lista_indices.append(t*dt)

            # Enviar control (PWM R, PWM L)                
            serialPort.write(f"{pwm},{pwm}\n".encode())
            #print(f"[{t}] CorrienteD: {corrienteD:.2f} mA | RPMD: {rpmD:.2f}")
            #print(f"[{t}] CorrienteI: {corrienteI:.2f} mA | RPMI: {rpmI:.2f}")

            t += 1
            
            # time.sleep(dt) # dt
        except ValueError:
            # print(f"[{i}] Error al convertir: {raw_data}")
            pass
    else:
        # print(f"[{i}] Formato inválido: {raw_data}")
        pass
    
serialPort.close()

# === Guardar CSV ===
df = pd.DataFrame({
    't': lista_indices,
    'Corriente_Der': lista_corrienteD,
    'Rad_Der': lista_rpmD,
    'Corriente_Izq': lista_corrienteI,
    'Rad_Izq': lista_rpmI
})

df.to_csv("Dataset_Motores.csv", index=False)
print("Datos guardados en mediciones_motores.csv")

# === Graficar ===
plt.plot(lista_indices, lista_corrienteD, label="Corriente Motor Derecha (mA)")
plt.plot(lista_indices, lista_corrienteI, label="Corriente Motor Izquierda (mA)")
plt.plot(lista_indices, lista_rpmD, label="Rad/s Motor Derecha")
plt.plot(lista_indices, lista_rpmI, label="Rad/s Motor Izquierda")
plt.title("Mediciones de Motores")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
