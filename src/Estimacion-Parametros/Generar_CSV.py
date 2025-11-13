import serial
import pandas as pd
import time
import matplotlib.pyplot as plt

PORT = "/dev/esp32"
BAUDRATE = 115200

serialPort = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=2)
serialPort.reset_input_buffer()
time.sleep(0.5)

serialPort.write(bytes([0]))
time.sleep(0.5)

START1 = 0xAA
START2 = 0x55
HANDSHAKE_CMD = 0x01
MEASURE_CMD = 0x02
HANDSHAKE_ACK = 0xCC

serialPort.reset_input_buffer()
time.sleep(0.5)

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

if not do_handshake(serialPort):
    raise Exception("No se pudo establecer el handshake con el ESP32")

print("ya se hizo el handshake")

lista_corrienteD = [] 
lista_rpmD = []
lista_corrienteI = []
lista_rpmI = []
lista_indices = []
lista_tiempo = []

pwmTest = [0,255,0,200,0,150,0,100,0,50]
j = 0

t = 0
dt = 0.05
seg = 2
muestreoMaximo = 400
cambioPWM = int(seg / dt)
i = 1

pwm = pwmTest[0]
while True:
    send_frame(serialPort, MEASURE_CMD)
    raw_data = serialPort.readline().decode(errors='ignore').strip()
    partes = raw_data.split(',')
    if len(partes) == 4:
        try:
            
            if(t > 20):
                pwm = 0
                serialPort.write(f"{pwm},{pwm}\n".encode())
                time.sleep(1)
                break
            
            if t % 2 > 1.95 and j < len(pwmTest)-1:
                j += 1
                pwm = pwmTest[j]
            
            print(pwm)
            print(j)
            corrienteD = float(partes[0])
            rpmD = float(partes[1])
            corrienteI = float(partes[2])
            rpmI = float(partes[3])

            lista_corrienteD.append(corrienteD/1000)
            lista_rpmD.append(rpmD*0.10472)
            lista_corrienteI.append(corrienteI/1000)
            lista_rpmI.append(rpmI*0.10472)
            lista_indices.append(i)
            lista_tiempo.append(t)
              
            serialPort.write(f"{pwm},{pwm}\n".encode())

            t += dt
            i += 1
            time.sleep(dt)
            
        except ValueError:
            pass
    else:
        pass
    
serialPort.close()

df = pd.DataFrame({
    'Index': lista_indices,
    'tiempo': lista_tiempo,
    'CorrienteD_mA': lista_corrienteD,
    'RPMD': lista_rpmD,
    'CorrienteI_mA': lista_corrienteI,
    'RPMI': lista_rpmI
})

df.to_csv("Dataset_Motores.csv", index=False)
print("Datos guardados en Dataset_Motores.csv")