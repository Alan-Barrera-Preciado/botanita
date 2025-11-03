import serial
import pandas as pd
import time
import matplotlib.pyplot as plt

# === CONFIGURACIÓN ===
PORT = "/dev/esp32"
BAUDRATE = 115200
# N = 50000  # Número de muestras

# === Inicialización ===
serialPort = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=2)
serialPort.reset_input_buffer()
time.sleep(0.5)

# === Handshake inicial ===
serialPort.write(bytes([0]))  # HANDSHAKE
time.sleep(0.5)

# === Protocolo ===
START1 = 0xAA
START2 = 0x55
HANDSHAKE_CMD = 0x01
MEASURE_CMD = 0x02
HANDSHAKE_ACK = 0xCC

# === Limpiar buffer antes de empezar ===
serialPort.reset_input_buffer()
time.sleep(0.5)


# para comunicacion #
def send_frame(serialPort, cmd: int):
    START1 = 0xAA
    START2 = 0x55
    serialPort.write(bytes([START1, START2, cmd]))
    # serialPort.flush()

def hexdump(b: bytes) -> str:
    return ' '.join(f'{x:02X}' for x in b)

def do_handshake(serialPort, retries=8, wait_after_open=1.0):
    # === Protocolo ===
    START1 = 0xAA
    START2 = 0x55
    HANDSHAKE_CMD = 0x01
    MEASURE_CMD = 0x02
    HANDSHAKE_ACK = 0xCC
    # esperar que la placa termine de resetear al abrir puerto
    time.sleep(wait_after_open)
    serialPort.reset_input_buffer()
    for attempt in range(1, retries + 1):
        print(f"[Handshake] Intento {attempt}/{retries} ...")
        send_frame(serialPort, HANDSHAKE_CMD)

        # leer hasta N bytes que lleguen en el timeout
        time.sleep(0.05)           # breve espera para que la ESP responda
        resp = serialPort.read(64) # lee hasta 64 bytes disponibles
        print("[Handshake] Raw recv:", resp, "HEX:", hexdump(resp))

        # condición de éxito flexible:
        # 1) si aparece el byte ACK en la respuesta -> OK
        # 2) o si aparece la secuencia [AA,55,ACK] en la respuesta -> OK
        if resp:
            if bytes([HANDSHAKE_ACK]) in resp:
                print("[Handshake] ACK detectado en respuesta.")
                return True
            if bytes([START1, START2, HANDSHAKE_ACK]) in resp:
                print("[Handshake] Frame completo detectado.")
                return True

        # si no, esperar un poco y reintentar (la ESP puede tardar en arrancar)
        time.sleep(0.2)

    print("[Handshake] Fallido tras reintentos.")
    return False


# === Handshake inicial ===
if not do_handshake(serialPort):
    raise Exception("No se pudo establecer el handshake con el ESP32")

print("ya se hizo el handshake")


# === Recolección de datos ===
lista_corrienteD = [] 
lista_rpmD = []
lista_corrienteI = []
lista_rpmI = []
lista_indices = []
lista_tiempo = []

# pwmTest = [20,40,60,80,100,120,140,160,180,200]
pwmTest = [0,255,0,200,0,150,0,100,0,50]
j = 0

t = 0;
dt = 0.05
seg = 2
muestreoMaximo = 400
cambioPWM = int(seg / dt) # seg (2)/dt (0.05) = 40 iteraciones
i = 1

pwm = pwmTest[0]
while True:
    # mandar petición de medida
    send_frame(serialPort, MEASURE_CMD)
    #time.sleep(0.01355) # esperar (al tanteo) # comentar si el dt lo da el ESP32
    #time.sleep(dt) # se tardo 1:06 min
    
    raw_data = serialPort.readline().decode(errors='ignore').strip()
    partes = raw_data.split(',')

    if len(partes) == 4:
        try:
            
            if(t >= muestreoMaximo+1):
                pwm = 0
                serialPort.write(f"{pwm},{pwm}\n".encode())
                time.sleep(1)
                break
            
            if t % cambioPWM > 39.95 and j < len(pwmTest)-1:
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

            # Enviar control (PWM R, PWM L)                
            serialPort.write(f"{pwm},{pwm}\n".encode())
            #print(f"[{t}] CorrienteD: {corrienteD:.2f} mA | RPMD: {rpmD:.2f}")
            #print(f"[{t}] CorrienteI: {corrienteI:.2f} mA | RPMI: {rpmI:.2f}")

            t += dt
            i += 1
            
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
    'Index': lista_indices,
    'tiempo': lista_tiempo,
    'CorrienteD_mA': lista_corrienteD,
    'RPMD': lista_rpmD,
    'CorrienteI_mA': lista_corrienteI,
    'RPMI': lista_rpmI
})

df.to_csv("Dataset_Motores.csv", index=False)
print("Datos guardados en Dataset_Motores.csv")

# === Graficar ===
# plt.plot(lista_indices, lista_corrienteD, label="Corriente Derecha (mA)")
# plt.plot(lista_indices, lista_corrienteI, label="Corriente Izquierda (mA)")
# plt.plot(lista_indices, lista_rpmD, label="RPM Derecha")
# plt.plot(lista_indices, lista_rpmI, label="RPM Izquierda")
# plt.xlabel("Índice")
# plt.title("Mediciones de Motores")
# plt.grid(True)
# plt.legend()
# plt.tight_layout()
# plt.show()

