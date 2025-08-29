#!/usr/bin/env python3
import os
from datetime import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt



def _unique_filename(path):
    if not os.path.exists(path):
        return path
    base, ext = os.path.splitext(path)
    i = 1
    while True:
        cand = f"{base}_{i}{ext}"
        if not os.path.exists(cand):
            return cand
        i += 1

def _pick_column(df, candidates):
    """Devuelve la primera columna de df que coincida con alguno en candidates (case-insensitive)."""
    cols = {c.lower(): c for c in df.columns}
    for cand in candidates:
        if cand.lower() in cols:
            return cols[cand.lower()]
    return None

def plot_motors_from_csv(csv_path,
                         outdir='.',
                         save=True,
                         save_mode='timestamp',  # 'timestamp' o 'count'
                         show=False,
                         convert_rads_to_rpm=False):
    """
    Lee csv y grafica comportamiento de ambos motores.
    - csv_path: ruta al csv combinado.
    - outdir: carpeta para guardar imágenes.
    - save: si True guarda las figuras; si False solo muestra (si show=True).
    - save_mode: 'timestamp' (default) o 'count' para evitar sobreescritura.
    - show: si True llama plt.show() al final (útil si estás en máquina con GUI).
    - convert_rads_to_rpm: si True convierte columnas detectadas como rad/s a RPM para graficar (si aplica).
    """
    df = pd.read_csv(csv_path)
    # detectar columna tiempo
    tcol = _pick_column(df, ['t', 'time', 'tiempo', 'timestamp']) or df.columns[0]
    t = df[tcol].values

    # detectar columnas para cada elemento (izq/der)
    vel_izq_col = _pick_column(df, ['vel_izq', 'velocidad_izq', 'omega_izq', 'vel_izquierda', 'vel_izq_meas'])
    vel_der_col = _pick_column(df, ['vel_der', 'velocidad_der', 'omega_der', 'vel_derecha', 'vel_der_meas'])
    i_izq_col   = _pick_column(df, ['i_izq', 'corriente_izq', 'i_izquierda', 'I_izq', 'corriente_izq_mA'])
    i_der_col   = _pick_column(df, ['i_der', 'corriente_der', 'i_derecha', 'I_der', 'corriente_der_mA'])
    vel_izq_est_col = _pick_column(df, ['vel_izq_est', 'vel_izq_estim', 'vel_izq_estimada', 'vel_izq_est'])
    vel_der_est_col = _pick_column(df, ['vel_der_est', 'vel_der_estim', 'vel_der_estimada', 'vel_der_est'])
    i_izq_est_col = _pick_column(df, ['i_izq_est', 'i_izq_estim', 'i_izq_estimada'])
    i_der_est_col = _pick_column(df, ['i_der_est', 'i_der_estim', 'i_der_estimada'])
    u_izq_col = _pick_column(df, ['u_izq', 'uizq', 'u_izquierda', 'u_left', 'u_left'])
    u_der_col = _pick_column(df, ['u_der', 'uder', 'u_derecha', 'u_right', 'u_right'])
    ref_izq_col = _pick_column(df, ['ref_vel_izq', 'vel_ref', 'velocidad_ref', 'reference'])
    ref_der_col = _pick_column(df, ['ref_der_izq', 'vel_ref', 'velocidad_ref', 'reference'])
    # conveniencia: si faltan algunas columnas no rompa todo
    def get_arr(col):
        return df[col].values if (col is not None and col in df.columns) else None

    vel_izq = get_arr(vel_izq_col)
    vel_der = get_arr(vel_der_col)
    i_izq = get_arr(i_izq_col)
    i_der = get_arr(i_der_col)
    vel_izq_est = get_arr(vel_izq_est_col)
    vel_der_est = get_arr(vel_der_est_col)
    i_izq_est = get_arr(i_izq_est_col)
    i_der_est = get_arr(i_der_est_col)
    u_izq = get_arr(u_izq_col)
    u_der = get_arr(u_der_col)
    ref_vel_izq = get_arr(ref_izq_col)
    ref_der_izq = get_arr(ref_der_col)

    # Opcional: si detectas rad/s y prefieres RPM (usuario lo pidió antes). 
    # Aquí convertimos si convert_rads_to_rpm True y los nombres no contienen 'rpm'
    def maybe_convert_rad_s_to_rpm(arr, colname):
        if arr is None: 
            return None
        if not convert_rads_to_rpm:
            return arr
        if colname and 'rpm' in colname.lower():
            return arr  # ya está en rpm
        # asumimos rad/s, convertir
        return arr * 60.0 / (2.0 * np.pi)

    vel_izq = maybe_convert_rad_s_to_rpm(vel_izq, vel_izq_col)
    vel_der = maybe_convert_rad_s_to_rpm(vel_der, vel_der_col)
    vel_izq_est = maybe_convert_rad_s_to_rpm(vel_izq_est, vel_izq_est_col)
    vel_der_est = maybe_convert_rad_s_to_rpm(vel_der_est, vel_der_est_col)
    # si convertiste a RPM, etiqueta axes luego acorde

    # Preparar nombres y carpeta
    os.makedirs(outdir, exist_ok=True)
    base_name = os.path.splitext(os.path.basename(csv_path))[0]

    def make_outpath(suffix):
        fname = f"{base_name}_{suffix}.png"
        fpath = os.path.join(outdir, fname)
        if save_mode == 'timestamp':
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            fpath = os.path.join(outdir, f"{base_name}_{suffix}_{ts}.png")
            if os.path.exists(fpath):
                fpath = _unique_filename(fpath)
        else:  # count
            if os.path.exists(fpath):
                fpath = _unique_filename(fpath)
        return fpath
    
    def customGrafica(t, z, x_est, u, ref, name="data"):
        plt.figure(figsize=(10, 8))
            
        # Velocidad
        plt.subplot(2, 1, 1)
        plt.plot(t, z[1], 'b-', markersize=2, label='Velocidad medida')
        plt.plot(t, x_est[1], 'k-', label='Velocidad estimada')
        
        plt.plot(t, ref, 'g--', label='Velocidad referencia')
        plt.ylabel('Velocidad [rad/s]')
        plt.title(f"Velocidad Angular - {name}")
        plt.grid(True)
        plt.legend()

        # Corriente
        plt.subplot(2, 1, 2)
        plt.plot(t, z[0], 'rx', markersize=2, label='Corriente medida')
        plt.plot(t, x_est[0], 'k-', label='Corriente estimada')
        plt.xlabel('Tiempo [s]')
        plt.ylabel('Corriente [A]')
        plt.title(f"Corriente - {name}")
        plt.grid(True)
        plt.legend()

        plt.tight_layout()

        outpath = make_outpath(f"grafica {name}")
        plt.savefig(outpath)
        print("Guardado:", outpath)

        # accion de control
        plt.figure()
    
        # accion de control
        plt.plot(t, u, 'r--', label='Accion de control')
        plt.ylabel('[V]')
        plt.title(f"Accion de control - {name}")
        plt.grid(True)
        plt.legend()

        outpath = make_outpath(f"grafica u {name}")
        plt.savefig(outpath)
        print("Guardado:", outpath)

    # --- Figuras ---
    customGrafica(t, [vel_izq, i_izq], [vel_izq_est, i_izq_est], u_izq, ref_vel_izq, name="izquierdo")
    customGrafica(t, [vel_der, i_der], [vel_der_est, i_der_est], u_der, ref_der_izq, name="derecho")

    if show:
        plt.show()
    else:
        # cerrar figuras en entornos sin GUI para liberar memoria
        plt.close('all')

        

# --- Ejemplo de uso ---
if __name__ == "__main__":
    # ruta a tu csv combinado
    csv_path = "datos_2.csv"   # <- cambia a tu archivo
    plot_motors_from_csv(csv_path,
                         outdir='.',        # carpeta donde guardar las imágenes
                         save=True,
                         save_mode='count',  # 'timestamp' o 'count'
                         show=True,
                         convert_rads_to_rpm=False)
    

