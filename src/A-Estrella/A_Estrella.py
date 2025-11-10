#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
A* sobre PGM con dilatación y opción para VOLTEAR el mapa en Y.
Adaptado para ser ejecutado dentro de un paquete ROS.
"""
import math
import os
import yaml
import csv
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import binary_dilation
import sys 
import rospy


rospy.init_node('a_estrella', anonymous=True)

# --- Importe y Configuración de Rutas ROS ---
try:
    import rospkg
    
    # Reemplaza 'botanita' con el nombre exacto de tu paquete ROS si es diferente
    ROS_PACKAGE_NAME = "botanita" 
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path(ROS_PACKAGE_NAME)
    Map_dir = os.path.join(pkg_path, "src", "Mapas")    
    Map_Name = rospy.get_param("~Map_Name", "Mapa")
    # Rutas absolutas para los mapas (Input)
    if not Map_Name.lower().endswith('.pgm'):
        Map_pgm = f"{Map_Name}.pgm"
    RUTA_PGM = os.path.join(Map_dir, Map_pgm)
    if not Map_Name.lower().endswith('.yaml'):
        Map_yaml = f"{Map_Name}.yaml"
    RUTA_YAML = os.path.join(Map_dir, Map_yaml)
    
    # Rutas absolutas para la trayectoria (Output)
    CSV_DIR = os.path.join(pkg_path, "src", "Trayectorias")
    Route_Name = rospy.get_param("~Route_Name", "Ruta")
    if not Route_Name.lower().endswith('.csv'):
        Route = f"{Route_Name}.csv"
    RUTA_CSV_SALIDA = os.path.join(CSV_DIR, Route)
    
    # Crear el directorio de salida si no existe
    if not os.path.exists(CSV_DIR):
        os.makedirs(CSV_DIR)
        print(f"[Info] Directorio de trayectorias creado: {CSV_DIR}")
        
    print(f"[Info] Usando rutas absolutas ROS desde el paquete '{ROS_PACKAGE_NAME}'.")

except Exception as e:
    # Fallback si rospkg falla o el paquete no se encuentra (e.g., ejecutando fuera de un entorno ROS)
    print(f"AVISO: Fallo al configurar rutas ROS ({e}). Usando rutas relativas por defecto.")
    RUTA_PGM = "Mapa_Abierto.pgm"
    RUTA_YAML = "Mapa_Abierto.yaml"
    RUTA_CSV_SALIDA = "ruta_metros_prueba.csv"

# ---------------- Parámetros editables ----------------
RADIO_DILATACION = 2             # píxeles para inflar paredes
UMBRAL_LIBRE = 250               # >= blanco
FIGSIZE = (12,12)

# --- Ajustes de Y / volteo ---
FLIP_MAP_Y = True
FLIP_Y_OVERRIDE = None

# --- Parámetros de A* ---
# Heurística: Usa "manhattan" o "euclidiana"
HEURISTICA_DISTANCIA = rospy.get_param("~Distance", "manhattan")
# Vecindario: Usa "moore" (ortogonal + diagonal) o "neumann" (solo ortogonal)
TIPO_VECINDARIO = rospy.get_param("~Neighborhood", "neumann")
# -----------------------------------------------------

# ---------------- Nodo y A* (tu versión) ----------------
class Nodo:
    # Se corrige el nombre de los métodos de inicialización y comparación
    def __init__(self, posicion, padre=None, costo=0):
        self.posicion = posicion
        self.padre = padre
        self.costo = costo
        self.g = float('inf')
        self.h = 0
        self.f = float('inf')
    def __eq__(self, otro):
        return self.posicion == otro.posicion

class BusquedaAEstrella:
    # Se corrige el nombre del método de inicialización
    def __init__(self, cuadricula, tipo_distancia=HEURISTICA_DISTANCIA, tipo_vecindario=TIPO_VECINDARIO): 
        self.cuadricula = cuadricula
        self.alto = len(cuadricula)
        self.ancho = len(cuadricula[0])
        self.tipo_distancia = tipo_distancia
        self.tipo_vecindario = tipo_vecindario

        # VALIDACIÓN DE HEURÍSTICA
        if self.tipo_distancia not in ["manhattan", "euclidiana"]:
            print(f"ERROR FATAL: El parámetro HEURISTICA_DISTANCIA ('{self.tipo_distancia}') es inválido.")
            print("Debe ser 'manhattan' o 'euclidiana'.")
            sys.exit(1)
         
        # VALIDACIÓN DE VECINDARIO
        if self.tipo_vecindario not in ["moore", "neumann"]:
            print(f"ERROR FATAL: El parámetro TIPO_VECINDARIO ('{self.tipo_vecindario}') es inválido.")
            print("Debe ser 'moore' (diagonal) o 'neumann' (ortogonal).")
            sys.exit(1)


    def buscar(self, inicio, fin):
        # Chequeo rápido de inicio/fin inválidos en la cuadrícula
        if inicio[0] < 0 or inicio[0] >= self.alto or inicio[1] < 0 or inicio[1] >= self.ancho or self.cuadricula[inicio[0]][inicio[1]] == 1:
            return "ERROR_INICIO_INVALIDO"
        if fin[0] < 0 or fin[0] >= self.alto or fin[1] < 0 or fin[1] >= self.ancho or self.cuadricula[fin[0]][fin[1]] == 1:
            return "ERROR_FIN_INVALIDO"
         
        nodo_inicio = Nodo(inicio)
        nodo_fin = Nodo(fin)
        nodo_inicio.g = 0
        nodo_inicio.h = self._heuristica(inicio, fin)
        nodo_inicio.f = nodo_inicio.h

        lista_abierta = [nodo_inicio]
        lista_cerrada = []

        while lista_abierta:
            nodo_actual = min(lista_abierta, key=lambda n: n.f)
            lista_abierta.remove(nodo_actual)
            lista_cerrada.append(nodo_actual)

            if nodo_actual.posicion == nodo_fin.posicion:
                return self._reconstruir_camino(nodo_actual)

            for vecino in self._obtener_vecinos(nodo_actual):
                if any(v.posicion == vecino.posicion for v in lista_cerrada):
                    continue

                g_tentativo = nodo_actual.g + vecino.costo
                existente = next((v for v in lista_abierta if v.posicion == vecino.posicion), None)

                if existente is None or g_tentativo < existente.g:
                    vecino.g = g_tentativo
                    vecino.h = self._heuristica(vecino.posicion, nodo_fin.posicion)
                    vecino.f = vecino.g + vecino.h
                    vecino.padre = nodo_actual
                    if existente is None:
                        lista_abierta.append(vecino)
         
        return "ERROR_RUTA_NO_ALCANZABLE"

    def _obtener_vecinos(self, nodo):
         
        # Lógica para seleccionar el vecindario (Moore o von Neumann)
        movimientos_ortogonales = [(0,1,1), (0,-1,1), (1,0,1), (-1,0,1)]
        movimientos_diagonales = [(1,1,1.4), (-1,-1,1.4), (1,-1,1.4), (-1,1,1.4)]
         
        if self.tipo_vecindario == "moore":
            movimientos = movimientos_ortogonales + movimientos_diagonales
        elif self.tipo_vecindario == "neumann":
            movimientos = movimientos_ortogonales
        else:
            movimientos = movimientos_ortogonales 

        vecinos = []
        for dr, dc, costo in movimientos:
            r, c = nodo.posicion[0] + dr, nodo.posicion[1] + dc
            if 0 <= r < self.alto and 0 <= c < self.ancho and self.cuadricula[r][c] == 0:
                vecinos.append(Nodo((r, c), costo=costo))
        return vecinos

    def _heuristica(self, a, b):
        if self.tipo_distancia == "manhattan":
            return abs(a[0]-b[0]) + abs(a[1]-b[1])
        # Ya que se validó, si no es manhattan, debe ser euclidiana.
        else: 
             return math.hypot(a[0]-b[0], a[1]-b[1])

    def _reconstruir_camino(self, nodo_final):
        camino = []
        actual = nodo_final
        while actual:
            camino.append(actual.posicion)
            actual = actual.padre
        return camino[::-1]

# ---------------- Procesamiento PGM / dilatación ----------------
def cargar_mapa_pgm(ruta_pgm, umbral_libre=UMBRAL_LIBRE, flip_y=False):
    if not os.path.exists(ruta_pgm):
        # Muestra la ruta que no se encontró
        raise FileNotFoundError(f"No existe PGM en la ruta absoluta: '{ruta_pgm}'")
    with Image.open(ruta_pgm) as img:
        mapa = np.array(img.convert('L'))
    if flip_y:
        print("[Info] Volteando verticalmente el mapa (flip Y) para corregir orientación.")
        mapa = np.flipud(mapa)
    cuadricula = np.ones_like(mapa, dtype=np.uint8)
    cuadricula[mapa >= umbral_libre] = 0    # 0 libre, 1 obstáculo
    print(f"Mapa '{ruta_pgm}' cargado: {mapa.shape[1]}x{mapa.shape[0]} píxeles. (flip_y={flip_y})")
    return mapa, cuadricula

def dilatar_paredes(cuadricula, radio_dilatacion=3):
    """Dilata únicamente las paredes (1) sin tocar el fondo libre."""
    if radio_dilatacion <= 0:
        return cuadricula
    obst = cuadricula == 1
    estructura = np.ones((2*radio_dilatacion+1, 2*radio_dilatacion+1), dtype=bool)
    obst_dil = binary_dilation(obst, structure=estructura)
    cuadricula_dilatada = cuadricula.copy()
    cuadricula_dilatada[obst_dil] = 1
    return cuadricula_dilatada

def detectar_bbox_util(mapa):
    alto, ancho = mapa.shape
    bordes = np.concatenate([mapa[0,:], mapa[-1,:], mapa[:,0], mapa[:,-1]])
    fondo = int(np.median(bordes))
    mascara = mapa != fondo
    coords = np.argwhere(mascara)
    if coords.size == 0:
        return (0, 0, alto, ancho)
    min_r, min_c = coords.min(axis=0)
    max_r, max_c = coords.max(axis=0)
    return min_r, min_c, max_r, max_c

# ---------------- Visualización / selección (devuelve zoom limits) ----------------
def mostrar_mapa_y_seleccionar(mapa):
    Two_Points = rospy.get_param("~Two_Points", False)
    bbox = detectar_bbox_util(mapa)
    min_r, min_c, max_r, max_c = bbox
    plt.figure(figsize=FIGSIZE)
    plt.imshow(mapa, cmap='gray', origin='lower')
    x_min = max(min_c-50, 0)
    x_max = min(max_c+50, mapa.shape[1])
    y_min = max(min_r-50, 0)
    y_max = min(max_r+50, mapa.shape[0])
    plt.xlim(x_min, x_max)
    plt.ylim(y_min, y_max)
    if Two_Points:
        plt.title("Selecciona INICIO y FIN (clic con el mouse) — mapa DILATADO mostrado")
        plt.xlabel("Eje X (pixeles)")
        plt.ylabel("Eje Y (pixeles)")
        puntos = plt.ginput(2, timeout=-1)
        plt.close()
    else:
        plt.title("Selecciona FIN (clic con el mouse) — mapa DILATADO mostrado")
        plt.xlabel("Eje X (pixeles)")
        plt.ylabel("Eje Y (pixeles)")
        punto_inicio = (1024,1024)
        punto_final = plt.ginput(1, timeout=-1)[0]
        puntos = [punto_inicio, punto_final]
        plt.close()
    zoom_limits = (x_min, x_max, y_min, y_max)
    puntos_int = [(int(p[0]), int(p[1])) for p in puntos]
    return puntos_int, zoom_limits

def mostrar_resultado(mapa, camino, zoom_limits_px): 
    plt.figure(figsize=FIGSIZE)
    plt.imshow(mapa, cmap='gray', origin='lower')
    
    # Aplicar el mismo zoom que la ventana de selección
    x_min, x_max, y_min, y_max = zoom_limits_px
    plt.xlim(x_min, x_max)
    plt.ylim(y_min, y_max)
    
    if camino:
        camino = np.array(camino)
        plt.plot(camino[:,1], camino[:,0], color='red', linewidth=2, label='Ruta A*')
        plt.scatter(camino[0,1], camino[0,0], color='lime', s=100, label='Inicio')
        plt.scatter(camino[-1,1], camino[-1,0], color='cyan', s=100, label='Fin')
        plt.legend()
    plt.title("Resultado A* (con dilatación visual aplicada)")
    plt.xlabel("X (pixeles)"); plt.ylabel("Y (pixeles)")
    plt.show()

# ---------------- YAML loader (solo después de obtener ruta) ----------------
def cargar_config_mapa_yaml(ruta_yaml):
    if not os.path.exists(ruta_yaml):
        print(f"[Aviso] YAML '{ruta_yaml}' no encontrado. No se podrá convertir a metros.")
        return None
    try:
        with open(ruta_yaml, 'r') as f:
            cfg = yaml.safe_load(f)
        # El cálculo del dir_base original no es necesario con rutas absolutas de ROS
        return cfg
    except Exception as e:
        print(f"[Aviso] Error leyendo YAML: {e}. No se podrá convertir a metros.")
        return None

# ---------------- Conversión píxel -> mundo (centro del píxel) ----------------
def pixel_a_mundo(row, col, resolucion, origen_x, origen_y, alto_mapa, flip_y_effective=False):
    x = origen_x + col * resolucion + resolucion/2.0
    if not flip_y_effective:
        y = origen_y + (alto_mapa - 1 - row) * resolucion + resolucion/2.0
    else:
        y = origen_y + row * resolucion + resolucion/2.0
    return (x, y)

# ---------------- Función para guardar a CSV ----------------
def guardar_ruta_a_csv(ruta_mundos, ruta_csv):
    """Guarda una lista de tuplas (x, y) en un archivo CSV."""
    if not ruta_mundos:
        print("[Aviso] La ruta en metros está vacía, no se guardará CSV.")
        return
    
    try:
        with open(ruta_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            # Escribir encabezados
            writer.writerow(['x', 'y'])
            # Escribir los puntos (x, y)
            for x, y in ruta_mundos:
                writer.writerow([x, y])
        print(f"Ruta guardada exitosamente en: '{ruta_csv}'")
    except Exception as e:
        print(f"[Error] No se pudo guardar el archivo CSV: {e}")

# ---------------- Mostrar resultado en metros (misma vista convertida) ----------------
def mostrar_resultado_metros(mapa_arr, ruta_mundos, cfg, zoom_limits_px):
    if cfg is None:
        print("[Aviso] No hay YAML válido; no se puede mostrar en metros.")
        return
    alto, ancho = mapa_arr.shape
    res = cfg['resolution']
    origen = cfg['origin']
    origen_x, origen_y = origen[0], origen[1]
    extent = [origen_x, origen_x + ancho * res, origen_y, origen_y + alto * res]

    x_min_px, x_max_px, y_min_px, y_max_px = zoom_limits_px
    x_min_m = origen_x + x_min_px * res
    x_max_m = origen_x + x_max_px * res
    
    y_min_m = origen_y + y_min_px * res
    y_max_m = origen_y + y_max_px * res

    plt.figure(figsize=FIGSIZE)
    plt.imshow(mapa_arr, cmap='gray', extent=extent, origin='lower')
    plt.xlim(x_min_m, x_max_m)
    plt.ylim(y_min_m, y_max_m)

    if ruta_mundos:
        xs = [p[0] for p in ruta_mundos]
        ys = [p[1] for p in ruta_mundos]
        plt.plot(xs, ys, color='red', linewidth=2, label='Ruta A* (m)')
        plt.scatter([xs[0]],[ys[0]], color='lime', s=80, label='Inicio (m)')
        plt.scatter([xs[-1]],[ys[-1]], color='cyan', s=80, label='Fin (m)')
        plt.legend()

    plt.title("Resultado en metros (misma vista)")
    plt.xlabel("X (m)"); plt.ylabel("Y (m)")
    plt.grid(True)
    plt.show()

# ---------------- Main (flujo idéntico, con flip opcional) ----------------
def main():
    # 1) cargar mapa (opcionalmente volteado)
    # RUTA_PGM ya es absoluta gracias al bloque de código ROS inicial
    mapa, cuadricula = cargar_mapa_pgm(RUTA_PGM, umbral_libre=UMBRAL_LIBRE, flip_y=FLIP_MAP_Y)

    # 2) dilatar solo paredes
    cuadricula_dilatada = dilatar_paredes(cuadricula, RADIO_DILATACION)

    # 3) crear vista (paredes dilatadas en negro) y selección
    mapa_visual = mapa.copy()
    mapa_visual[cuadricula_dilatada == 1] = 0

    puntos, zoom_limits_px = mostrar_mapa_y_seleccionar(mapa_visual)
    
    if len(puntos) < 2:
        print("No se seleccionaron suficientes puntos.")
        return

    # Los puntos del mouse vienen en formato (columna, fila) - Matplotlib
    # Se convierten a formato (fila, columna) - NumPy para el algoritmo A*
    inicio = (int(puntos[0][1]), int(puntos[0][0]))
    fin    = (int(puntos[1][1]), int(puntos[1][0]))
    print(f"Punto inicio (fila, col): {inicio}")
    print(f"Punto fin (fila, col): {fin}")

    # 4) ejecutar A*
    buscador = BusquedaAEstrella(cuadricula_dilatada) 
    camino = buscador.buscar(inicio, fin) 

    # Manejo de errores de A*
    if isinstance(camino, str):
        if camino == "ERROR_INICIO_INVALIDO":
            print("ERROR: Punto de inicio inválido (cae sobre un obstáculo/dilatación).")
        elif camino == "ERROR_FIN_INVALIDO":
            print("ERROR: Punto de fin inválido (cae sobre un obstáculo/dilatación).")
        elif camino == "ERROR_RUTA_NO_ALCANZABLE":
            print("ERROR: Ruta no alcanzable (el destino está aislado del inicio).")
        else:
            print(f"ERROR: No se encontró un camino. Código de error: {camino}")
        return

    # Si camino es una lista (éxito)
    print(f"Camino encontrado con {len(camino)} pasos (píxeles).")
    mostrar_resultado(mapa_visual, camino, zoom_limits_px) 

    # 5) cargar YAML solo ahora y convertir la ruta a metros
    # RUTA_YAML ya es absoluta gracias al bloque de código ROS inicial
    cfg = cargar_config_mapa_yaml(RUTA_YAML)
    if cfg is None:
        print("No se realizó conversión a metros (falta YAML o es inválido).")
        return
    if 'resolution' not in cfg or 'origin' not in cfg:
        print("YAML no tiene 'resolution' u 'origin' -> no se puede convertir a metros.")
        return

    # decidir si invertimos Y al convertir según override o flip del mapa
    if FLIP_Y_OVERRIDE is None:
        flip_y_effective = FLIP_MAP_Y
    else:
        flip_y_effective = bool(FLIP_Y_OVERRIDE)

    alto_mapa_pixels = mapa.shape[0]
    resolucion = cfg['resolution']
    origen_x = cfg['origin'][0]
    origen_y = cfg['origin'][1]

    # Conversión a ruta en metros
    ruta_mundos = [
        pixel_a_mundo(r, c, resolucion, origen_x, origen_y, alto_mapa_pixels, flip_y_effective)
        for (r,c) in camino
    ]
    
    # 5.1) Guardar la ruta en metros en CSV
    # RUTA_CSV_SALIDA ya es absoluta y su directorio existe
    guardar_ruta_a_csv(ruta_mundos, RUTA_CSV_SALIDA)
    
    # 6) mostrar la ruta en metros con la misma vista convertida
    mostrar_resultado_metros(mapa, ruta_mundos, cfg, zoom_limits_px)

if __name__ == "__main__":
    main()
