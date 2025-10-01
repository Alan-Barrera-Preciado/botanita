#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
from PIL import Image, ImageDraw
import numpy as np

import yaml
import os
import matplotlib.pyplot as plt

# --- Clase para cada Nodo de la cuadrícula ---
class Nodo:
    """
    Representa un nodo en la cuadrícula de búsqueda.
    Cada nodo tiene una posición, un padre (para reconstruir el camino),
    y los valores g, h, f utilizados por el algoritmo A*.
    """
    def __init__(self, posicion, padre=None, costo=0):
        self.posicion = posicion  # Una tupla (fila, columna)
        self.padre = padre
        self.costo = costo

        self.g = 0  # Costo desde el inicio hasta este nodo
        self.h = 0  # Costo heurístico estimado hasta el final
        self.f = 0  # Costo total (g + h)

    def __eq__(self, otro):
        # Dos nodos son iguales si sus posiciones son iguales
        return self.posicion == otro.posicion

    def __repr__(self):
        # Representación en string para facilitar la depuración
        return f"Nodo({self.posicion}, {self.costo}, g={self.g}, h={self.h}, f={self.f})"
    
# --- Clase principal del Algoritmo A* ---
class BusquedaAEstrella:
    """
    Implementa el algoritmo A* para encontrar el camino más corto en una cuadrícula.
    """
    def __init__(self, cuadricula, tipo_distancia="manhattan"):
        self.cuadricula = cuadricula
        self.alto = len(cuadricula)
        self.ancho = len(cuadricula[0])

        self.tipo_distancia = tipo_distancia

    def buscar(self, inicio, fin):
        """
        Ejecuta el algoritmo A* y devuelve la ruta como una lista de posiciones.
        """
        nodo_inicio = Nodo(inicio)
        nodo_fin = Nodo(fin)

        lista_abierta = []
        lista_cerrada = []

        # 1. Inicialización
        lista_abierta.append(nodo_inicio)

        # 2. Bucle principal
        while lista_abierta:
            # a. Encontrar el nodo con el menor costo f en la lista abierta
            nodo_actual = min(lista_abierta, key=lambda nodo: nodo.f)

            # b. Mover el nodo actual de la lista abierta a la cerrada
            lista_abierta.remove(nodo_actual)
            lista_cerrada.append(nodo_actual)

            # c. Si encontramos el destino, reconstruimos el camino y terminamos
            if nodo_actual == nodo_fin:
                return self._reconstruir_camino(nodo_actual)

            # d. Explorar los vecinos
            vecinos = self._obtener_vecinos(nodo_actual, self.tipo_distancia)
            for vecino in vecinos:
                #print(vecino)
                # Ignorar si el vecino ya está en la lista cerrada
                #casilla = vecino.posicion[0:2]
                #costo = vecino[2]
                if vecino in lista_cerrada:
                    continue

                # Calcular el costo g tentativo
                g_tentativo = nodo_actual.g + vecino.costo  # Asumimos costo de 1 entre nodos adyacentes

                # Si el vecino es nuevo o encontramos un camino mejor
                if g_tentativo < vecino.g or vecino not in lista_abierta:
                    vecino.g = g_tentativo
                    vecino.h = self._heuristica(vecino.posicion, nodo_fin.posicion)
                    vecino.f = vecino.g + vecino.h
                    vecino.padre = nodo_actual

                    if vecino not in lista_abierta:
                        lista_abierta.append(vecino)
        
        # Si la lista abierta se vacía y no encontramos el destino, no hay camino
        return None

    def _obtener_vecinos(self, nodo, tipo_distancia):
        
        if tipo_distancia == "manhattan":
            vecinos = [(0, 1,1), (0, -1,1), (1, 0,1), (-1, 0,1)] # (x,y,costo)
        elif tipo_distancia == "euclidiana":
            vecinos = [(0, 1,1), (1, 0,1), (0, -1,1), (-1, 0,1), (1, 1,1.41), (-1, -1,1.41), (1, -1,1.41), (-1, 1,1.41)]

        """
        Obtiene los nodos vecinos transitables (no obstáculos).
        """
        posiciones_vecinas = []
        # Movimientos en 4 direcciones (arriba, abajo, izquierda, derecha)
        for dr, dc, costo in vecinos:
            r, c = nodo.posicion[0] + dr, nodo.posicion[1] + dc

            # Verificar si la posición está dentro de la cuadrícula
            if 0 <= r < self.alto and 0 <= c < self.ancho:
                # Verificar si no es un obstáculo (asumimos que 1 es obstáculo)
                if self.cuadricula[r][c] == 0:
                    posiciones_vecinas.append(Nodo((r, c), costo=costo))
        
        return posiciones_vecinas

    def _heuristica(self, pos_a, pos_b):
        """
        Calcula la distancia de Manhattan como heurística.
        """
        
        if self.tipo_distancia == "manhattan":
            distancia = abs(pos_a[0] - pos_b[0]) + abs(pos_a[1] - pos_b[1])
        elif self.tipo_distancia == "euclidiana":
            distancia = math.sqrt((pos_a[0] - pos_b[0]) ** 2 + (pos_a[1] - pos_b[1]) ** 2)
            
        return distancia

    def _reconstruir_camino(self, nodo_final):
        """
        Reconstruye el camino desde el nodo final hasta el inicio
        siguiendo los punteros 'padre'.
        """
        camino = []
        actual = nodo_final
        while actual is not None:
            camino.append(actual.posicion)
            actual = actual.padre
        return camino[::-1]  # Devolver el camino en orden de inicio a fin

def cargar_y_procesar_mapa_pgm(ruta_archivo, umbral_obstaculo=240):
    """
    Carga un mapa desde un archivo PGM y lo convierte a una cuadrícula binaria.

    Args:
        ruta_archivo (str): La ruta al archivo .pgm.
        umbral_obstaculo (int): El valor de píxel por debajo del cual se 
                                considera un obstáculo. En los mapas de ROS,
                                los obstáculos son cercanos a 0 y el espacio
                                libre es cercano a 254/255.

    Returns:
        list: Una lista de listas (cuadrícula) donde 0 es transitable y 1 es un obstáculo.
    """
    try:
        # 1. Abrir la imagen PGM con Pillow
        with Image.open(ruta_archivo) as img:
            # Convertir la imagen a un array de NumPy para un procesamiento numérico eficiente
            mapa_array = np.array(img)

            # 2. Aplicar el umbral para crear la cuadrícula binaria
            # Asumimos que los valores por debajo del umbral son obstáculos (1)
            # y los valores por encima son transitables (0).
            # Esto es clave: debes ajustar el umbral según cómo tu Lidar genera el mapa.
            # Por ejemplo, si los obstáculos son < 50, el espacio libre es 255.
            cuadricula = np.where(mapa_array < umbral_obstaculo, 1, 0).tolist()
            
            print(f"Mapa '{ruta_archivo}' cargado exitosamente.")
            print(f"Dimensiones del mapa: {len(cuadricula)}x{len(cuadricula[0])} píxeles.")
            
            return cuadricula

    except FileNotFoundError:
        print(f"Error: No se encontró el archivo en la ruta '{ruta_archivo}'")
        return None
    except Exception as e:
        print(f"Ocurrió un error al procesar el archivo de imagen: {e}")
        return None


def cargar_config_mapa_yaml(ruta_yaml):
    """
    Carga el archivo de configuración del mapa .yml.

    Returns:
        Un diccionario con la configuración y la ruta completa a la imagen pgm.
    """
    try:
        with open(ruta_yaml, 'r') as f:
            config = yaml.safe_load(f)
            
            # Construye la ruta completa al archivo PGM, asumiendo que está
            # en el mismo directorio que el archivo YAML.
            dir_base = os.path.dirname(ruta_yaml)
            ruta_imagen = os.path.join(dir_base, config['image'])
            config['image_path'] = ruta_imagen
            
            return config
    except FileNotFoundError:
        print(f"Error: No se encontró el archivo YAML en '{ruta_yaml}'")
        return None
    except Exception as e:
        print(f"Error al leer el archivo YAML: {e}")
        return None
    
def mundo_a_pixel(x_mundo, y_mundo, resolucion, origen_x, origen_y, alto_mapa):
    """
    Convierte coordenadas del mundo real (metros) a coordenadas de píxel.
    """
    # Convertir a píxeles desde el origen
    pixel_x = int((x_mundo - origen_x) / resolucion)
    pixel_y_desde_origen = int((y_mundo - origen_y) / resolucion)

    # La coordenada 'y' en las imágenes se cuenta desde arriba hacia abajo,
    # mientras que en los mapas del mundo se cuenta desde abajo hacia arriba.
    # Por lo tanto, debemos invertirla.
    pixel_y = alto_mapa - 1 - pixel_y_desde_origen
    
    return (pixel_y, pixel_x) # Devuelve en formato (fila, columna)

def visualizar_con_matplotlib_real(mapa_binario, camino_pixels, config_mapa):
    """
    Crea una visualización con Matplotlib usando coordenadas del mundo real en los ejes.
    """
    resolucion = config_mapa['resolution']
    origen = config_mapa['origin']
    alto, ancho = len(mapa_binario), len(mapa_binario[0])

    # Definimos la extensión del mapa en metros para los ejes de la gráfica
    extent = [
        origen[0], origen[0] + ancho * resolucion,
        origen[1], origen[1] + alto * resolucion
    ]

    plt.imshow(mapa_binario, cmap='gray_r', extent=extent, origin='lower')
    
    # Convertimos el camino de píxeles de vuelta a coordenadas del mundo para graficarlo
    camino_mundo_x = [origen[0] + p[1] * resolucion for p in camino_pixels]
    camino_mundo_y = [origen[1] + (alto - 1 - p[0]) * resolucion for p in camino_pixels]

    plt.plot(camino_mundo_x, camino_mundo_y, color='red', linewidth=2, label='Ruta A*')

    # Marcamos inicio y fin
    plt.scatter(camino_mundo_x[0], camino_mundo_y[0], color='lime', s=100, zorder=5, label='Inicio')
    plt.scatter(camino_mundo_x[-1], camino_mundo_y[-1], color='cyan', s=100, zorder=5, label='Fin')
    
    plt.xlabel("Coordenada X (metros)")
    plt.ylabel("Coordenada Y (metros)")
    plt.title('Ruta Encontrada en el Mapa')
    plt.legend()
    plt.axis('equal') # Asegura que la escala en x e y sea la misma
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    # 1. Cargar la configuración del mapa desde el archivo YAML
    ruta_del_yaml = 'mapa_fer.yaml'
    config = cargar_config_mapa_yaml(ruta_del_yaml)

    if config:
        # 2. Cargar el mapa PGM usando la ruta del archivo de configuración
        mapa = cargar_y_procesar_mapa_pgm(config['image_path'])

        if mapa:
            alto_mapa_pixels = len(mapa)
            
            # 3. Definir inicio y fin en COORDENADAS DEL MUNDO (metros)
            inicio_mundo = (0, 0)  # (x, y) en metros
            fin_mundo = (4.82, -2.8)     # (x, y) en metros

            # 4. Convertir coordenadas del mundo a píxeles
            inicio_pixel = mundo_a_pixel(
                inicio_mundo[0], inicio_mundo[1],
                config['resolution'], config['origin'][0], config['origin'][1],
                alto_mapa_pixels
            )
            fin_pixel = mundo_a_pixel(
                fin_mundo[0], fin_mundo[1],
                config['resolution'], config['origin'][0], config['origin'][1],
                alto_mapa_pixels
            )
            
            print(f"Buscando ruta desde {inicio_mundo}m [pixel {inicio_pixel}] hasta {fin_mundo}m [pixel {fin_pixel}]")

            # 5. Ejecutar A* con las coordenadas de píxeles
            buscador = BusquedaAEstrella(mapa, tipo_distancia="euclidiana") # euclidiana | manhattan
            camino_en_pixeles = buscador.buscar(inicio_pixel, fin_pixel)

            # 6. Graficar el resultado
            if camino_en_pixeles:
                print(f"Camino encontrado con {len(camino_en_pixeles)} pasos.")
                visualizar_con_matplotlib_real(mapa, camino_en_pixeles, config)
            else:
                print("No se encontró un camino.")