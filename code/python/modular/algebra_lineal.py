# algebra_lineal.py

import numpy as np
import math

# --- Funciones de Álgebra Lineal ---
def matriz_rotacion_x(angulo_grados):
    """Rotación 4x4 alrededor del eje X."""
    angulo_rad = math.radians(angulo_grados)
    c, s = np.cos(angulo_rad), np.sin(angulo_rad)
    R = np.array([
        [1, 0,  0, 0],
        [0, c, -s, 0],
        [0, s,  c, 0],
        [0, 0,  0, 1]
    ])
    return R

def matriz_rotacion_y(angulo_grados):
    """Rotación 4x4 alrededor del eje Y."""
    angulo_rad = math.radians(angulo_grados)
    c, s = np.cos(angulo_rad), np.sin(angulo_rad)
    R = np.array([
        [ c, 0, s, 0],
        [ 0, 1, 0, 0],
        [-s, 0, c, 0],
        [ 0, 0, 0, 1]
    ])
    return R

def matriz_rotacion_z(angulo_grados):
    """Rotación 4x4 alrededor del eje Z."""
    angulo_rad = math.radians(angulo_grados)
    c, s = np.cos(angulo_rad), np.sin(angulo_rad)
    R = np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1]
    ])
    return R

# --> Matriz de traslación 4x4 que traslada un punto
def matriz_traslacion(tx, ty, tz):
    """Genera la matriz de traslación 4x4."""
    T = np.identity(4)
    T[0, 3] = tx
    T[1, 3] = ty
    T[2, 3] = tz
    return T

def aplicar_transformada(matriz_transformada, coordenada_local):
    """Aplica la matriz de transformación a una coordenada 3D local."""
    # Convertir a coordenadas homogéneas [x, y, z, 1]
    p_local_homogeneo = np.append(coordenada_local, 1) 
    p_global_homogeneo = matriz_transformada @ p_local_homogeneo
    # Devolver las coordenadas 3D globales [x, y, z]
    return p_global_homogeneo[:3]