# --- configuracion.py (Estructura Ajustada a sus Puntos) ---

import numpy as np
from algebra_lineal import matriz_traslacion

# --- 1. CONFIGURACIÓN GENERAL --- #
RADIO_ESFERA = 1.0 # radio de todas las esferas que componen el robot (cm)
T_BASE_GLOBAL = np.array([0, 0, 0]) # indica la posición de la base del robot en el marco global

# --- Matriz de Colisiones ---
# Los índices i, j corresponden al Eslabón i y Eslabón j (0-indexado).
# 1 -> Colisionan; 0 -> No colisionan
MATRIZ_COLISIONES = np.array([
    [ 0, 0, 0, 0, 1], # Eslabón 0
    [ 0, 0, 1, 0, 1], # Eslabón 1
    [ 0, 1, 0, 0, 0], # Eslabón 2
    [ 0, 0, 0, 0, 0], # Eslabón 3
    [ 1, 1, 0, 0, 0]  # Eslabón 4
])


# Fijamos los ángulos de prueba 
ANGULO_FIJO_ESLABON_0 = 0
ANGULO_FIJO_ESLABON_1 = 0 
ANGULO_FIJO_ESLABON_2 = 0 
ANGULO_FIJO_ESLABON_3 = 0
ANGULO_FIJO_ESLABON_4 = 0 

# Lista de 5 angulos entre 0 y 180 para realizar un barrido/sampling
# ANGULOS_E0 = np.linspace(0, 180, 5) 
# ANGULOS_E1 = np.linspace(0, -135, 5)
# ANGULOS_E2 = np.linspace(0, 180, 5) 
# ANGULOS_E3 = np.linspace(0, 180, 5) 
# ANGULOS_E4 = np.linspace(0, 180, 5)

# Lista que indica el minimo angulo que puede tomar cada servo
min_angulos = np.array([0, 0, -170, -1, 0])
# Lista que indica el maximo angulo que puede tomar cada servo
max_angulos = np.array([170, 135, 0, -1, 180])

# --- 2. VECTORES DE DESPLAZAMIENTO (FIN DEL ESLABÓN EN EL MARCO LOCAL) --- #

# Los vectores V_i definen la posición del punto final de cada eslabón i en su marco local.
# Las constantes L_i son las longitudes de cada eslabón (norma de V_i).

SALTO_0 = np.array([0.0, 0.0, 3])  # Salto del eslabón base

# SALTO_1 = np.array([2.3, 0.0, 1.5])  # Salto del eslabón 1
SALTO_1 = np.array([2.3, 0, 0.0])  # Salto del eslabón 1 (Y se vuelve -Z)

# SALTO_2 = np.array([6.9, 0.0, 0.0])  # Salto del eslabón 2
SALTO_2 = np.array([6.9, 0.0, 0.0])  # Salto del eslabón 2

# SALTO_3 = np.array([5.7, 3.0, 0.0])  # Salto del eslabón 3
SALTO_3 = np.array([0.0, 0.0, -3.0]) # (X se vuelve Z)

# SALTO_4 = np.array([2.2, 0, 0])  # Salto del eslabón 4 (herramienta)
SALTO_4 = np.array([0.0, 0.0, 2.0])  # (X se vuelve -Z)

SALTO_ESLABONES = [SALTO_0, SALTO_1, SALTO_2, SALTO_3, SALTO_4]
# --- 3. CREACIÓN DE ESFERAS LOCALES (Modular) ---

# Lista de arrays que contiene las esferas de cada eslabón (posicion local)
esf_0 = np.array([
    [0.0, 0.0, 0.0]  # Esfera en la base
])

# Eslabón 1: Solo una esfera en el "codo" (que no es el final)
esf_1 = np.array([
    [0.0, 0.0, 0.0],
    [2, 0, 0.0]
])

# Eslabón 2: Tres esferas (inicio, medio, fin)
esf_2 = np.array([
    [0.0, 0.0, 0.0],
    [3.0, 0.0, 0.0],
    [5.0, 0.0, 0.0]
])

# Eslabón 3: Sin esferas de colisión
esf_3 = np.array([[0.0, 0.0, 0.0]])

# Eslabón 4: Una esfera al final
esf_4 = np.array([
    [0, 0.0, 0.0],
    [0, -2.0, 0.0]
])

esferas_local = [esf_0, esf_1, esf_2, esf_3, esf_4]


# --- 4. Creacion de vectores OFFSET que llevan de la posicion del servo con la posición del eslabón ---

T_E0 = np.eye(4)  # Matriz identidad, sin offset
T_E1 = matriz_traslacion(0,2.3, 1.5)
T_E2 = np.eye(4)
T_E3 = matriz_traslacion(0, 2.3, 1.5)
T_E4 = matriz_traslacion(0, 2.2, 0.0)

OFFSET_ESLABON_EN_JUNTA = [T_E0, T_E1, T_E2, T_E3, T_E4]

# --- Parámetros de Simulación ---
MODO_DINAMICO = True

def set_view(ax, plane='xy'):
    if plane == 'xy':      # vista superior
        ax.view_init(elev=90, azim=-90)
    elif plane == 'yz':    # vista lateral (X hacia el observador)
        ax.view_init(elev=0, azim=90)
    elif plane == 'xz':    # vista frontal (Y hacia el observador)
        ax.view_init(elev=0, azim=0)
    elif plane == 'xz_rotada':
        ax.view_init(elev=0, azim=270)
    else:
        ax.view_init(elev=20, azim=-35)  # isométrica por defecto