# --- configuracion.py (Estructura Ajustada a sus Puntos) ---

import numpy as np

# --- 1. CONFIGURACIÓN GENERAL --- #
RADIO_ESFERA = 1.0 # radio de todas las esferas que componen el robot (cm)
T_BASE_GLOBAL = np.array([0, 0, 0]) # indica la posición de la base del robot en el marco global

# Fijamos los ángulos de prueba 
ANGULO_FIJO_ESLABON_0 = 0
ANGULO_FIJO_ESLABON_1 = 0 
ANGULO_FIJO_ESLABON_2 = 0 
ANGULO_FIJO_ESLABON_3 = 0
ANGULO_FIJO_ESLABON_4 = 0 

# Lista de 5 angulos entre 0 y 180 para realizar un barrido/sampling
ANGULOS_E0 = np.linspace(0, 180, 5) 
ANGULOS_E1 = np.linspace(0, 180, 5) 
ANGULOS_E2 = np.linspace(0, 180, 5) 
ANGULOS_E3 = np.linspace(0, 180, 5) 

# Lista que indica el minimo angulo que puede tomar cada servo
min_angulos = np.array([0, 0, 0, -1,0])
# Lista que indica el maximo angulo que puede tomar cada servo
max_angulos = np.array([170, 135, 170, -1, 170])

# --- 2. VECTORES DE DESPLAZAMIENTO (FIN DEL ESLABÓN EN EL MARCO LOCAL) --- #

# Los vectores V_i definen la posición del punto final de cada eslabón i en su marco local.
# Las constantes L_i son las longitudes de cada eslabón (norma de V_i).

V_0 = np.array([0.0, 0.0, 3.7])
L_0 = np.linalg.norm(V_0) 

V_1 = np.array([0.0, 2.3, 1.5])
L_1 = np.linalg.norm(V_1) 

V_2 = np.array([6.9, 0.0, 0.2])
L_2 = np.linalg.norm(V_2) 

V_3 = np.array([5.7, 3.0, 0.0])
L_3 = np.linalg.norm(V_3) 


V_4 = np.array([2.2, 2.2, 0.0])
L_4 = np.linalg.norm(V_4) 

# --- 3. CREACIÓN DE ESFERAS LOCALES (Modular) ---

# Lista de arrays que contiene las esferas de cada eslabón (posicion local)
esferas_local = []
VECTORES_DE_MOVIMIENTO = [V_0, V_1, V_2, V_3, V_4]

for k, V_fin in enumerate(VECTORES_DE_MOVIMIENTO):
    # Punto de inicio (Junta)
    P_inicio = np.array([0.0, 0.0, 0.0])
    
    # Punto final (Nueva junta)
    P_fin = V_fin
    
    # Punto intermedio (Usamos el punto medio para asegurar cobertura, si es necesario)
    P_medio = V_fin / 2.0 
    
    # Creamos un array que contiene las tres esferas: Inicio, Medio, Fin
    # NOTA: Puede ajustar el número de puntos según la longitud del eslabón (L_k)
    
    # Si la longitud es pequeña, usamos solo dos esferas (Inicio y Fin)
    if np.linalg.norm(V_fin) < 4 * RADIO_ESFERA:
         esf_array = np.array([P_inicio, P_fin])   
    # Si la longitud es grande, usamos tres esferas (Inicio, Medio, Fin)
    else:
         esf_array = np.array([P_inicio, P_medio, P_fin])

    esferas_local.append(esf_array)


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