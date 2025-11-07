# --- configuracion.py (Estructura Ajustada a sus Puntos) ---

import numpy as np

# --- 1. CONFIGURACIÓN GENERAL ---
RADIO_ESFERA = 1.0 # Umbral de colisión 2.0
T_BASE_GLOBAL = np.array([0, 0, 0])

# Fijamos los ángulos de prueba (mantener el formato)
ANGULO_FIJO_ESLABON_0 = 0
ANGULO_FIJO_ESLABON_1 = 0 
ANGULO_FIJO_ESLABON_2 = 0 
ANGULO_FIJO_ESLABON_3 = 0
ANGULO_FIJO_ESLABON_4 = 0 

ANGULOS_E0 = np.linspace(0, 180, 5) # 5 pasos por defecto
ANGULOS_E1 = np.linspace(0, 180, 5) # 5 pasos por defecto
ANGULOS_E2 = np.linspace(0, 180, 5) # 5 pasos por defecto
ANGULOS_E3 = np.linspace(0, 180, 5) # 5 pasos por defecto
ANGULOS_E4 = np.linspace(0, 180, 5) # 5 pasos por defecto

min_angulos = np.array([-1, 0, 0, -1,0])
max_angulos = np.array([-1, 135, 170, -1, 170])

# --- 2. VECTORES DE DESPLAZAMIENTO (FIN DEL ESLABÓN EN EL MARCO LOCAL) ---

# Los eslabones se definen por dos puntos (inicio=junta, fin=longitud)
# Usaremos tres esferas para representar el cuerpo: Inicio, Medio, Fin.

# [0] Eslabón 0: Desde (0,0,0) hasta (0,0, 3.7) [Vector Z]
V_0 = np.array([3.7, 0.0, 0.0])
L_0 = np.linalg.norm(V_0) # Longitud: 3.7 cm

# [1] Eslabón 1: Desde junta 0 hasta (0, 2.3, 1.5) [Vector YZ]
V_1 = np.array([1.5, 2.3, 0.0])
L_1 = np.linalg.norm(V_1) # Longitud: 2.75 cm

# [2] Eslabón 2: Desde junta 1 hasta (6.9, 0, 0.2) [Vector X]
V_2 = np.array([6.9, 0.0, 0.2])
L_2 = np.linalg.norm(V_2) # Longitud: 6.9 cm

# [3] Eslabón 3: Desde junta 2 hasta (5.7, 3.0, 0) [Vector XY]
V_3 = np.array([5.7, 3.0, 0.0])
L_3 = np.linalg.norm(V_3) # Longitud: 6.44 cm

# [4] Eslabón 4: Desde junta 3 hasta (2.2, 2.2, 0) [Vector XY]
V_4 = np.array([2.2, 2.2, 0.0])
L_4 = np.linalg.norm(V_4) # Longitud: 3.11 cm

# --- 3. CREACIÓN DE ESFERAS LOCALES (Modular) ---

# La matriz de esferas locales debe incluir: [Punto Inicial], [Punto Intermedio], [Punto Final (Junta)]
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

# --- Verificación Final (Opcional) ---
# for i, esl in enumerate(esferas_local):
#     print(f"Eslabón {i}: {len(esl)} esferas. Longitud aprox: {np.linalg.norm(esl[-1]):.2f}")

# --- Parámetros de Simulación ---
MODO_DINAMICO = True 