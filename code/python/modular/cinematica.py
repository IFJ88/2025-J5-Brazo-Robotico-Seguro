# cinematica.py

import numpy as np
from configuracion import T_BASE_GLOBAL, SALTO_ESLABONES, OFFSET_ESLABON_EN_JUNTA
from algebra_lineal import matriz_rotacion_x, matriz_rotacion_y, matriz_rotacion_z, matriz_traslacion, aplicar_transformada

# --- Lógica de Detección de Colisión ---

def verificar_distancia(centro_a, centro_b, radio, tol=1e-9):
    """Verifica si la distancia entre dos centros es menor o igual a 2*radio."""
    distancia = np.linalg.norm(centro_a - centro_b)
    # Colisión ocurre si distancia <= 2 * RADIO_ESFERA
    return distancia <= 2.0 * radio + tol

    
def detectar_colision_par(esf_a, esf_b, idx_ignore_1, idx_ignore_2, radio):
    """Detecta colisiones entre dos eslabones, ignorando la esfera de la junta si es contiguo."""
    
    centros_colisionantes = [] # Lista de tuplas (coordenadas)
    
    for i, centro_a in enumerate(esf_a):
        for j, centro_b in enumerate(esf_b):
            # Ignorar la esfera de la junta contigua (última de A, primera de B)
            if i == idx_ignore_1 and j == idx_ignore_2:
                continue
            
            if verificar_distancia(centro_a, centro_b, radio):
                # Si hay colisión, agrega AMBOS centros a la lista
                print(f"!!! Colisión detectada entre esfera {i+1} de A y esfera {j+1} de B !!!")
                print(f"   Centro A: {centro_a}")
                print(f"   Centro B: {centro_b}")
                print(f"   Distancia: {np.linalg.norm(centro_a - centro_b):.4f} < Umbral: {2.0 * radio:.4f}")
                centros_colisionantes.append(tuple(centro_a)) 
                centros_colisionantes.append(tuple(centro_b))
                
    return centros_colisionantes # Retorna la lista de centros (vacía o con coordenadas)

def detectar_colision_total_modular(esferas_globales, radio):
    """Detecta colisiones en toda la configuración del robot."""
    
    num_eslabones = len(esferas_globales)
    # Usamos un set de tuplas de coordenadas para almacenar centros únicos
    centros_colisionantes_global = set() 
    
    for i in range(num_eslabones):
        # Comparar el eslabón i con todos los eslabones posteriores j > i
        for j in range(i + 1, num_eslabones):
            
            esf_i = esferas_globales[i]
            esf_j = esferas_globales[j]
            
            # Es una junta contigua (E1-E2, E2-E3, etc.)
            if j == i + 1:
                # El origen del eslabón j es la junta con el final del eslabón i.
                # Ignoramos la última esfera del eslabón i y la primera del eslabón j.
                idx_ignore_1 = len(esf_i) - 1
                idx_ignore_2 = 0
            else:
                # No es una junta (E1-E3), chequeamos todos los pares
                idx_ignore_1 = -1
                idx_ignore_2 = -1
            
            print(f"Verificando colisión entre Eslabón A={i+1} y Eslabón B={j+1}...")
            centros_colisionantes_par = detectar_colision_par(esf_i, esf_j, idx_ignore_1, idx_ignore_2, radio)

            if centros_colisionantes_par:
                # Agrega todas las coordenadas colisionantes al set global
                for centro_tupla in centros_colisionantes_par:
                    centros_colisionantes_global.add(centro_tupla)

    # Devuelve el booleano global de colisión y el set de coordenadas.
    return bool(centros_colisionantes_global), centros_colisionantes_global


# --- Función principal: cinemática directa ---
# Entrada
# - angulos_grados:
# - esferas_locales:  (de configuracion.py)
# - radio: 
# Salida
# - esferas_globales: lista de posiciones 3d finales de las esferas
# - colision: booleano si hay colisión
# - centros_colisionantes: lista de esferas que chocan

def calcular_configuracion_modular(angulos_grados, esferas_locales, radio):
    """
    Calcula la posición global de las esferas para cada eslabón.
    
    Args:
        angulos_grados (np.array): Ángulos de rotación (junta) para cada eslabón.
        esferas_locales (list of np.array): Coordenadas locales de las esferas de cada eslabón.
        radio (float): Radio de las esferas.
        
    Returns:
        tuple: (esferas_globales, colision, centros_colisionantes)
    """
    
    # Rotaciones fijas previas para alinear el eje físico de cada servo con Z
    Rfix = [
        np.eye(4),                 # Servo 0: ya es Z
        matriz_rotacion_x(90),     # Servo 1: Y -> Z
        np.eye(4),                 # Servo 2: Y -> Z MANTIENE EL EJE DEL SERVO ANTERIOR DADO QUE ES EL MISMO
        matriz_rotacion_y(-90),    # Servo 3: X -> Z
        matriz_rotacion_y(90),     # Servo 4: Y -> Z
    ]

    # Lista para almacenar las coordenadas globales de cada eslabón
    esferas_globales = []

    # Inicializamos la matriz acumulada con la traslación de la base global
    M_acum = matriz_traslacion(T_BASE_GLOBAL[0], T_BASE_GLOBAL[1], T_BASE_GLOBAL[2])
    
    # Iteramos sobre todos los eslabones (las juntas)
    for i, angulo in enumerate(angulos_grados):
        

        # Matriz que transforma cualquier punto del eslabon i al marco global desde el punto del eslabón (después del offset)
        M_eslabon = M_acum @ Rfix[i] @ matriz_rotacion_z(angulo)
        # Obtenemos las posiciones globales de las esferas del eslabón i con M_link
        esf_global_i = np.array([
            aplicar_transformada(M_eslabon, p_local)
            for p_local in esferas_locales[i]
        ])
        esferas_globales.append(esf_global_i)
        print(f"valor de i {i}°")
        # Actualizamos M_acum para que represente la pose del FINAL de la junta i 
        vector_desplazamiento = SALTO_ESLABONES[i]
        # Le aplico la traslación a M_link para obtener el nuevo M_acum
        M_acum = M_eslabon @ matriz_traslacion(*vector_desplazamiento)


    # Calculamos todas las esferas globales, llamamos a la detección de colisiones
    colision, centros_colisionantes = detectar_colision_total_modular(esferas_globales, radio)
    return esferas_globales, colision, centros_colisionantes

def validador_limites_fisico(angulo, min_angulo, max_angulo):
    """
    Valida que los ángulos estén dentro de los límites físicos.
    
    Args:
        angulos_grados (np.array): angulo a validar.
        min_angulos (list): Mínimos permitidos para cada ángulo.
        max_angulos (list): Máximos permitidos para cada ángulo.
        
    Returns:
        bool: True si el angulo no esta dentro de los limites.
    """
    
    if min_angulo == -1 and max_angulo == -1:
        return False  # Sin límites definidos
    elif angulo < min_angulo or angulo > max_angulo:
        print(f"Ángulo fuera de límites físicos: {angulo}° (Límites: {min_angulo}° - {max_angulo}°)")
        return True
    return False