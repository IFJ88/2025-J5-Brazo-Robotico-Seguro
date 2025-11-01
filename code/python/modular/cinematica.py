# cinematica.py

import numpy as np
from configuracion import T_BASE_GLOBAL
from algebra_lineal import matriz_rotacion_z, matriz_traslacion, aplicar_transformada

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


# --- Lógica Cinemática Principal ---

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
    
    # Lista para almacenar las coordenadas globales de cada eslabón
    esferas_globales = []
    
    # Iteramos sobre todos los eslabones (las juntas)
    for i, angulo in enumerate(angulos_grados):
        
        # 1. Rotación Local
        # La rotación R_local se aplica a las coordenadas del eslabón, 
        # que están definidas en su marco local (0,0,0)
        M_rot_local = matriz_rotacion_z(angulo)
        
        # 2. Traslación a la Junta Anterior (calculando T_junta)
        if i == 0:
            # Eslabón 1: La junta está en T_BASE_GLOBAL (0,0,0)
            M_tras_junta = matriz_traslacion(T_BASE_GLOBAL[0], T_BASE_GLOBAL[1], T_BASE_GLOBAL[2])
            
        else:
            # Eslabones subsiguientes: La junta anterior es la última esfera del eslabón anterior
            # P_union_anterior es la última esfera del eslabón i-1
            P_union_anterior = esferas_globales[-1][-1]
            
            # Traslación a esta posición global de la junta
            M_tras_junta = matriz_traslacion(P_union_anterior[0], P_union_anterior[1], P_union_anterior[2])
            
        # 3. Transformada Global
        # M_actual = T_junta * R_local * T_eslabon_base_local(ya implícita en la definición local)
        # T_junta lleva el origen local (0,0,0) a la posición global de la junta.
        # R_local rota el eslabón *después* de haber sido colocado en la junta.
        M_actual = M_tras_junta @ M_rot_local
        
        
        # 4. Aplicar Transformada a las Esferas Locales del Eslabón i
        esferas_global_i = np.array([
            aplicar_transformada(M_actual, esfera_local) 
            for esfera_local in esferas_locales[i]
        ])
        
        # Guardar la configuración global del eslabón actual
        esferas_globales.append(esferas_global_i)
        
    # 5. Detección de Colisión Total
    colision, centros_colisionantes = detectar_colision_total_modular(esferas_globales, radio)
    
    print(f"Número de eslabones procesados: {len(esferas_globales)}")
    
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