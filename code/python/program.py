import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- Configuración Inicial (AJUSTE CLAVE) ---
RADIO_ESFERA = 2.0
T_BASE_GLOBAL = np.array([0, 0, 0])

# AJUSTE CLAVE: Separación de centros > 2*R (2.0). Usamos 4.2 para margen de seguridad.
SEPARACION = 4.2

# Creamos una lista para almacenar los arrays de coordenadas locales
esferas_local = []

# Eslabón 1 (Índice 0): 3 esferas
esferas_local.append(np.array([[0.0, 0, 0], [SEPARACION, 0, 0], [2 * SEPARACION, 0, 0]])) 

# Eslabón 2 (Índice 1): 2 esferas
esferas_local.append(np.array([[SEPARACION, 0, 0]])) 

# Eslabón 3 (Índice 2): 2 esferas
esferas_local.append(np.array([[SEPARACION, 0, 0], [2*SEPARACION, 0, 0]]))

# Eslabón 3 (Índice 2): 2 esferas
esferas_local.append(np.array([[SEPARACION, 0, 0]]))

# --- Funciones de Álgebra Lineal (Sin cambios) ---
def matriz_rotacion_z(angulo_grados):
    angulo_rad = math.radians(angulo_grados)
    c = np.cos(angulo_rad)
    s = np.sin(angulo_rad)
    R = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    return R

def matriz_traslacion(tx, ty, tz):
    T = np.identity(4)
    T[0, 3] = tx
    T[1, 3] = ty
    T[2, 3] = tz
    return T

def aplicar_transformada(matriz_transformada, coordenada_local):
    p_local_homogeneo = np.append(coordenada_local, 1)
    p_global_homogeneo = matriz_transformada @ p_local_homogeneo
    return p_global_homogeneo[:3]

# --- Lógica de Detección de Colisión Refinada (Sin cambios) ---

def verificar_distancia(centro_a, centro_b, radio, tol=1e-9):
    distancia = np.linalg.norm(centro_a - centro_b)
    return distancia <= 2.0 * radio + tol

    
def detectar_colision_par(esf_a, esf_b, idx_ignore_1, idx_ignore_2, radio):
    
    # Inicializa una lista para almacenar las coordenadas de los centros colisionantes
    centros_colisionantes = []
    
    
    for i, centro_a in enumerate(esf_a):
        for j, centro_b in enumerate(esf_b):
            if i == idx_ignore_1 and j == idx_ignore_2:
                continue
            
            if verificar_distancia(centro_a, centro_b, radio):
                # Si hay colisión, agrega AMBOS centros a la lista
                # Usamos una tupla de tuplas para que pueda ser hasheada y agregada al set global
                print(f"!!! Colisión detectada entre esfera {i+1} de A y esfera {j+1} de B !!!")
                print(f"   Centro A: {centro_a}")
                print(f"   Centro B: {centro_b}")
                print(f"   Distancia: {np.linalg.norm(centro_a - centro_b):.4f} < Umbral: {SEPARACION}")
                centros_colisionantes.append(tuple(centro_a)) 
                centros_colisionantes.append(tuple(centro_b))
                # Continuamos el bucle para buscar MÁS colisiones dentro de este par de eslabones
                # No usamos 'return True' aquí
                
    return centros_colisionantes # Retorna la lista de centros (vacía o con coordenadas)

def detectar_colision_total_modular(esferas_globales, radio):
    
    num_eslabones = len(esferas_globales)
    centros_colisionantes_global = set() # Usamos un set para almacenar índices únicos
    
    for i in range(num_eslabones):
        for j in range(i + 1, num_eslabones):
            
            esf_i = esferas_globales[i]
            esf_j = esferas_globales[j]
            
            # Determinar si el par (i, j) es una junta contigua (ej., E1-E2, E2-E3)
            if j == i + 1:
                # Es una junta contigua, ignoramos el par de esferas de la unión
                idx_ignore_1 = len(esf_i) - 1
                idx_ignore_2 = 0
            else:
                # No es una junta (ej., E1-E3), chequeamos todos los pares
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

# --- Lógica Cinemática Principal (Sin cambios) ---

# La función ahora acepta una lista/array de ángulos + array de colisiones
def calcular_configuracion_modular(angulos_grados, esferas_locales, radio):
    
    # Lista para almacenar las coordenadas globales de cada eslabón
    esferas_globales = []
    
    # Inicializamos la matriz de transformación del eslabón anterior (BASE GLOBAL)
    # M_anterior comienza siendo la matriz identidad (o la transformada de la base global, si no fuera 0,0,0)
    M_anterior = np.identity(4) 
    
    # Iteramos sobre todos los eslabones
    for i, angulo in enumerate(angulos_grados):
        
        # 1. Definir las transformaciones locales (Rotación y Traslación)
        
        # Rotación local del eslabón actual
        M_rot_local = matriz_rotacion_z(angulo)
        
        # Si NO es el primer eslabón (i > 0), el origen local [0,0,0] está en el final del eslabón anterior.
        # Ya que M_anterior apunta a la junta, M_tras_junta es implícita en la multiplicación T * M_anterior.
        # Pero, como nuestra lógica simplificada usa T_junta @ R_local, necesitamos calcular P_union
        
        # En esta estructura modular, necesitamos la posición global de la junta anterior
        if i == 0:
            # Eslabón 1: Se traslada desde T_BASE_GLOBAL (0,0,0) y rota
            M_tras_junta = matriz_traslacion(T_BASE_GLOBAL[0], T_BASE_GLOBAL[1], T_BASE_GLOBAL[2])
            
            # M_actual = T_base @ R_local
            M_actual = M_tras_junta @ M_rot_local
            
        else:
            # Eslabones subsiguientes: La nueva junta es la última esfera del eslabón anterior
            P_union_anterior = esferas_globales[-1][-1]
            
            # 2. Traslación a la Junta Anterior
            M_tras_junta = matriz_traslacion(P_union_anterior[0], P_union_anterior[1], P_union_anterior[2])
            
            # M_actual = T_junta @ R_local
            M_actual = M_tras_junta @ M_rot_local
            
        
        # 3. Aplicar Transformada a las Esferas Locales del Eslabón i
        esferas_global_i = np.array([
            aplicar_transformada(M_actual, esfera_local) 
            for esfera_local in esferas_locales[i]
        ])
        
        # Guardar la configuración global del eslabón actual
        esferas_globales.append(esferas_global_i)
        
    # 4. Detección de Colisión Total
    # La función de colisión total también debe ser refactorizada para aceptar la lista
    colision, centros_colisionantes = detectar_colision_total_modular(esferas_globales, radio)
    
    # voy a imprimir el length de esferas_globales para saber cuantos eslabones hay
    print(f"Número de eslabones procesados: {len(esferas_globales)}")
    
    return esferas_globales, colision, centros_colisionantes

# --- Función de Visualización 3D (CORREGIDA) ---

def visualizar_simulacion_modular(esferas_globales_list, angulos_list, centros_colisionantes_global, esferas_iniciales_list=None):
    """
    Dibuja N eslabones utilizando listas de configuraciones globales.

    Args:
        esferas_globales_list (list of np.array): Lista de coordenadas globales actuales (N eslabones).
        angulos_list (list of float): Lista de ángulos de rotación de las N juntas.
        colision (bool): True si hay colisión, False si no.
        esferas_iniciales_list (list of np.array, optional): Configuración inicial para referencia.
    """
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(projection='3d')
    
    N = len(esferas_globales_list)
    COLORES_BASE = ['blue', 'green', 'yellow', 'cyan', 'magenta']
    
    # Lista de todas las coordenadas para el cálculo de límites
    all_coords = []

    # --- 1. Dibujar Posición Inicial (Gris) ---
    if esferas_iniciales_list is not None and len(esferas_iniciales_list) == N:
        for k in range(N):
            esf_init = esferas_iniciales_list[k]
            ax.scatter(esf_init[:, 0], esf_init[:, 1], esf_init[:, 2], 
                       c='gray', marker='o', s=100 * RADIO_ESFERA, alpha=0.3)
            all_coords.append(esf_init)
        
        # Añadir una leyenda genérica para la posición inicial (solo el primer elemento)
        ax.scatter([], [], [], c='gray', marker='o', s=100 * RADIO_ESFERA, alpha=0.3, label='Inicial')


    # --- 2. Dibujar Posición Actual (Iteración sobre N eslabones) ---
    for k in range(N):
        esf_global = esferas_globales_list[k]
        color_base_eslabon = COLORES_BASE[k % len(COLORES_BASE)]
        
        # 2a. Dibujar Líneas (Cuerpo del eslabón)
        # La línea siempre usa el color base del eslabón (no el color_final)
        
        # Determinamos si hay colisión global para el título
        colision_global = bool(centros_colisionantes_global)
        
        # La línea (cuerpo) mantiene el color base, independiente de la colisión
        for i in range(len(esf_global) - 1):
            p1, p2 = esf_global[i], esf_global[i+1]
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 
                    color=color_base_eslabon, # Usa siempre el color base
                    linestyle='-', linewidth=3)
        
        # 2b. Dibujar Esferas (Puntos)
        for centro_coord in esf_global:
            
            # Convertir el centro a tupla para poder compararlo con el set
            centro_tupla = tuple(centro_coord)
            
            # Lógica de color de la esfera individual
            if centro_tupla in centros_colisionantes_global:
                color_esfera = 'red' # Colisión: Rojo
            else:
                color_esfera = color_base_eslabon # No colisión: Color base del eslabón

            # Dibujar esfera individual
            ax.scatter(centro_coord[0], centro_coord[1], centro_coord[2], 
                       c=color_esfera, marker='o', s=200 * RADIO_ESFERA, 
                       label=None, alpha=0.4) # La leyenda ya no se genera por cada esfera

        all_coords.append(esf_global)

    # --- 3. Dibujar Juntas de Conexión (entre Eslabón k y Eslabón k+1) ---
    for k in range(N - 1):
        p_union_k = esferas_globales_list[k][-1]     # Última esfera del eslabón k
        p_origen_k1 = esferas_globales_list[k+1][0] # Primera esfera del eslabón k+1
        
        ax.plot([p_union_k[0], p_origen_k1[0]], [p_union_k[1], p_origen_k1[1]], [p_union_k[2], p_origen_k1[2]], 
                color='black', linestyle='--', linewidth=2, label=f'Junta {k+1}-{k+2}' if k == 0 else None)


    # --- 4. Configuración Final ---
    ax.scatter(T_BASE_GLOBAL[0], T_BASE_GLOBAL[1], T_BASE_GLOBAL[2], c='black', marker='X', s=200, label='Origen Global (0,0,0)')

    # Cálculo de límites adaptativos
    if all_coords:
        all_coords_stack = np.vstack(all_coords)
        max_abs_coord = np.max(np.abs(all_coords_stack))
        max_range = max_abs_coord + RADIO_ESFERA + 1
        
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([-max_range, max_range])
    
    # Generar cadena de ángulos para el título
    angulos_titulo = " | ".join([f'E{i+1}: {ang:.1f}°' for i, ang in enumerate(angulos_list)])
    
    # if len(eslabones_colisionantes) != 0 : colision = True
    # else: colision = False
    
    ax.set_title(f'Rotaciones: {angulos_titulo} | Colisión: {"SI" if colision_global else "NO"}')
    ax.legend()
    plt.show()

# --------------------------------------------------------------------------
# --- SIMULACIÓN Y PRUEBA DE MOVIMIENTO (CORREGIDA) ---
# --------------------------------------------------------------------------

# Suponemos que tienes la definición de ESFERAS_LOCALES y RADIO_ESFERA
# y la definición de calcular_configuracion_modular disponible.

# Fijamos la posición del Eslabón 1 
ANGULO_FIJO_ESLABON_1 = 0 
ANGULO_FIJO_ESLABON_3 = 90
ANGULO_FIJO_ESLABON_4 = 45 


print(f"--- FASE 1: Fijando Eslabón 1 a {ANGULO_FIJO_ESLABON_1}° y Eslabón 3 a {ANGULO_FIJO_ESLABON_3}° ---")

# 1. CONSTRUIR ARRAY DE ÁNGULOS BASE
angulos_base = np.array([ANGULO_FIJO_ESLABON_1, 0, ANGULO_FIJO_ESLABON_3, ANGULO_FIJO_ESLABON_4])

# Llamada CORREGIDA: Pasar el array de ángulos como primer argumento
esferas_base, colision_base, centros_colisionantes = calcular_configuracion_modular(angulos_base, esferas_local, RADIO_ESFERA)

if colision_base:
    print("¡Error Crítico! Colisión detectada en la posición base. Revise la separación de esferas locales.")
    exit()
else:
    print("Base OK. Eslabones en posición inicial NO tienen colisión. Se procede a rotar el Eslabón 2.")
    # La visualización sigue usando los valores individuales
    visualizar_simulacion_modular(esferas_base, angulos_base, centros_colisionantes, esferas_base)

# Rangos de rotación para el Eslabón 2 (de 0 a 180 grados, 5 pasos)
ANGULOS_A_PROBAR = np.linspace(0, 180, 5) 

print("\n--- FASE 2: Rotación Secuencial del Eslabón 2 (0° a 180°) ---")

resultados_colision_E2 = {} # Usar un diccionario dedicado para E2

for angulo_2 in ANGULOS_A_PROBAR:
    angulo_2 = round(angulo_2, 2)
    
    # 2. CONSTRUIR ARRAY DE ÁNGULOS PARA E2
    angulos_prueba_e2 = np.array([ANGULO_FIJO_ESLABON_1, angulo_2, ANGULO_FIJO_ESLABON_3, ANGULO_FIJO_ESLABON_4])
    
    # Llamada CORREGIDA: Pasar el array de ángulos
    esf_rotado, colision, centros_colisionantes = calcular_configuracion_modular(angulos_prueba_e2, esferas_local, RADIO_ESFERA)
    
    resultados_colision_E2[angulo_2] = "SI" if colision else "NO"
    
    print(f"\nProbando Eslabón 2 en {angulo_2}°: Colisión = {resultados_colision_E2[angulo_2]}")
    
    # Visualizar
    visualizar_simulacion_modular(esf_rotado, angulos_prueba_e2, centros_colisionantes, esf_rotado)
    
    print("----------------------------------------")
    print("Centros colisionantes detectados:")
    for centro in centros_colisionantes:
        print(centro)
    print

print("\n--- RESUMEN DE COLISIONES (Eslabón 2) ---")
for ang, col in resultados_colision_E2.items():
    print(f"Ángulo Esl. 2: {ang}° -> Colisión: {col}")

# --- NUEVA FASE: Rotación del Eslabón 3 ---
ANGULO_FIJO_ESLABON_2 = 0 

print("\n\n--- FASE 3: Rotación Secuencial del Eslabón 3 (0° a 180°, E2 fijo) ---")
resultados_colision_E3 = {}

for angulo_3 in ANGULOS_A_PROBAR:
    angulo_3 = round(angulo_3, 2)
    
    # 3. CONSTRUIR ARRAY DE ÁNGULOS PARA E3
    angulos_prueba_e3 = np.array([ANGULO_FIJO_ESLABON_1, ANGULO_FIJO_ESLABON_2, angulo_3, ANGULO_FIJO_ESLABON_4])

    # Calcular la nueva configuración (E1 fijo, E2 fijo)
    esf_rotado, colision_base, centros_colisionantes = calcular_configuracion_modular(angulos_prueba_e3, esferas_local, RADIO_ESFERA)
    
    resultados_colision_E3[angulo_3] = "SI" if colision else "NO"
    
    print(f"\nProbando Eslabón 3 en {angulo_3}°: Colisión = {resultados_colision_E3[angulo_3]}") 
    
    # Llamada CORREGIDA: utilizamos los arrays correspondientes
    visualizar_simulacion_modular(esf_rotado,angulos_prueba_e3,centros_colisionantes,esf_rotado)
    
    # visualizar_simulacion(esf_rotado[0], esf_rotado[1], esf_rotado[2], ANGULO_FIJO_ESLABON_1, ANGULO_FIJO_ESLABON_2, angulo_3, colision, esf_rotado[0], esf_rotado[1], esf_rotado[2])

print("\n--- RESUMEN DE COLISIONES (Eslabón 3) ---")
for ang, col in resultados_colision_E3.items():
    print(f"Ángulo Esl. 3: {ang}° -> Colisión: {col}")