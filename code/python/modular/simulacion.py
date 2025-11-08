# simulacion.py

import numpy as np

# Importar las funciones y constantes de los módulos
from configuracion import RADIO_ESFERA, esferas_local, ANGULO_FIJO_ESLABON_0, ANGULO_FIJO_ESLABON_1, ANGULO_FIJO_ESLABON_2, ANGULO_FIJO_ESLABON_3, ANGULO_FIJO_ESLABON_4, MODO_DINAMICO, ANGULOS_E0, ANGULOS_E1, ANGULOS_E2, ANGULOS_E3, ANGULOS_E4,min_angulos,max_angulos
from cinematica import calcular_configuracion_modular, validador_limites_fisico
from visualizacion import (
    inicializar_visualizacion, 
    actualizar_visualizacion_dinamica, 
    finalizar_visualizacion, 
    visualizar_plot_unico # <--- Función para modo estático
)

# --------------------------------------------------------------------------
# --- SIMULACIÓN Y PRUEBA DE MOVIMIENTO (MAIN) ---
# --------------------------------------------------------------------------

def main():
    
    N_eslabones = len(esferas_local)

    # --- FASE 0: Cálculo e Inicialización ---
    print("--- FASE 0: Calculando e Inicializando el Gráfico Base ---")
    angulos_base = np.array([ANGULO_FIJO_ESLABON_0, ANGULO_FIJO_ESLABON_1, ANGULO_FIJO_ESLABON_2, ANGULO_FIJO_ESLABON_3, ANGULO_FIJO_ESLABON_4])
    esferas_base, colision_base, centros_colisionantes = calcular_configuracion_modular(angulos_base, esferas_local, RADIO_ESFERA)
    
    if colision_base:
        print("¡Error Crítico! Colisión detectada en la posición base. Terminando simulación.")
    else:
        print("Base OK.")
    
    # 🔑 LÓGICA CONDICIONAL DE INICIALIZACIÓN
    if MODO_DINAMICO:
        print("Modo de visualización: DINÁMICO (animación en un solo plot).")
        # Inicializar el plot una sola vez
        inicializar_visualizacion(N_eslabones, esferas_base)
        actualizar_visualizacion_dinamica(esferas_base, angulos_base, centros_colisionantes)
    else:
        print("Modo de visualización: PLOT ÚNICO (una ventana por paso).")
        visualizar_plot_unico(esferas_base, angulos_base, centros_colisionantes, esferas_base)
        

    # ... (FASE 0 - Rotación Secuencial del Eslabón 0) ...

    print("\n\n--- FASE 0: Rotación Secuencial del Eslabón 0 (0° a 180°) ---")
    resultados_colision_E0 = {}

    for angulo_0 in ANGULOS_E0:
        angulo_0 = round(angulo_0, 2)
        
        if (validador_limites_fisico( angulo_0, min_angulos[0], max_angulos[0])):
          print("Ángulo fuera de los límites físicos permitidos.")
          break
        
        angulos_prueba_e0 = np.array([angulo_0, ANGULO_FIJO_ESLABON_1, ANGULO_FIJO_ESLABON_2, ANGULO_FIJO_ESLABON_3, ANGULO_FIJO_ESLABON_4])

        esf_rotado, colision, centros_colisionantes = calcular_configuracion_modular(angulos_prueba_e0, esferas_local, RADIO_ESFERA)
        
        resultados_colision_E0[ angulo_0] = "SI" if colision else "NO"
        
        print(f"\nProbando Eslabón 1 en { angulo_0}°: Colisión = { resultados_colision_E0[ angulo_0]}") 
        
        # 🔑 LÓGICA CONDICIONAL DE ACTUALIZACIÓN
        if MODO_DINAMICO:
            actualizar_visualizacion_dinamica(esf_rotado, angulos_prueba_e0, centros_colisionantes)
        else:
            visualizar_plot_unico(esf_rotado, angulos_prueba_e0, centros_colisionantes, esferas_base)
    
    # ... (FASE 1 - Rotación Secuencial del Eslabón 1) ...

    print("\n\n--- FASE 1: Rotación Secuencial del Eslabón 1 (0° a 180°) ---")
    resultados_colision_E1 = {}

    for angulo_1 in ANGULOS_E1:
        angulo_1 = round(angulo_1, 2)
        
        if (validador_limites_fisico( angulo_1, min_angulos[1], max_angulos[1])):
          print("Ángulo fuera de los límites físicos permitidos.")
          break
        
        angulos_prueba_e1 = np.array([ANGULO_FIJO_ESLABON_0, angulo_1, ANGULO_FIJO_ESLABON_2, ANGULO_FIJO_ESLABON_3, ANGULO_FIJO_ESLABON_4])

        esf_rotado, colision, centros_colisionantes = calcular_configuracion_modular(angulos_prueba_e1, esferas_local, RADIO_ESFERA)
        
        resultados_colision_E1[ angulo_1] = "SI" if colision else "NO"
        
        print(f"\nProbando Eslabón 1 en { angulo_1}°: Colisión = {resultados_colision_E1[ angulo_1]}") 
        
        # 🔑 LÓGICA CONDICIONAL DE ACTUALIZACIÓN
        if MODO_DINAMICO:
            actualizar_visualizacion_dinamica(esf_rotado, angulos_prueba_e1, centros_colisionantes)
        else:
            visualizar_plot_unico(esf_rotado, angulos_prueba_e1, centros_colisionantes, esferas_base)


    # --- FASE 2: Rotación Secuencial del Eslabón 2 ---
    print("\n--- FASE 2: Rotación Secuencial del Eslabón 2 (0° a 180°) ---")
    resultados_colision_E2 = {}

    for angulo_2 in ANGULOS_E2:
        angulo_2 = round(angulo_2, 2)
        
        if (validador_limites_fisico(angulo_2, min_angulos[2], max_angulos[2])):
          print("Ángulo fuera de los límites físicos permitidos.")
          break
        
        angulos_prueba_e2 = np.array([ANGULO_FIJO_ESLABON_0, ANGULO_FIJO_ESLABON_1, angulo_2, ANGULO_FIJO_ESLABON_3, ANGULO_FIJO_ESLABON_4])
        
        esf_rotado, colision, centros_colisionantes = calcular_configuracion_modular(angulos_prueba_e2, esferas_local, RADIO_ESFERA)
        
        resultados_colision_E2[angulo_2] = "SI" if colision else "NO"
        
        print(f"\nProbando Eslabón 2 en {angulo_2}°: Colisión = {resultados_colision_E2[angulo_2]}")
        
        # 🔑 LÓGICA CONDICIONAL DE ACTUALIZACIÓN
        if MODO_DINAMICO:
            actualizar_visualizacion_dinamica(esf_rotado, angulos_prueba_e2, centros_colisionantes)
        else:
            visualizar_plot_unico(esf_rotado, angulos_prueba_e2, centros_colisionantes, esferas_base) # Usa esferas_base como referencia inicial

        # ... (impresión de centros colisionantes) ...

    # ... (FASE 3 - Rotación Secuencial del Eslabón 3) ...

    print("\n\n--- FASE 3: Rotación Secuencial del Eslabón 3 (0° a 180°) ---")
    resultados_colision_E3 = {}

    for angulo_3 in ANGULOS_E3:
        angulo_3 = round(angulo_3, 2)
        
        if (validador_limites_fisico(angulo_3, min_angulos[3], max_angulos[3])):
          print("Ángulo fuera de los límites físicos permitidos.")
          break
        
        angulos_prueba_e3 = np.array([ANGULO_FIJO_ESLABON_0, ANGULO_FIJO_ESLABON_1, ANGULO_FIJO_ESLABON_2, angulo_3, ANGULO_FIJO_ESLABON_4])

        esf_rotado, colision, centros_colisionantes = calcular_configuracion_modular(angulos_prueba_e3, esferas_local, RADIO_ESFERA)
        
        resultados_colision_E3[angulo_3] = "SI" if colision else "NO"
        
        print(f"\nProbando Eslabón 3 en {angulo_3}°: Colisión = {resultados_colision_E3[angulo_3]}") 
        
        # 🔑 LÓGICA CONDICIONAL DE ACTUALIZACIÓN
        if MODO_DINAMICO:
            actualizar_visualizacion_dinamica(esf_rotado, angulos_prueba_e3, centros_colisionantes)
        else:
            visualizar_plot_unico(esf_rotado, angulos_prueba_e3, centros_colisionantes, esferas_base)

    # ... (FASE 4 - Rotación Secuencial del Eslabón 4) ...

    print("\n\n--- FASE 4: Rotación Secuencial del Eslabón 4 (0° a 180°) ---")
    resultados_colision_E4 = {}

    for angulo_4 in ANGULOS_E4:
        angulo_4 = round(angulo_4, 2)
        
        if (validador_limites_fisico(angulo_4, min_angulos[4], max_angulos[4])):
          print("Ángulo fuera de los límites físicos permitidos.")
          break
        
        angulos_prueba_e4 = np.array([ANGULO_FIJO_ESLABON_0, ANGULO_FIJO_ESLABON_1, ANGULO_FIJO_ESLABON_2, ANGULO_FIJO_ESLABON_3, angulo_4])

        esf_rotado, colision, centros_colisionantes = calcular_configuracion_modular(angulos_prueba_e4, esferas_local, RADIO_ESFERA)
        
        resultados_colision_E4[angulo_4] = "SI" if colision else "NO"
        
        print(f"\nProbando Eslabón 4 en {angulo_4}°: Colisión = {resultados_colision_E4[angulo_4]}") 
        
        # 🔑 LÓGICA CONDICIONAL DE ACTUALIZACIÓN
        if MODO_DINAMICO:
            actualizar_visualizacion_dinamica(esf_rotado, angulos_prueba_e4, centros_colisionantes)
        else:
            visualizar_plot_unico(esf_rotado, angulos_prueba_e4, centros_colisionantes, esferas_base)

    # --- Finalización ---
    if MODO_DINAMICO:
        finalizar_visualizacion()

if __name__ == "__main__":
    main()