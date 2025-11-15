# simulacion.py

import numpy as np
import sys # Importar sys para salir del script

# Importar las funciones y constantes de los módulos
from configuracion import RADIO_ESFERA, esferas_local, ANGULO_FIJO_ESLABON_0, ANGULO_FIJO_ESLABON_1, ANGULO_FIJO_ESLABON_2, ANGULO_FIJO_ESLABON_3, ANGULO_FIJO_ESLABON_4, MODO_DINAMICO, min_angulos, max_angulos
from cinematica import calcular_configuracion_modular, validador_limites_fisico
from visualizacion import (
    inicializar_visualizacion,
    actualizar_visualizacion_dinamica,
    finalizar_visualizacion,
    visualizar_plot_unico  # <--- Función para modo estático
)
##from intermediario import mapeo_angulo_esp32

# Definición de la función interactiva
def simulacion_manual(N_eslabones, angulos_actuales, esferas_base):
    """Bucle principal de la simulación manual e interactiva."""

    # Creamos un array mutable para los ángulos
    angulos_actuales = angulos_actuales.copy() 

    while True:
        try:
            print("\n" + "="*50)
            print("🤖 **SIMULACIÓN INTERACTIVA**")
            print("-" * 50)
            print(f"Ángulos actuales: {angulos_actuales.round(2)}°")
            print("Seleccione una opción:")
            print("  [0-4] : Eslabón (Junta) a rotar (0=Base, 4=Extremo)")
            print("  [e/E] : Salir")
            print("-" * 50)

            # 1. Solicitar el eslabón
            entrada_eslabon = input("Ingrese el índice del eslabón [0-4] o 'e' para salir: ").strip().lower()

            if entrada_eslabon == 'e':
                print("Saliendo de la simulación interactiva...")
                break

            eslabon_idx = int(entrada_eslabon)
            
            if not (0 <= eslabon_idx < N_eslabones):
                print(f"⚠️ Error: El índice de eslabón debe estar entre 0 y {N_eslabones - 1}.")
                continue

            # 2. Solicitar el nuevo ángulo
            print(f"\nJunta seleccionada: Eslabón {eslabon_idx} (Límites: {min_angulos[eslabon_idx]}° a {max_angulos[eslabon_idx]}°)")
            
            ## DEBE SER LEIDO POR EL SERIAL PORT QUE OBTENGA DE LA ESP32
            angulo_leido = float(input(f"Ingrese el nuevo ángulo para el Eslabón {eslabon_idx}: "))

            ##angulo_nuevo = mapeo_angulo_esp32(eslabon_idx, angulo_leido)

            angulo_nuevo=angulo_leido ## LUEGO SE BORRA ESTA LINEA
            
            print(f"angulo_nuevo:{angulo_nuevo}")
            
            # 3. Validar límites físicos
            if validador_limites_fisico(angulo_nuevo, min_angulos[eslabon_idx], max_angulos[eslabon_idx]):
                print("❌ Operación cancelada: Ángulo fuera de los límites físicos.")
                continue            

            # 4. Aplicar el ángulo y calcular la nueva configuración
            angulos_actuales[eslabon_idx] = angulo_nuevo # Modificar el ángulo en el array

            print("\n... Calculando cinemática y colisiones ...")
            esf_rotado, colision, centros_colisionantes = calcular_configuracion_modular(
                angulos_actuales, esferas_local, RADIO_ESFERA)

            # 5. Mostrar resultados y actualizar visualización
            print(f"RESULTADO: Eslabón {eslabon_idx} en {angulo_nuevo}°")
            print(f"🚨 ¡COLISIÓN DETECTADA! 🚨" if colision else "✅ No hay colisión.")

            if MODO_DINAMICO:
                actualizar_visualizacion_dinamica(
                    esf_rotado, angulos_actuales, centros_colisionantes, eslabon_actual=eslabon_idx)
            else:
                visualizar_plot_unico(
                    esf_rotado, angulos_actuales, centros_colisionantes, esferas_base, eslabon_actual=eslabon_idx)

        except ValueError:
            print("⚠️ Entrada inválida. Por favor, ingrese un número entero para el eslabón o 'e' para salir, y un número para el ángulo.")
        except IndexError:
            print("⚠️ Error de índice. Asegúrese de que la entrada sea válida.")
        except Exception as e:
            print(f"Ha ocurrido un error inesperado: {e}")
            break

# ----------------------------------------------------------------------

def main():

    N_eslabones = len(esferas_local)

    # --- FASE 0: Cálculo e Inicialización ---
    print("--- FASE 0: Calculando e Inicializando el Gráfico Base ---")
    
    # --> Pose de inicio (ángulos fijos).
    angulos_base = np.array([ANGULO_FIJO_ESLABON_0, ANGULO_FIJO_ESLABON_1,
                             ANGULO_FIJO_ESLABON_2, ANGULO_FIJO_ESLABON_3, ANGULO_FIJO_ESLABON_4])
    
    # --> Se lleva el brazo a la posición base, y se verifica si ya en la posicion base hay colisión.
    esferas_base, colision_base, centros_colisionantes = calcular_configuracion_modular(
        angulos_base, esferas_local, RADIO_ESFERA)
    
    # Comentamos la impresión de esferas locales para no llenar la consola
    # print("Esferas locales")
    # print(esferas_local) 

    # if colision_base:
    #     print(
    #         "¡Error Crítico! Colisión detectada en la posición base. Terminando simulación.")
    #     if MODO_DINAMICO:
    #         # Asegurar que se cierre la visualización si hay un error crítico
    #         finalizar_visualizacion() 
    #     sys.exit() # Salir del script
    # else:
    #     print("Base OK.")

    # Modo de visualizacion
    if MODO_DINAMICO:
        print("Modo de visualización: DINÁMICO (animación en un solo plot).")
        # Inicializar el plot una sola vez
        inicializar_visualizacion(N_eslabones, esferas_base)
        actualizar_visualizacion_dinamica(
            esferas_base, angulos_base, centros_colisionantes, eslabon_actual=0)
    else:
        print("Modo de visualización: PLOT ÚNICO (una ventana por paso).")
        visualizar_plot_unico(esferas_base, angulos_base,
                              centros_colisionantes, esferas_base, eslabon_actual=0)

    
    # ---------------------------------------------------------------------------------
    # --- REEMPLAZO DE BARRIDO SECUENCIAL POR SIMULACIÓN INTERACTIVA ---
    # ---------------------------------------------------------------------------------
    
    # Llamamos al nuevo bucle interactivo
    simulacion_manual(N_eslabones, angulos_base, esferas_base)
    

    # --- Finalización (Solo se ejecuta después de salir del bucle while) ---
    if MODO_DINAMICO:
        finalizar_visualizacion()


if __name__ == "__main__":
    main()