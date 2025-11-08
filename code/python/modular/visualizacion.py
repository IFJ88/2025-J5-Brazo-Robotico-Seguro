# visualizacion.py

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Importar las constantes de configuración
from configuracion import RADIO_ESFERA, T_BASE_GLOBAL, set_view

# Diccionarios globales para almacenar referencias a los objetos gráficos (solo para modo dinámico)
_plot_references = {'scatters': [], 'lines': [], 'juntas': [], 'ax': None}

# ----------------------------------------------
# 🎯 MODO DINÁMICO (Funciones existentes)
# ----------------------------------------------

def inicializar_visualizacion(N_eslabones, esferas_iniciales_list=None):
    # ... (código para inicializar el gráfico dinámico, sin cambios) ...
    # ... (ax.view_init(elev=90, azim=-90) y plt.ion() están aquí) ...
    
    # [Mantener el código de inicializar_visualizacion como está]
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    COLORES_BASE = ['blue', 'green', 'yellow', 'cyan', 'magenta']
    
    _plot_references['scatters'] = []
    _plot_references['lines'] = []
    _plot_references['juntas'] = []
    _plot_references['ax'] = ax

    set_view(ax, 'xz_rotada')
    ax.dist = 8 
    
    all_coords = []
    
    if esferas_iniciales_list is not None and len(esferas_iniciales_list) == N_eslabones:
        for k in range(N_eslabones):
            esf_init = esferas_iniciales_list[k]
            ax.scatter(esf_init[:, 0], esf_init[:, 1], esf_init[:, 2], 
                       c='gray', marker='o', s=100 * RADIO_ESFERA, alpha=0.3)
            all_coords.append(esf_init)
        
        ax.scatter([], [], [], c='gray', marker='o', s=100 * RADIO_ESFERA, alpha=0.3, label='Inicial')

    for k in range(N_eslabones):
        color_base_eslabon = COLORES_BASE[k % len(COLORES_BASE)]
        scatter = ax.scatter([], [], [], 
                             c=color_base_eslabon, marker='o', s=200 * RADIO_ESFERA, alpha=0.4)
        _plot_references['scatters'].append(scatter)
        
    for k in range(N_eslabones):
        line = ax.plot([], [], [], color=COLORES_BASE[k % len(COLORES_BASE)], linestyle='-', linewidth=3)[0]
        _plot_references['lines'].append(line)
        
    for k in range(N_eslabones - 1):
        junta_line = ax.plot([], [], [], color='black', linestyle='--', linewidth=2, label='Junta' if k == 0 else None)[0]
        _plot_references['juntas'].append(junta_line)

    ax.scatter(T_BASE_GLOBAL[0], T_BASE_GLOBAL[1], T_BASE_GLOBAL[2], c='black', marker='X', s=200, label='Origen Global (0,0,0)')

    plt.legend()
    plt.ion()
    plt.show()
    
    return fig, ax

def actualizar_visualizacion_dinamica(esferas_globales_list, angulos_list, centros_colisionantes_global):
    """Actualiza los datos de los objetos gráficos existentes en el plot."""
    # [Mantener el código de actualizar_visualizacion sin cambios, renombrado a actualizar_visualizacion_dinamica]
    ax = _plot_references['ax']
    if ax is None:
        print("Error: La visualización no ha sido inicializada.")
        return

    N = len(esferas_globales_list)
    COLORES_BASE = ['blue', 'green', 'yellow', 'cyan', 'magenta']
    all_coords = []
    colision_global = bool(centros_colisionantes_global)
    
    for k in range(N):
        esf_global = esferas_globales_list[k]
        scatter = _plot_references['scatters'][k]
        line = _plot_references['lines'][k]
        color_base_eslabon = COLORES_BASE[k % len(COLORES_BASE)]
        
        x, y, z = esf_global[:, 0], esf_global[:, 1], esf_global[:, 2]
        scatter._offsets3d = (x, y, z)
        
        colores_esferas = []
        for centro_coord in esf_global:
            centro_tupla = tuple(centro_coord)
            if centro_tupla in centros_colisionantes_global:
                colores_esferas.append('red')
            else:
                colores_esferas.append(color_base_eslabon)
        scatter.set_facecolors(colores_esferas)
        
        if len(esf_global) > 1:
            line_data = np.hstack([esf_global[i:i+2, :].T for i in range(len(esf_global) - 1)])
            line.set_data(line_data[0], line_data[1])
            line.set_3d_properties(line_data[2])
        else:
            line.set_data([], [])
            line.set_3d_properties([])
        
        all_coords.append(esf_global)

    for k in range(N - 1):
        p_union_k = esferas_globales_list[k][-1]     
        p_origen_k1 = esferas_globales_list[k+1][0] 
        junta_line = _plot_references['juntas'][k]

        x_junta = [p_union_k[0], p_origen_k1[0]]
        y_junta = [p_union_k[1], p_origen_k1[1]]
        z_junta = [p_union_k[2], p_origen_k1[2]]

        junta_line.set_data(x_junta, y_junta)
        junta_line.set_3d_properties(z_junta)

    if all_coords:
        all_coords_stack = np.vstack(all_coords)
        max_abs_coord = np.max(np.abs(all_coords_stack))
        max_range = max_abs_coord + RADIO_ESFERA + 1
        
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([-max_range, max_range]) 
    
    angulos_titulo = " | ".join([f'E{i}: {ang:.1f}°' for i, ang in enumerate(angulos_list)])
    ax.set_title(f'Rotaciones: {angulos_titulo} | Colisión: {"SI" if colision_global else "NO"}')

    plt.draw()
    plt.pause(2) 

def finalizar_visualizacion():
    """Deja la ventana de plot abierta al finalizar la simulación."""
    plt.ioff()
    if _plot_references['ax'] is not None:
        print("Simulación terminada. Cerrar la ventana de gráfico para finalizar el programa.")
        plt.show()

# ----------------------------------------------
# 🎯 MODO PLOT ÚNICO (Función para crear un nuevo plot por paso)
# ----------------------------------------------

def visualizar_plot_unico(esferas_globales_list, angulos_list, centros_colisionantes_global, esferas_iniciales_list=None):
    """
    Dibuja N eslabones, creando un nuevo plot (ventana) para cada llamada.
    """
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    N = len(esferas_globales_list)
    COLORES_BASE = ['blue', 'green', 'yellow', 'cyan', 'magenta']
    
    all_coords = []
    colision_global = bool(centros_colisionantes_global)
    
    # Fijar la vista para el plano XY
    set_view(ax, 'xz_rotada')
    ax.dist = 8

    # --- 1. Dibujar Posición Inicial (Gris) ---
    if esferas_iniciales_list is not None and len(esferas_iniciales_list) == N:
        for k in range(N):
            esf_init = esferas_iniciales_list[k]
            ax.scatter(esf_init[:, 0], esf_init[:, 1], esf_init[:, 2], 
                       c='gray', marker='o', s=100 * RADIO_ESFERA, alpha=0.3)
            all_coords.append(esf_init)
        
        ax.scatter([], [], [], c='gray', marker='o', s=100 * RADIO_ESFERA, alpha=0.3, label='Inicial')


    # --- 2. Dibujar Posición Actual (Eslabones) ---
    for k in range(N):
        esf_global = esferas_globales_list[k]
        color_base_eslabon = COLORES_BASE[k % len(COLORES_BASE)]
        
        # Dibujar Líneas (Cuerpo del eslabón)
        if len(esf_global) > 1:
            for i in range(len(esf_global) - 1):
                p1, p2 = esf_global[i], esf_global[i+1]
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 
                        color=color_base_eslabon, linestyle='-', linewidth=3)
        
        # Dibujar Esferas (Puntos)
        for centro_coord in esf_global:
            centro_tupla = tuple(centro_coord)
            
            # Colorear en rojo si hay colisión
            if centro_tupla in centros_colisionantes_global:
                color_esfera = 'red' 
            else:
                color_esfera = color_base_eslabon 

            ax.scatter(centro_coord[0], centro_coord[1], centro_coord[2], 
                       c=color_esfera, marker='o', s=200 * RADIO_ESFERA, 
                       label=None, alpha=0.4) 

        all_coords.append(esf_global)

    # --- 3. Dibujar Juntas de Conexión ---
    for k in range(N - 1):
        p_union_k = esferas_globales_list[k][-1]     
        p_origen_k1 = esferas_globales_list[k+1][0] 
        
        ax.plot([p_union_k[0], p_origen_k1[0]], [p_union_k[1], p_origen_k1[1]], [p_union_k[2], p_origen_k1[2]], 
                color='black', linestyle='--', linewidth=2, label=f'Junta {k+1}-{k+2}' if k == 0 else None)


    # --- 4. Configuración Final del Gráfico ---
    ax.scatter(T_BASE_GLOBAL[0], T_BASE_GLOBAL[1], T_BASE_GLOBAL[2], c='black', marker='X', s=200, label='Origen Global (0,0,0)')

    # Cálculo de límites adaptativos
    if all_coords:
        all_coords_stack = np.vstack(all_coords)
        max_abs_coord = np.max(np.abs(all_coords_stack))
        max_range = max_abs_coord + RADIO_ESFERA + 1
        
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([-max_range, max_range]) 
    
    # Título
    angulos_titulo = " | ".join([f'E{i+1}: {ang:.1f}°' for i, ang in enumerate(angulos_list)])
    ax.set_title(f'Rotaciones: {angulos_titulo} | Colisión: {"SI" if colision_global else "NO"}')

    plt.show() # Bloquea la ejecución hasta que se cierre, o se abre una ventana por frame