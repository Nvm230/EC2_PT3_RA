#!/usr/bin/env python3
"""
===============================================================================
GENERADOR DE GRÁFICAS Y BENCHMARK CINEMÁTICO - PARTE 1 (UNITREE A1)
===============================================================================
Descripción:
    Evalúa numéricamente la cinemática directa (FK) y el Jacobiano de cuerpo
    completo calculados desde primeros principios (NumPy) frente a la librería
    Pinocchio para 200 configuraciones aleatorias.
    
    Genera y guarda en `outputs/`:
    1. outputs/part1_errores_benchmark.png (Distribución de errores numéricos)
    2. outputs/part1_estructura_robot_3d.png (Diagrama 3D del robot y pies)
===============================================================================
"""

import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Importar validador y cinemática manual
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from cinematica_directa_jacobiano import KinematicsValidator, A1KinematicsManual


def generate_benchmark_plot(validator, num_samples=200):
    """Ejecuta el benchmark y genera el histograma de errores de alta resolución."""
    q_low = np.array([-0.8, -1.0, -2.6] * 4)
    q_high = np.array([0.8,  3.5, -0.9] * 4)
    
    fk_errors = []
    jac_errors = []
    
    np.random.seed(42)
    for _ in range(num_samples):
        q = np.random.uniform(q_low, q_high)
        res = validator.test_configuration(q)
        fk_errors.append(res["error_fk_max"])
        jac_errors.append(res["error_jac_max"])
        
    fk_errors = np.array(fk_errors)
    jac_errors = np.array(jac_errors)
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5), dpi=300)
    plt.subplots_adjust(wspace=0.28, top=0.85, bottom=0.15)
    
    # Subplot 1: Cinemática Directa (FK)
    n1, bins1, patches1 = ax1.hist(
        fk_errors, bins=20, color="#1f77b4", edgecolor="#0d3b66", alpha=0.85, linewidth=1.2
    )
    ax1.set_title("Error Absoluto en Cinemática Directa (FK)", fontsize=12, fontweight="bold", pad=10)
    ax1.set_xlabel("Error Máximo ||p_manual - p_pinocchio|| (m)", fontsize=11)
    ax1.set_ylabel("Frecuencia (muestras)", fontsize=11)
    ax1.grid(True, linestyle="--", alpha=0.5)
    
    # Caja de estadísticas FK
    stats_text1 = (
        f"Muestras: {num_samples}\n"
        f"Error Máx: {np.max(fk_errors):.2e} m\n"
        f"Error Med: {np.mean(fk_errors):.2e} m\n"
        f"Desv. Est: {np.std(fk_errors):.2e} m\n"
        f"Precisión: IEEE 754 (Double)"
    )
    ax1.text(
        0.95, 0.95, stats_text1, transform=ax1.transAxes, fontsize=9.5,
        verticalalignment="top", horizontalalignment="right",
        bbox=dict(boxstyle="round,pad=0.5", facecolor="#eaf2f8", edgecolor="#1f77b4", alpha=0.9)
    )
    
    # Subplot 2: Jacobiano de Cuerpo Completo (J)
    n2, bins2, patches2 = ax2.hist(
        jac_errors, bins=20, color="#2ca02c", edgecolor="#1b5e20", alpha=0.85, linewidth=1.2
    )
    ax2.set_title("Error Absoluto en Jacobiano de Cuerpo (J)", fontsize=12, fontweight="bold", pad=10)
    ax2.set_xlabel("Error Máximo ||J_manual - J_pinocchio||", fontsize=11)
    ax2.set_ylabel("Frecuencia (muestras)", fontsize=11)
    ax2.grid(True, linestyle="--", alpha=0.5)
    
    # Caja de estadísticas Jacobiano
    stats_text2 = (
        f"Muestras: {num_samples}\n"
        f"Error Máx: {np.max(jac_errors):.2e}\n"
        f"Error Med: {np.mean(jac_errors):.2e}\n"
        f"Desv. Est: {np.std(jac_errors):.2e}\n"
        f"Precisión: Exacta (Álgebra Pura)"
    )
    ax2.text(
        0.95, 0.95, stats_text2, transform=ax2.transAxes, fontsize=9.5,
        verticalalignment="top", horizontalalignment="right",
        bbox=dict(boxstyle="round,pad=0.5", facecolor="#eafaf1", edgecolor="#2ca02c", alpha=0.9)
    )
    
    fig.suptitle(
        "Validación Numérica: Cinemática Analítica Propia vs. Pinocchio (Unitree A1)",
        fontsize=14, fontweight="bold"
    )
    
    output_path = "/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part1_errores_benchmark.png"
    plt.savefig(output_path, bbox_inches="tight")
    plt.close()
    print(f"Benchmark guardado: {output_path}")


def generate_robot_3d_plot(manual):
    """Genera diagrama esquemático 3D con perspectiva isométrica clara del robot Unitree A1."""
    q_stand = np.array([0.0, 0.75, -1.50] * 4)
    _, dict_feet = manual.forward_kinematics_full(q_stand)
    
    fig = plt.figure(figsize=(9, 8), dpi=300)
    ax = fig.add_subplot(111, projection="3d")
    
    # Base Tronco (polígono sombreado)
    dx, dy = manual.d_x, manual.d_y
    base_corners = np.array([
        [ dx,  dy, 0.0],
        [ dx, -dy, 0.0],
        [-dx, -dy, 0.0],
        [-dx,  dy, 0.0]
    ])
    
    trunk_poly = Poly3DCollection([base_corners], alpha=0.35, facecolor="#4a5568", edgecolor="#1a202c", linewidth=2.5)
    ax.add_collection3d(trunk_poly)
    
    # Ejes coordenados en el origen de la base
    axis_len = 0.07
    ax.quiver(0, 0, 0, axis_len, 0, 0, color="red", linewidth=2.5, arrow_length_ratio=0.3)
    ax.quiver(0, 0, 0, 0, axis_len, 0, color="green", linewidth=2.5, arrow_length_ratio=0.3)
    ax.quiver(0, 0, 0, 0, 0, axis_len, color="blue", linewidth=2.5, arrow_length_ratio=0.3)
    ax.text(axis_len+0.01, 0, 0, "X (Adelante)", color="red", fontsize=8, fontweight="bold")
    ax.text(0, axis_len+0.01, 0, "Y (Izq)", color="green", fontsize=8, fontweight="bold")
    ax.text(0, 0, axis_len+0.01, "Z (Arriba)", color="blue", fontsize=8, fontweight="bold")
    
    # Centro de la base
    ax.scatter(0, 0, 0, color="black", s=90, marker="o", label="Origen Base B (0,0,0)")
    
    colors = {
        "FL": "#e53e3e",  # Rojo
        "FR": "#dd6b20",  # Naranja
        "RL": "#3182ce",  # Azul
        "RR": "#38a169"   # Verde
    }
    
    leg_full_names = {
        "FL": "Delantera Izquierda (FL)",
        "FR": "Delantera Derecha (FR)",
        "RL": "Trasera Izquierda (RL)",
        "RR": "Trasera Derecha (RR)"
    }
    
    for i, info in enumerate(manual.legs_info):
        name = info["name"]
        q_leg = q_stand[info["q_start"]:info["q_start"]+3]
        p_foot, j_pos, _, _ = manual.forward_kinematics_leg(i, q_leg)
        
        pts = np.vstack([
            j_pos["hip"],
            j_pos["thigh"],
            j_pos["calf"],
            p_foot
        ])
        
        # Trazar eslabones de la pata
        ax.plot(
            pts[:, 0], pts[:, 1], pts[:, 2], "-o", color=colors[name],
            linewidth=3.0, markersize=7, label=leg_full_names[name]
        )
        
        # Marcador del pie
        ax.scatter(p_foot[0], p_foot[1], p_foot[2], color=colors[name], s=120, marker="^")
        
        # Etiqueta de pie
        z_offset = -0.025
        ax.text(
            p_foot[0], p_foot[1], p_foot[2] + z_offset,
            f"{name}_foot\n[{p_foot[0]:.2f}, {p_foot[1]:.2f}, {p_foot[2]:.2f}]",
            fontsize=8, fontweight="bold", ha="center"
        )
        
        # Proyección de sombra en el plano de contacto
        ax.plot([p_foot[0], p_foot[0]], [p_foot[1], p_foot[1]], [p_foot[2], -0.32],
                linestyle=":", color=colors[name], alpha=0.5)
        ax.scatter(p_foot[0], p_foot[1], -0.32, color=colors[name], alpha=0.3, s=40, marker="o")
        
    ax.set_xlim([-0.30, 0.30])
    ax.set_ylim([-0.25, 0.25])
    ax.set_zlim([-0.38, 0.12])
    ax.set_xlabel("X (m) [Longitudinal]", fontsize=10, labelpad=8)
    ax.set_ylabel("Y (m) [Lateral]", fontsize=10, labelpad=8)
    ax.set_zlabel("Z (m) [Vertical]", fontsize=10, labelpad=8)
    
    ax.set_title("Estructura Cinemática y Posición de Pies (Unitree A1)\nPostura Nominal de Reposo", fontsize=13, fontweight="bold", pad=15)
    ax.legend(loc="upper right", fontsize=8.5, framealpha=0.95)
    ax.set_box_aspect([1.1, 0.9, 0.9])
    ax.view_init(elev=22, azim=45)
    
    output_path = "/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part1_estructura_robot_3d.png"
    plt.savefig(output_path, bbox_inches="tight")
    plt.close()
    print(f"Diagrama 3D guardado: {output_path}")


def main():
    os.makedirs("/home/nvm/Documents/UTEC/Avanzada_tareas/outputs", exist_ok=True)
    validator = KinematicsValidator()
    manual = A1KinematicsManual()
    
    generate_benchmark_plot(validator, num_samples=200)
    generate_robot_3d_plot(manual)
    print(">>> Gráficas de la Parte 1 generadas exitosamente. <<<")


if __name__ == "__main__":
    main()
