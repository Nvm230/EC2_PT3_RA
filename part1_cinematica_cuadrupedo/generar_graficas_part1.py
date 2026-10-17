#!/usr/bin/env python3
"""
Generador de Gráficas y Métricas Visuales para la Parte 1
Guarda figuras en la carpeta `outputs/`:
- outputs/part1_errores_benchmark.png
- outputs/part1_estructura_robot_3d.png
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from cinematica_directa_jacobiano import KinematicsValidator, A1KinematicsManual

def main():
    os.makedirs("/home/nvm/Documents/UTEC/Avanzada_tareas/outputs", exist_ok=True)
    validator = KinematicsValidator()
    
    # 1. Benchmark de 200 configuraciones
    num_samples = 200
    q_low = np.array([-0.8, -1.0, -2.6] * 4)
    q_high = np.array([0.8,  3.5, -0.9] * 4)
    
    fk_errors = []
    jac_errors = []
    
    np.random.seed(123)
    for _ in range(num_samples):
        q = np.random.uniform(q_low, q_high)
        res = validator.test_configuration(q)
        fk_errors.append(res["error_fk_max"])
        jac_errors.append(res["error_jac_max"])
        
    # --- Gráfica 1: Histograma de Errores Numéricos ---
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5), dpi=150)
    
    ax1.hist(fk_errors, bins=25, color="#1f77b4", edgecolor="black", alpha=0.8)
    ax1.set_title("Distribución de Error Absoluto en Cinemática Directa (FK)")
    ax1.set_xlabel("Error Máximo |p_manual - p_pinocchio| (m)")
    ax1.set_ylabel("Frecuencia (muestras)")
    ax1.grid(True, linestyle="--", alpha=0.5)
    
    ax2.hist(jac_errors, bins=25, color="#2ca02c", edgecolor="black", alpha=0.8)
    ax2.set_title("Distribución de Error Absoluto en Jacobiano de Cuerpo (J)")
    ax2.set_xlabel("Error Máximo |J_manual - J_pinocchio|")
    ax2.set_ylabel("Frecuencia (muestras)")
    ax2.grid(True, linestyle="--", alpha=0.5)
    
    plt.suptitle("Validación de Precisión Numérica (Unitree A1 vs Pinocchio)", fontsize=14, fontweight="bold")
    plt.tight_layout()
    output_path1 = "/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part1_errores_benchmark.png"
    plt.savefig(output_path1)
    plt.close()
    print(f"Gráfica guardada: {output_path1}")
    
    # --- Gráfica 2: Diagrama Esquemático 3D de las 4 Patas y Base ---
    manual = A1KinematicsManual()
    q_stand = np.array([0.0, 0.75, -1.50] * 4)
    _, dict_feet = manual.forward_kinematics_full(q_stand)
    
    fig = plt.figure(figsize=(8, 7), dpi=150)
    ax = fig.add_subplot(111, projection="3d")
    
    # Dibujar Base Trunk (rectángulo)
    dx, dy = manual.d_x, manual.d_y
    base_corners = np.array([
        [ dx,  dy, 0.0],
        [ dx, -dy, 0.0],
        [-dx, -dy, 0.0],
        [-dx,  dy, 0.0],
        [ dx,  dy, 0.0]
    ])
    ax.plot(base_corners[:, 0], base_corners[:, 1], base_corners[:, 2], "k-", linewidth=3, label="Base (Trunk)")
    ax.scatter(0, 0, 0, color="black", s=80, marker="o", label="Origen Base (0,0,0)")
    
    colors = {"FL": "#d62728", "FR": "#ff7f0e", "RL": "#1f77b4", "RR": "#2ca02c"}
    
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
        
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-o", color=colors[name], linewidth=2.5, markersize=6, label=f"Pata {name}")
        ax.scatter(p_foot[0], p_foot[1], p_foot[2], color=colors[name], s=100, marker="^")
        ax.text(p_foot[0], p_foot[1], p_foot[2]-0.03, f"{name}_foot", fontsize=9, fontweight="bold")
        
    ax.set_xlabel("X (m) [Avance]")
    ax.set_ylabel("Y (m) [Lateral]")
    ax.set_zlabel("Z (m) [Vertical]")
    ax.set_title("Configuración Geométrica y Posición de Pies (Unitree A1)", fontsize=13, fontweight="bold")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_box_aspect([1, 0.8, 0.8])
    ax.view_init(elev=20, azim=45)
    
    output_path2 = "/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part1_estructura_robot_3d.png"
    plt.savefig(output_path2)
    plt.close()
    print(f"Gráfica guardada: {output_path2}")

if __name__ == "__main__":
    main()
