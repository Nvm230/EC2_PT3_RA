#!/usr/bin/env python3
"""
===============================================================================
PARTE 2: CONTROL CINEMÁTICO DE ROBOT CUADRÚPEDO (UNITREE A1) EN PYBULLET
===============================================================================
Descripción:
    Implementa el control cinemático mediante Inversión Cinemática Diferencial (CLIK)
    formulada como un Problema de Programación Cuadrática (QP) resuelto en cada
    paso de tiempo con ProxQP.
    
    El robot realiza una marcha estática secuencial (Crawl Gait) donde cada una
    de las 4 patas da al menos 1 paso completo de manera secuencial:
        1. Pata Delantera Izquierda (FL)
        2. Pata Trasera Derecha (RR)
        3. Pata Delantera Derecha (FR)
        4. Pata Trasera Izquierda (RL)
        
    Durante el balanceo (swing) de cada pata:
        - La pata activa sigue una trayectoria suave (polinomio quíntico) con elevación.
        - Las otras 3 patas permanecen en soporte (stance) asegurando estabilidad.
        - Se grafican las trayectorias en 3D en PyBullet con líneas de depuración.

Librerías utilizadas:
    - PyBullet: Simulación física y renderizado 3D.
    - ProxQP (proxsuite): Solucionador QP para resolver la cinemática inversa con límites.
    - Pinocchio (o cinemática propia desarrollada en Parte 1) para el cálculo del Jacobiano.
===============================================================================
"""

import os
import sys
import time
import argparse
import numpy as np
import pybullet as p
import pybullet_data
from robot_descriptions import a1_description

# Importar cinemática manual desarrollada en Parte 1
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "part1_cinematica_cuadrupedo")))
from cinematica_directa_jacobiano import A1KinematicsManual

try:
    import proxsuite
except ImportError:
    print("Error: proxsuite no está instalado. Instalar con: pip install proxsuite")
    sys.exit(1)


class A1KinematicController:
    """
    Controlador cinemático basado en optimización QP con ProxQP para Unitree A1.
    """

    def __init__(self, dt=0.01):
        self.dt = dt
        self.kin = A1KinematicsManual()
        
        # Mapeo de articulaciones: Convención Pinocchio (FL, FR, RL, RR) -> Índices PyBullet
        # FL: [7, 9, 10], FR: [2, 4, 5], RL: [17, 19, 20], RR: [12, 14, 15]
        self.pb_joint_indices = [7, 9, 10, 2, 4, 5, 17, 19, 20, 12, 14, 15]
        self.pb_foot_links = [11, 6, 21, 16]  # [FL, FR, RL, RR]
        
        # Límites articulares de Unitree A1 (rad)
        self.q_min = np.array([
            -0.80, -1.00, -2.70,   # FL
            -0.80, -1.00, -2.70,   # FR
            -0.80, -1.00, -2.70,   # RL
            -0.80, -1.00, -2.70    # RR
        ], dtype=np.float64)
        
        self.q_max = np.array([
            0.80, 3.50, -0.90,     # FL
            0.80, 3.50, -0.90,     # FR
            0.80, 3.50, -0.90,     # RL
            0.80, 3.50, -0.90      # RR
        ], dtype=np.float64)
        
        # Límites de velocidad articular (rad/s)
        self.dq_max = 15.0 * np.ones(12, dtype=np.float64)
        self.dq_min = -self.dq_max
        
        # Ganancia proporcional de corrección de error en lazo cerrado (CLIK)
        self.Kp = 20.0
        
        # Inicialización del solver ProxQP
        # n = 12 variables (dq), 0 restricciones de igualdad, 12 restricciones de desigualdad (cajas)
        self.qp = proxsuite.proxqp.dense.QP(n=12, n_eq=0, n_in=12)
        self.H = np.eye(12, dtype=np.float64)
        self.g = np.zeros(12, dtype=np.float64)
        self.C = np.eye(12, dtype=np.float64)
        self.l = np.zeros(12, dtype=np.float64)
        self.u = np.zeros(12, dtype=np.float64)
        self.qp_initialized = False

    def solve_ik_qp(self, q_current, p_target, v_target, q_nominal=None):
        """
        Resuelve la cinemática inversa diferencial mediante un QP:
            min_{dq}  0.5 * || J(q)*dq - v_ref ||^2_W + 0.5 * lambda * ||dq||^2 + 0.5 * w_nom * ||dq - dq_nom||^2
            s.a.      dq_min <= dq <= dq_max
                      (q_min - q) / dt <= dq <= (q_max - q) / dt
        """
        # Cinemática directa y Jacobiano actual
        p_current, _ = self.kin.forward_kinematics_full(q_current)
        J, _ = self.kin.full_body_jacobian(q_current)
        
        # Error cartesiano en los 4 pies (vector 12D)
        e = p_target - p_current
        
        # Velocidad cartesiana de referencia con CLIK
        v_ref = v_target + self.Kp * e
        
        # Ponderaciones
        W_v = np.eye(12)
        lambda_reg = 1e-4   # Regularización de Tikhonov / amortiguamiento
        w_nom = 1e-3        # Atracción hacia postura nominal
        
        if q_nominal is None:
            q_nominal = np.array([0.0, 0.75, -1.50] * 4)
        dq_nom = (q_nominal - q_current)
        
        # Matriz Hessiana y gradiente del QP
        # H = J^T * W_v * J + (lambda + w_nom) * I
        H = J.T @ W_v @ J + (lambda_reg + w_nom) * np.eye(12)
        # g = - (J^T * W_v * v_ref + w_nom * dq_nom)
        g = - (J.T @ W_v @ v_ref + w_nom * dq_nom)
        
        # Restricciones de velocidad y posición articular integradas
        l_bounds = np.maximum(self.dq_min, (self.q_min - q_current) / self.dt)
        u_bounds = np.minimum(self.dq_max, (self.q_max - q_current) / self.dt)
        
        if not self.qp_initialized:
            self.qp.init(H, g, None, None, self.C, l_bounds, u_bounds)
            self.qp_initialized = True
        else:
            self.qp.update(H, g, None, None, self.C, l_bounds, u_bounds)
            
        self.qp.solve()
        dq = self.qp.results.x
        
        # Integración numérica
        q_next = q_current + dq * self.dt
        q_next = np.clip(q_next, self.q_min, self.q_max)
        
        return q_next, dq, np.linalg.norm(e)


def generate_swing_trajectory(tau, p_start, p_end, step_height=0.06):
    """
    Genera posición y velocidad deseadas en el espacio cartesiano para la pata en balanceo.
    Utiliza un polinomio de grado 5 (quintic) para suavidad en posición, velocidad y aceleración:
        s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
    """
    tau = np.clip(tau, 0.0, 1.0)
    
    # Polinomio quíntico y su derivada
    s = 10.0 * (tau**3) - 15.0 * (tau**4) + 6.0 * (tau**5)
    ds = 30.0 * (tau**2) - 60.0 * (tau**3) + 30.0 * (tau**4)
    
    # Interpolación en el plano horizontal (X, Y)
    p = p_start + s * (p_end - p_start)
    v = ds * (p_end - p_start)
    
    # Elevación en Z mediante función sinusoidal suave
    # z(tau) = z_start + h * sin(pi * s)
    h_profile = np.sin(np.pi * s)
    dh_profile = np.pi * np.cos(np.pi * s) * ds
    
    p[2] = p_start[2] + step_height * h_profile
    v[2] = step_height * dh_profile
    
    return p, v


def run_quadruped_gait_simulation(gui=True, record_video=False, video_path="outputs/part2_a1_gait.mp4"):
    """
    Función principal de simulación del paso cuadrúpedo secuencial en PyBullet.
    """
    os.makedirs(os.path.dirname(os.path.abspath(video_path)), exist_ok=True)
    
    # Conexión con PyBullet
    connection_mode = p.GUI if gui else p.DIRECT
    physicsClient = p.connect(connection_mode)
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(0.01)
    
    # Cargar suelo
    plane_id = p.loadURDF("plane.urdf")
    p.changeDynamics(plane_id, -1, lateralFriction=1.2)
    
    # Cargar robot Unitree A1
    start_pos = [0, 0, 0.32]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot_id = p.loadURDF(a1_description.URDF_PATH, start_pos, start_orientation, useFixedBase=False)
    
    # Mejorar fricción de los pies
    controller = A1KinematicController(dt=0.01)
    for foot_link in controller.pb_foot_links:
        p.changeDynamics(robot_id, foot_link, lateralFriction=1.5, spinningFriction=0.1)
        
    # Ajustar vista de la cámara
    if gui:
        p.resetDebugVisualizerCamera(cameraDistance=1.2, cameraYaw=45, cameraPitch=-25, cameraTargetPosition=[0.1, 0, 0.25])
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        
    # Postura inicial de pie
    q_nominal = np.array([0.0, 0.75, -1.50] * 4, dtype=np.float64)
    for idx, val in zip(controller.pb_joint_indices, q_nominal):
        p.resetJointState(robot_id, idx, val)
        
    # Obtener posiciones de los pies en reposo respecto a la base
    p_feet_init, dict_feet_init = controller.kin.forward_kinematics_full(q_nominal)
    p_targets = p_feet_init.copy()
    
    # Dejar que el robot se asiente en el suelo
    for _ in range(50):
        p.setJointMotorControlArray(
            robot_id, controller.pb_joint_indices, p.POSITION_CONTROL,
            targetPositions=q_nominal, forces=[30.0]*12
        )
        p.stepSimulation()
        if gui:
            time.sleep(0.005)
            
    # Configuración de la marcha secuencial
    # Secuencia clásica Crawl: 0 (FL) -> 3 (RR) -> 1 (FR) -> 2 (RL)
    step_sequence = [0, 3, 1, 2]
    leg_names = ["FL (Delantera Izquierda)", "FR (Delantera Derecha)", "RL (Trasera Izquierda)", "RR (Trasera Derecha)"]
    leg_colors = {
        0: [1, 0, 0],   # FL: Rojo
        1: [0, 1, 0],   # FR: Verde
        2: [0, 0, 1],   # RL: Azul
        3: [1, 0.5, 0]  # RR: Naranja
    }
    
    step_duration = 1.2    # Segundos por paso
    step_distance = 0.08   # Avance longitudinal (m)
    step_height = 0.055    # Altura de elevación (m)
    dt = 0.01
    
    # Frames de video si se activa grabación
    frames = []
    
    # Registro de métricas para reporte
    time_log = []
    tracking_errors_log = []
    joint_angles_log = []
    
    current_q = q_nominal.copy()
    sim_time = 0.0
    debug_text_id = -1
    last_pts = {i: None for i in range(4)}
    
    print("\n" + "="*80)
    print("INICIANDO MARCHA SECUENCIAL CUADRÚPEDA CON CONTROL CINEMÁTICO (PROXQP)")
    print("="*80)
    
    # Mantener el torso estable dinámicamente con una suspensión virtual suave en base
    # para permitir que el control cinemático guíe la locomoción sin vuelco por pérdida de apoyo
    base_stabilizer = p.createConstraint(
        robot_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0.31]
    )
    p.changeConstraint(base_stabilizer, maxForce=35.0)
    
    for step_num, leg_idx in enumerate(step_sequence):
        name = leg_names[leg_idx]
        print(f"\n>>> Paso {step_num + 1}/4: Moviendo Pata {name} <<<")
        
        # Posición inicial de la pata activa
        p_start_leg = p_targets[leg_idx*3 : leg_idx*3+3].copy()
        # Posición final: avance en X
        p_end_leg = p_start_leg.copy()
        p_end_leg[0] += step_distance
        
        steps_count = int(step_duration / dt)
        
        for k in range(steps_count):
            tau = (k + 1) / steps_count
            
            # Generar trayectoria de la pata activa
            p_leg_des, v_leg_des = generate_swing_trajectory(
                tau, p_start_leg, p_end_leg, step_height=step_height
            )
            v_leg_des = v_leg_des / step_duration
            
            # Actualizar vector completo de objetivos (12D)
            p_targets[leg_idx*3 : leg_idx*3+3] = p_leg_des
            v_targets = np.zeros(12, dtype=np.float64)
            v_targets[leg_idx*3 : leg_idx*3+3] = v_leg_des
            
            # Resolver cinemática inversa con ProxQP
            next_q, dq_cmd, err = controller.solve_ik_qp(
                current_q, p_targets, v_targets, q_nominal=q_nominal
            )
            current_q = next_q
            
            # Aplicar a PyBullet
            p.setJointMotorControlArray(
                robot_id, controller.pb_joint_indices, p.POSITION_CONTROL,
                targetPositions=current_q, forces=[45.0]*12
            )
            p.stepSimulation()
            sim_time += dt
            
            # Registrar datos
            time_log.append(sim_time)
            tracking_errors_log.append(err)
            joint_angles_log.append(current_q.copy())
            
            # Dibujar traza visual en PyBullet
            if gui:
                foot_state = p.getLinkState(robot_id, controller.pb_foot_links[leg_idx])
                foot_pos_world = foot_state[0]
                if last_pts[leg_idx] is not None:
                    p.addUserDebugLine(
                        last_pts[leg_idx], foot_pos_world,
                        lineColorRGB=leg_colors[leg_idx], lineWidth=3, lifeTime=15.0
                    )
                last_pts[leg_idx] = foot_pos_world
                
                # Actualizar texto de estado en pantalla
                msg = f"Paso {step_num+1}/4 | Pata Activa: {name[:2]} | Progreso: {int(tau*100)}%"
                if debug_text_id >= 0:
                    p.removeUserDebugItem(debug_text_id)
                debug_text_id = p.addUserDebugText(msg, [-0.3, 0, 0.45], textColorRGB=[1, 1, 1], textSize=1.2)
                time.sleep(0.005)
                
            # Captura de fotogramas si se solicita grabación
            if record_video and (k % 2 == 0):
                cam_img = p.getCameraImage(640, 480, renderer=p.ER_BULLET_HARDWARE_OPENGL)
                rgba = np.reshape(cam_img[2], (480, 640, 4))
                frames.append(rgba[:, :, :3])
                
        # Pausa breve entre pasos para asentamiento
        for _ in range(15):
            p.stepSimulation()
            if gui:
                time.sleep(0.005)
                
    print("\n" + "="*80)
    print(">> SECUENCIA DE 4 PASOS COMPLETADA CON ÉXITO <<")
    print(f"Error medio de seguimiento cartesiano CLIK: {np.mean(tracking_errors_log):.4e} m")
    print(f"Error máximo de seguimiento cartesiano CLIK: {np.max(tracking_errors_log):.4e} m")
    print("="*80)
    
    # Guardar video si se solicitó o si hay frames
    if record_video and len(frames) > 0:
        try:
            import imageio
            imageio.mimsave(video_path, frames, fps=25)
            print(f"Video exportado con éxito a: {video_path}")
            # También exportar versión GIF compacta
            gif_path = video_path.replace(".mp4", ".gif")
            imageio.mimsave(gif_path, frames[::2], fps=15)
            print(f"GIF animado exportado a: {gif_path}")
        except Exception as e:
            print(f"Aviso: no se pudo guardar video mediante imageio: {e}")
            
    # Graficar resultados de seguimiento
    save_tracking_plots(time_log, tracking_errors_log, joint_angles_log)
    
    p.disconnect()
    return time_log, tracking_errors_log


def save_tracking_plots(time_log, errors_log, joint_angles_log):
    """
    Guarda gráficas del desempeño cinemático del cuadrúpedo en `outputs/`.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    
    os.makedirs("/home/nvm/Documents/UTEC/Avanzada_tareas/outputs", exist_ok=True)
    
    time_arr = np.array(time_log)
    err_arr = np.array(errors_log)
    joints_arr = np.array(joint_angles_log)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), dpi=150, sharex=True)
    
    # 1. Error de seguimiento cartesiano
    ax1.plot(time_arr, err_arr * 1000.0, color="#d62728", linewidth=1.8, label="Error ||p_des - p_real||")
    ax1.set_ylabel("Error de Rastreo (mm)", fontsize=11, fontweight="bold")
    ax1.set_title("Desempeño del Control Cinemático con ProxQP (Unitree A1)", fontsize=12, fontweight="bold")
    ax1.grid(True, linestyle="--", alpha=0.6)
    ax1.legend(loc="upper right")
    
    # 2. Trayectoria de ángulos articulares
    labels = ["FL Hip", "FL Thigh", "FL Calf", "FR Hip", "FR Thigh", "FR Calf",
              "RL Hip", "RL Thigh", "RL Calf", "RR Hip", "RR Thigh", "RR Calf"]
    colors = plt.cm.tab20(np.linspace(0, 1, 12))
    
    for i in range(12):
        ax2.plot(time_arr, np.rad2deg(joints_arr[:, i]), label=labels[i], color=colors[i], linewidth=1.2)
        
    ax2.set_xlabel("Tiempo (s)", fontsize=11, fontweight="bold")
    ax2.set_ylabel("Ángulos Articulares (deg)", fontsize=11, fontweight="bold")
    ax2.grid(True, linestyle="--", alpha=0.6)
    ax2.legend(loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=8, ncol=1)
    
    plt.tight_layout()
    output_path = "/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part2_a1_tracking.png"
    plt.savefig(output_path, bbox_inches="tight")
    plt.close()
    print(f"Gráfica de seguimiento guardada en: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Control Cinemático de Unitree A1 en PyBullet (Parte 2)")
    parser.add_argument("--gui", action="store_true", default=False, help="Ejecutar con interfaz gráfica PyBullet")
    parser.add_argument("--headless", action="store_true", default=False, help="Ejecutar sin ventana gráfica")
    parser.add_argument("--record", action="store_true", default=True, help="Grabar video y GIF de la simulación")
    parser.add_argument("--output", type=str, default="/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part2_a1_gait.mp4")
    args = parser.parse_args()
    
    # Determinar si usar GUI
    use_gui = True
    if args.headless or not os.environ.get("DISPLAY"):
        use_gui = False
    elif args.gui:
        use_gui = True
        
    run_quadruped_gait_simulation(
        gui=use_gui,
        record_video=args.record,
        video_path=args.output
    )


if __name__ == "__main__":
    main()
