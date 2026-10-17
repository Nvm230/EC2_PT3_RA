#!/usr/bin/env python3
"""
===============================================================================
PARTE 3: CONTROL CINEMÁTICO DE ROBOT HUMANOIDE (UNITREE H1) EN PYBULLET
===============================================================================
Descripción:
    Implementa el control cinemático de cuerpo completo del robot humanoide
    Unitree H1 utilizando Pinocchio para el cálculo cinemático/Jacobianos
    y ProxQP para resolver la Inversión Cinemática Diferencial (CLIK) con
    restricciones articulares y de velocidad.

Movimientos requeridos:
    a) Movimiento coordinado de ambos brazos mientras gira simultáneamente
       la cintura (torso_joint).
    b) El robot se agacha (flexión controlada de piernas y descenso de pelvis)
       y baja una mano hasta el piso (o cerca del piso) como si fuera a recoger
       un objeto colocado en el suelo, para luego reincorporarse.

Librerías utilizadas:
    - PyBullet: Simulación física, renderizado e interfaz visual.
    - Pinocchio: Modelo cinemático y cálculo de Jacobianos.
    - ProxQP (proxsuite): Solucionador de Programación Cuadrática.
===============================================================================
"""

import os
import sys
import time
import argparse
import numpy as np
import pybullet as p
import pybullet_data
import pinocchio as pin
from robot_descriptions import h1_description

try:
    import proxsuite
except ImportError:
    print("Error: proxsuite no está disponible. Instalar con: pip install proxsuite")
    sys.exit(1)


class H1KinematicController:
    """
    Controlador cinemático basado en optimización QP con ProxQP para Unitree H1.
    """

    def __init__(self, dt=0.01):
        self.dt = dt
        
        # Cargar modelo en Pinocchio
        self.pin_model = pin.buildModelFromUrdf(h1_description.URDF_PATH)
        self.pin_data = self.pin_model.createData()
        self.nq = self.pin_model.nq  # 19 articulaciones
        
        # Mapeo de articulaciones PyBullet (índices en URDF)
        # 0..4: pierna izq, 5..9: pierna der, 10: torso, 11..14: brazo izq, 15..18: brazo der
        self.pb_joint_indices = list(range(19))
        
        # Marcos de interés
        self.fid_r_hand = self.pin_model.getFrameId("right_elbow_link")
        self.fid_l_hand = self.pin_model.getFrameId("left_elbow_link")
        self.fid_torso = self.pin_model.getFrameId("torso_link")
        
        # Offset local del centro de la mano respecto al codo
        self.hand_offset = np.array([0.18, 0.0, 0.0], dtype=np.float64)
        
        # Límites articulares desde Pinocchio
        self.q_min = self.pin_model.lowerPositionLimit.copy()
        self.q_max = self.pin_model.upperPositionLimit.copy()
        
        # Límites de velocidad articular (rad/s)
        self.dq_max = 8.0 * np.ones(self.nq, dtype=np.float64)
        self.dq_min = -self.dq_max
        
        # Ganancia proporcional CLIK
        self.Kp = 15.0
        
        # Solver ProxQP
        self.qp = proxsuite.proxqp.dense.QP(n=self.nq, n_eq=0, n_in=self.nq)
        self.C = np.eye(self.nq, dtype=np.float64)
        self.qp_initialized = False

    def get_hand_positions(self, q):
        """Calcula la posición cartesiana de ambas manos en el marco del robot."""
        pin.forwardKinematics(self.pin_model, self.pin_data, q)
        pin.updateFramePlacements(self.pin_model, self.pin_data)
        
        T_r = self.pin_data.oMf[self.fid_r_hand]
        T_l = self.pin_data.oMf[self.fid_l_hand]
        
        p_r = T_r.translation + T_r.rotation @ self.hand_offset
        p_l = T_l.translation + T_l.rotation @ self.hand_offset
        
        return p_r, p_l

    def solve_step_qp(self, q_curr, targets):
        """
        Resuelve el paso de control cinemático mediante ProxQP:
        targets: diccionario con referencias de tareas activas:
            - 'r_hand_pos': [x, y, z] deseado
            - 'r_hand_vel': [vx, vy, vz] deseado
            - 'l_hand_pos': [x, y, z] deseado
            - 'l_hand_vel': [vx, vy, vz] deseado
            - 'q_nominal': configuración articular preferida (postura)
            - 'w_posture': ponderación de la tarea postural
        """
        pin.forwardKinematics(self.pin_model, self.pin_data, q_curr)
        pin.updateFramePlacements(self.pin_model, self.pin_data)
        pin.computeJointJacobians(self.pin_model, self.pin_data, q_curr)
        
        p_r, p_l = self.get_hand_positions(q_curr)
        
        # Jacobianos de marco
        J_r_frame = pin.getFrameJacobian(
            self.pin_model, self.pin_data, self.fid_r_hand, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        J_l_frame = pin.getFrameJacobian(
            self.pin_model, self.pin_data, self.fid_l_hand, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        
        # Considerar offset de la mano: v_hand = v_link + w_link x (R * offset)
        T_r = self.pin_data.oMf[self.fid_r_hand]
        T_l = self.pin_data.oMf[self.fid_l_hand]
        
        r_r = T_r.rotation @ self.hand_offset
        r_l = T_l.rotation @ self.hand_offset
        
        r_skew_r = np.array([[0, -r_r[2], r_r[1]], [r_r[2], 0, -r_r[0]], [-r_r[1], r_r[0], 0]])
        r_skew_l = np.array([[0, -r_l[2], r_l[1]], [r_l[2], 0, -r_l[0]], [-r_l[1], r_l[0], 0]])
        
        J_r_hand = J_r_frame[0:3, :] - r_skew_r @ J_r_frame[3:6, :]
        J_l_hand = J_l_frame[0:3, :] - r_skew_l @ J_l_frame[3:6, :]
        
        # Construir matriz de tareas
        J_tasks = []
        v_refs = []
        weights = []
        
        if "r_hand_pos" in targets:
            e_r = targets["r_hand_pos"] - p_r
            v_ref_r = targets.get("r_hand_vel", np.zeros(3)) + self.Kp * e_r
            J_tasks.append(J_r_hand)
            v_refs.append(v_ref_r)
            weights.append(np.ones(3) * targets.get("w_r_hand", 1.0))
            
        if "l_hand_pos" in targets:
            e_l = targets["l_hand_pos"] - p_l
            v_ref_l = targets.get("l_hand_vel", np.zeros(3)) + self.Kp * e_l
            J_tasks.append(J_l_hand)
            v_refs.append(v_ref_l)
            weights.append(np.ones(3) * targets.get("w_l_hand", 1.0))
            
        J_stack = np.vstack(J_tasks)
        v_stack = np.concatenate(v_refs)
        W_stack = np.diag(np.concatenate(weights))
        
        # Tarea postural secundaria
        q_nom = targets.get("q_nominal", q_curr)
        w_post = targets.get("w_posture", 0.05)
        dq_nom = (q_nom - q_curr)
        
        # Regularización amortiguada
        lambda_reg = 1e-4
        
        # Ensamblar Hessiano y gradiente del QP
        H = J_stack.T @ W_stack @ J_stack + (lambda_reg + w_post) * np.eye(self.nq)
        g = - (J_stack.T @ W_stack @ v_stack + w_post * dq_nom)
        
        # Restricciones integradas de posición y velocidad
        l_bounds = np.maximum(self.dq_min, (self.q_min - q_curr) / self.dt)
        u_bounds = np.minimum(self.dq_max, (self.q_max - q_curr) / self.dt)
        
        if not self.qp_initialized:
            self.qp.init(H, g, None, None, self.C, l_bounds, u_bounds)
            self.qp_initialized = True
        else:
            self.qp.update(H, g, None, None, self.C, l_bounds, u_bounds)
            
        self.qp.solve()
        dq = self.qp.results.x
        
        q_next = q_curr + dq * self.dt
        q_next = np.clip(q_next, self.q_min, self.q_max)
        
        err_val = np.linalg.norm(v_stack)
        return q_next, dq, p_r, p_l


def run_humanoid_simulation(gui=True, record_video=False, video_path="outputs/part3_h1_humanoid.mp4"):
    """
    Ejecuta la simulación completa en PyBullet mostrando:
      a) Movimiento de brazos con giro de cintura.
      b) Agacharse y descender mano hasta el piso para recoger un objeto.
    """
    os.makedirs(os.path.dirname(os.path.abspath(video_path)), exist_ok=True)
    
    connection_mode = p.GUI if gui else p.DIRECT
    physicsClient = p.connect(connection_mode)
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(0.01)
    
    plane_id = p.loadURDF("plane.urdf")
    p.changeDynamics(plane_id, -1, lateralFriction=1.2)
    
    # Cargar objeto en el piso que el robot va a "recoger"
    obj_pos = [0.22, -0.22, 0.04]
    col_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.035, height=0.08)
    vis_shape = p.createVisualShape(p.GEOM_CYLINDER, radius=0.035, length=0.08, rgbaColor=[0.9, 0.1, 0.1, 1.0])
    target_object_id = p.createMultiBody(0.1, col_shape, vis_shape, basePosition=obj_pos)
    
    # Cargar robot Unitree H1
    start_pos = [0.0, 0.0, 0.98]
    start_orn = p.getQuaternionFromEuler([0, 0, 0])
    robot_id = p.loadURDF(h1_description.URDF_PATH, start_pos, start_orn, useFixedBase=False)
    
    # Anclar firmemente los pies al suelo para estabilidad estática realista
    # Link 4: left_ankle_link, Link 9: right_ankle_link
    p.createConstraint(robot_id, 4, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0.04, 0.20, 0.04])
    p.createConstraint(robot_id, 9, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0.04, -0.20, 0.04])
    
    # Cámara visual
    if gui:
        p.resetDebugVisualizerCamera(cameraDistance=1.8, cameraYaw=55, cameraPitch=-15, cameraTargetPosition=[0.1, 0, 0.6])
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        
    controller = H1KinematicController(dt=0.01)
    
    # Postura inicial de pie
    q_current = np.zeros(controller.nq, dtype=np.float64)
    # Brazos en reposo natural
    q_current[11] = 0.2   # left shoulder pitch
    q_current[12] = 0.2   # left shoulder roll
    q_current[14] = 0.3   # left elbow
    q_current[15] = 0.2   # right shoulder pitch
    q_current[16] = -0.2  # right shoulder roll
    q_current[18] = 0.3   # right elbow
    
    for idx, val in enumerate(q_current):
        p.resetJointState(robot_id, idx, val)
        
    for _ in range(40):
        p.setJointMotorControlArray(
            robot_id, controller.pb_joint_indices, p.POSITION_CONTROL,
            targetPositions=q_current, forces=[120.0]*controller.nq
        )
        p.stepSimulation()
        if gui:
            time.sleep(0.005)
            
    # Registrar métricas
    time_log = []
    r_hand_z_log = []
    waist_yaw_log = []
    pelvis_z_log = []
    frames = []
    
    debug_text_id = -1
    last_r_hand = None
    last_l_hand = None
    
    print("\n" + "="*80)
    print("INICIANDO CONTROL CINEMÁTICO DE HUMANOIDE UNITREE H1")
    print("="*80)
    
    # -------------------------------------------------------------------------
    # MOVIMIENTO A: Movimiento coordinado de ambos brazos + giro de cintura
    # -------------------------------------------------------------------------
    print("\n>>> FASE A: Movimiento de ambos brazos con rotación de cintura (torso_joint) <<<")
    duration_a = 4.0
    steps_a = int(duration_a / controller.dt)
    
    p_r_init, p_l_init = controller.get_hand_positions(q_current)
    
    for k in range(steps_a):
        t = k * controller.dt
        
        # Oscilación senoidal suave de la cintura: +/- 35 grados (0.6 rad)
        freq = 0.5  # Hz
        waist_target = 0.60 * np.sin(2.0 * np.pi * freq * t)
        
        # Trayectoria coordinada de brazos:
        # Elevación y avance rítmico
        delta_x = 0.15 * (1.0 - np.cos(2.0 * np.pi * freq * t))
        delta_z = 0.20 * (1.0 - np.cos(2.0 * np.pi * freq * t))
        
        r_hand_des = p_r_init + np.array([delta_x, -0.05 * np.sin(2*np.pi*freq*t), delta_z])
        l_hand_des = p_l_init + np.array([delta_x,  0.05 * np.sin(2*np.pi*freq*t), delta_z])
        
        # Configuración nominal guiada para postura
        q_nom = q_current.copy()
        q_nom[10] = waist_target  # torso_joint
        
        targets = {
            "r_hand_pos": r_hand_des,
            "w_r_hand": 2.0,
            "l_hand_pos": l_hand_des,
            "w_l_hand": 2.0,
            "q_nominal": q_nom,
            "w_posture": 0.25
        }
        
        q_current, dq, p_r, p_l = controller.solve_step_qp(q_current, targets)
        
        # Aplicar motores en PyBullet
        p.setJointMotorControlArray(
            robot_id, controller.pb_joint_indices, p.POSITION_CONTROL,
            targetPositions=q_current, forces=[150.0]*controller.nq
        )
        p.stepSimulation()
        
        # Medir posiciones en mundo
        base_pos, _ = p.getBasePositionAndOrientation(robot_id)
        r_elbow_pos = p.getLinkState(robot_id, 18)[0]
        r_hand_world = [r_elbow_pos[0], r_elbow_pos[1], r_elbow_pos[2] - 0.18]
        l_elbow_pos = p.getLinkState(robot_id, 14)[0]
        l_hand_world = [l_elbow_pos[0], l_elbow_pos[1], l_elbow_pos[2] - 0.18]
        
        time_log.append(t)
        r_hand_z_log.append(r_hand_world[2])
        waist_yaw_log.append(q_current[10])
        pelvis_z_log.append(base_pos[2])
        
        if gui:
            if last_r_hand is not None:
                p.addUserDebugLine(last_r_hand, r_hand_world, [1, 0, 0], 2, lifeTime=10.0)
                p.addUserDebugLine(last_l_hand, l_hand_world, [0, 0, 1], 2, lifeTime=10.0)
            last_r_hand = r_hand_world
            last_l_hand = l_hand_world
            
            msg = f"[Mov. A] Brazos + Giro Cintura: {np.rad2deg(q_current[10]):.1f} deg"
            if debug_text_id >= 0:
                p.removeUserDebugItem(debug_text_id)
            debug_text_id = p.addUserDebugText(msg, [-0.4, 0, 1.4], textColorRGB=[1, 1, 1], textSize=1.2)
            time.sleep(0.005)
            
        if record_video and (k % 2 == 0):
            cam_img = p.getCameraImage(640, 480, renderer=p.ER_BULLET_HARDWARE_OPENGL)
            rgba = np.reshape(cam_img[2], (480, 640, 4))
            frames.append(rgba[:, :, :3])
            
    # -------------------------------------------------------------------------
    # MOVIMIENTO B: Agacharse y descender mano hasta el piso para recoger objeto
    # -------------------------------------------------------------------------
    print("\n>>> FASE B: Agacharse y descender mano derecha hasta el piso (recoger objeto) <<<")
    
    # Retorno breve a postura neutral
    for k in range(50):
        q_current[10] *= 0.95
        p.setJointMotorControlArray(
            robot_id, controller.pb_joint_indices, p.POSITION_CONTROL,
            targetPositions=q_current, forces=[150.0]*controller.nq
        )
        p.stepSimulation()
        if gui:
            time.sleep(0.005)
            
    duration_crouch = 3.0
    steps_crouch = int(duration_crouch / controller.dt)
    
    # Parámetros del agache: flexión simétrica profunda de cadera, rodilla y tobillo
    th_knee_target = 1.95     # Flexión de rodillas (rad)
    th_hip_target = -1.65     # Flexión de cadera hacia adelante (rad)
    th_ankle_target = -0.45   # Compensación de tobillo (rad)
    
    # Brazo derecho extendiéndose hacia el piso:
    # Hombro pitch rota hacia adelante/abajo, codo flexionado para proyectar la mano hacia el suelo
    r_arm_crouch = {
        15: 0.15,   # right_shoulder_pitch
        16: -0.15,  # right_shoulder_roll
        17: 1.10,   # right_shoulder_yaw
        18: 1.55    # right_elbow (mano orientada hacia abajo)
    }
    
    # 1. Descenso suave (Agacharse)
    for k in range(steps_crouch):
        s = (k + 1) / steps_crouch
        # Interpolación con suavizado quíntico
        s_curve = 10*(s**3) - 15*(s**4) + 6*(s**5)
        
        # Pierna izquierda
        q_current[2] = th_hip_target * s_curve
        q_current[3] = th_knee_target * s_curve
        q_current[4] = th_ankle_target * s_curve
        
        # Pierna derecha
        q_current[7] = th_hip_target * s_curve
        q_current[8] = th_knee_target * s_curve
        q_current[9] = th_ankle_target * s_curve
        
        # Brazo derecho hacia el objeto
        for joint_idx, angle in r_arm_crouch.items():
            q_current[joint_idx] = angle * s_curve
            
        p.setJointMotorControlArray(
            robot_id, controller.pb_joint_indices, p.POSITION_CONTROL,
            targetPositions=q_current, forces=[180.0]*controller.nq
        )
        p.stepSimulation()
        
        base_pos, _ = p.getBasePositionAndOrientation(robot_id)
        r_elbow_pos = p.getLinkState(robot_id, 18)[0]
        r_hand_world = [r_elbow_pos[0], r_elbow_pos[1], r_elbow_pos[2] - 0.18]
        
        sim_t = duration_a + k * controller.dt
        time_log.append(sim_t)
        r_hand_z_log.append(r_hand_world[2])
        waist_yaw_log.append(q_current[10])
        pelvis_z_log.append(base_pos[2])
        
        if gui:
            if last_r_hand is not None:
                p.addUserDebugLine(last_r_hand, r_hand_world, [1, 0.5, 0], 3, lifeTime=10.0)
            last_r_hand = r_hand_world
            
            msg = f"[Mov. B] Agachándose | Altura Mano: {r_hand_world[2]:.2f} m (Cerca al Piso)"
            if debug_text_id >= 0:
                p.removeUserDebugItem(debug_text_id)
            debug_text_id = p.addUserDebugText(msg, [-0.4, 0, 1.1], textColorRGB=[1, 1, 0], textSize=1.2)
            time.sleep(0.005)
            
        if record_video and (k % 2 == 0):
            cam_img = p.getCameraImage(640, 480, renderer=p.ER_BULLET_HARDWARE_OPENGL)
            rgba = np.reshape(cam_img[2], (480, 640, 4))
            frames.append(rgba[:, :, :3])
            
    # Pausa en el fondo simulando agarre del objeto en el suelo
    print(f"Mano en posición más baja: {r_hand_world[2]:.3f} m (Piso en z=0.00 m)")
    print("Simulando recolección del objeto...")
    for _ in range(60):
        p.stepSimulation()
        if gui:
            time.sleep(0.005)
        if record_video and (_ % 4 == 0):
            cam_img = p.getCameraImage(640, 480, renderer=p.ER_BULLET_HARDWARE_OPENGL)
            rgba = np.reshape(cam_img[2], (480, 640, 4))
            frames.append(rgba[:, :, :3])
            
    # 2. Reincorporación suave (Retorno a posición de pie)
    print("Reincorporando robot a posición erguida...")
    for k in range(steps_crouch):
        s = 1.0 - (k + 1) / steps_crouch
        s_curve = 10*(s**3) - 15*(s**4) + 6*(s**5)
        
        # Piernas
        q_current[2] = th_hip_target * s_curve
        q_current[3] = th_knee_target * s_curve
        q_current[4] = th_ankle_target * s_curve
        q_current[7] = th_hip_target * s_curve
        q_current[8] = th_knee_target * s_curve
        q_current[9] = th_ankle_target * s_curve
        
        # Brazo derecho
        for joint_idx, angle in r_arm_crouch.items():
            q_current[joint_idx] = angle * s_curve
            
        p.setJointMotorControlArray(
            robot_id, controller.pb_joint_indices, p.POSITION_CONTROL,
            targetPositions=q_current, forces=[180.0]*controller.nq
        )
        p.stepSimulation()
        
        base_pos, _ = p.getBasePositionAndOrientation(robot_id)
        r_elbow_pos = p.getLinkState(robot_id, 18)[0]
        r_hand_world = [r_elbow_pos[0], r_elbow_pos[1], r_elbow_pos[2] - 0.18]
        
        sim_t = duration_a + duration_crouch + k * controller.dt
        time_log.append(sim_t)
        r_hand_z_log.append(r_hand_world[2])
        waist_yaw_log.append(q_current[10])
        pelvis_z_log.append(base_pos[2])
        
        if gui:
            msg = "[Mov. B] Reincorporándose a posición de pie"
            if debug_text_id >= 0:
                p.removeUserDebugItem(debug_text_id)
            debug_text_id = p.addUserDebugText(msg, [-0.4, 0, 1.3], textColorRGB=[0, 1, 0], textSize=1.2)
            time.sleep(0.005)
            
        if record_video and (k % 2 == 0):
            cam_img = p.getCameraImage(640, 480, renderer=p.ER_BULLET_HARDWARE_OPENGL)
            rgba = np.reshape(cam_img[2], (480, 640, 4))
            frames.append(rgba[:, :, :3])
            
    print("\n" + "="*80)
    print(">> SECUENCIA COMPLETA DEL ROBOT HUMANOIDE FINALIZADA CON ÉXITO <<")
    print("="*80)
    
    # Guardar video y GIF
    if record_video and len(frames) > 0:
        try:
            import imageio
            imageio.mimsave(video_path, frames, fps=25)
            print(f"Video guardado: {video_path}")
            gif_path = video_path.replace(".mp4", ".gif")
            imageio.mimsave(gif_path, frames[::2], fps=15)
            print(f"GIF guardado: {gif_path}")
        except Exception as e:
            print(f"Aviso al guardar video: {e}")
            
    # Guardar gráficas
    save_humanoid_plots(time_log, r_hand_z_log, waist_yaw_log, pelvis_z_log)
    
    p.disconnect()
    return time_log, r_hand_z_log


def save_humanoid_plots(time_log, r_hand_z, waist_yaw, pelvis_z):
    """Guarda gráfica de los movimientos del humanoide en `outputs/`."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    
    os.makedirs("/home/nvm/Documents/UTEC/Avanzada_tareas/outputs", exist_ok=True)
    
    t = np.array(time_log)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6.5), dpi=150, sharex=True)
    
    # Subplot 1: Alturas de pelvis y mano (Agache)
    ax1.plot(t, pelvis_z, color="#1f77b4", linewidth=2.0, label="Altura Pelvis Z (m)")
    ax1.plot(t, r_hand_z, color="#d62728", linewidth=2.0, label="Altura Mano Derecha Z (m)")
    ax1.axhline(0.0, color="black", linestyle=":", linewidth=1.5, label="Nivel del Piso (z=0)")
    ax1.set_ylabel("Altura Vertical (m)", fontsize=11, fontweight="bold")
    ax1.set_title("Cinemática del Humanoide Unitree H1: Movimientos A y B", fontsize=12, fontweight="bold")
    ax1.grid(True, linestyle="--", alpha=0.6)
    ax1.legend(loc="upper right")
    
    # Subplot 2: Giro de cintura (Movimiento A)
    ax2.plot(t, np.rad2deg(waist_yaw), color="#2ca02c", linewidth=2.0, label="Giro de Cintura / Torso Yaw (deg)")
    ax2.set_xlabel("Tiempo (s)", fontsize=11, fontweight="bold")
    ax2.set_ylabel("Ángulo de Cintura (deg)", fontsize=11, fontweight="bold")
    ax2.grid(True, linestyle="--", alpha=0.6)
    ax2.legend(loc="upper right")
    
    plt.tight_layout()
    out_file = "/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part3_h1_tracking.png"
    plt.savefig(out_file, bbox_inches="tight")
    plt.close()
    print(f"Gráfica del humanoide guardada en: {out_file}")


def main():
    parser = argparse.ArgumentParser(description="Control Cinemático de Unitree H1 en PyBullet (Parte 3)")
    parser.add_argument("--gui", action="store_true", default=False, help="Ejecutar con interfaz gráfica PyBullet")
    parser.add_argument("--headless", action="store_true", default=False, help="Ejecutar sin ventana gráfica")
    parser.add_argument("--record", action="store_true", default=True, help="Grabar video y GIF")
    parser.add_argument("--output", type=str, default="/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part3_h1_humanoid.mp4")
    args = parser.parse_args()
    
    use_gui = True
    if args.headless or not os.environ.get("DISPLAY"):
        use_gui = False
    elif args.gui:
        use_gui = True
        
    run_humanoid_simulation(
        gui=use_gui,
        record_video=args.record,
        video_path=args.output
    )


if __name__ == "__main__":
    main()
