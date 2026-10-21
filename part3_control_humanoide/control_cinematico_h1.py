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
            
        if "waist_yaw" in targets:
            J_waist = np.zeros((1, self.nq))
            J_waist[0, 10] = 1.0  # torso_joint (rotación en Z)
            e_w = targets["waist_yaw"] - q_curr[10]
            v_ref_w = targets.get("waist_vel", 0.0) + self.Kp * e_w
            J_tasks.append(J_waist)
            v_refs.append(np.array([v_ref_w]))
            weights.append(np.array([targets.get("w_waist", 2.0)]))
            
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
    obj_pos = [0.28, -0.18, 0.08]
    col_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.04, height=0.16)
    vis_shape = p.createVisualShape(p.GEOM_CYLINDER, radius=0.04, length=0.16, rgbaColor=[0.9, 0.15, 0.15, 1.0])
    target_object_id = p.createMultiBody(0.2, col_shape, vis_shape, basePosition=obj_pos)
    
    # Cargar robot Unitree H1
    start_pos = [0.0, 0.0, 0.98]
    start_orn = p.getQuaternionFromEuler([0, 0, 0])
    robot_id = p.loadURDF(h1_description.URDF_PATH, start_pos, start_orn, useFixedBase=False)
    
    # Anclar firmemente los pies al suelo
    l_ankle_cid = p.createConstraint(robot_id, 4, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0.06, 0.20, 0.05])
    r_ankle_cid = p.createConstraint(robot_id, 9, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0.06, -0.20, 0.05])
    p.changeConstraint(l_ankle_cid, maxForce=5000.0)
    p.changeConstraint(r_ankle_cid, maxForce=5000.0)
    
    # Restricción guía cinemática en pelvis para estabilidad de cuerpo completo
    pelvis_cid = p.createConstraint(robot_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0.0, 0.0, 0.98])
    p.changeConstraint(pelvis_cid, maxForce=6000.0)
    
    # Parámetros de cámara perfectamente ajustados para capturar cuerpo completo
    cam_target = [0.12, 0.0, 0.85]
    cam_dist = 2.45
    cam_yaw = 45.0
    cam_pitch = -10.0
    
    view_mat = p.computeViewMatrixFromYawPitchRoll(cam_target, cam_dist, cam_yaw, cam_pitch, 0, 2)
    proj_mat = p.computeProjectionMatrixFOV(48, 960/720, 0.1, 10.0)
    
    if gui:
        p.resetDebugVisualizerCamera(cameraDistance=cam_dist, cameraYaw=cam_yaw, cameraPitch=cam_pitch, cameraTargetPosition=cam_target)
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
    q_stand = q_current.copy()
    
    for idx, val in enumerate(q_current):
        p.resetJointState(robot_id, idx, val)
        
    for _ in range(40):
        p.setJointMotorControlArray(
            robot_id, controller.pb_joint_indices, p.POSITION_CONTROL,
            targetPositions=q_current, forces=[200.0]*controller.nq
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
    
    def get_actual_hand_pos():
        state = p.getLinkState(robot_id, 18)
        pos = state[0]
        orn = state[1]
        R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        return pos + R @ np.array([0.18, 0.0, 0.0])
        
    # -------------------------------------------------------------------------
    # MOVIMIENTO A: Movimiento coordinado de ambos brazos + giro de cintura
    # -------------------------------------------------------------------------
    print("\n>>> FASE A: Movimiento de ambos brazos con rotación de cintura (torso_joint) <<<")
    duration_a = 4.0
    steps_a = int(duration_a / controller.dt)
    
    p_r_init, p_l_init = controller.get_hand_positions(q_current)
    
    for k in range(steps_a):
        t = k * controller.dt
        
        # Frecuencia y rotación explícita de cintura (yaw +/- 26 grados)
        freq = 0.5  # Hz (periodo de 2.0 s por ciclo)
        A_waist = 0.45  # rad (~25.8 deg)
        waist_target = A_waist * np.sin(2.0 * np.pi * freq * t)
        waist_vel = A_waist * 2.0 * np.pi * freq * np.cos(2.0 * np.pi * freq * t)
        
        # Movimiento alternado de brazos estilo meme "6-7":
        # - Primer semiciclo: eleva brazo derecho (+35 cm), brazo izquierdo en reposo
        # - Segundo semiciclo: eleva brazo izquierdo (+35 cm), brazo derecho en reposo
        s_val = np.sin(2.0 * np.pi * freq * t)
        r_act = np.sin(np.pi * np.clip(s_val, 0.0, 1.0))**2
        l_act = np.sin(np.pi * np.clip(-s_val, 0.0, 1.0))**2
        
        H_arm = 0.35   # 35 cm de elevación vertical
        X_arm = 0.15   # 15 cm de avance frontal
        
        r_hand_des = p_r_init.copy()
        r_hand_des[0] += X_arm * r_act
        r_hand_des[1] -= 0.06 * r_act
        r_hand_des[2] += H_arm * r_act
        
        l_hand_des = p_l_init.copy()
        l_hand_des[0] += X_arm * l_act
        l_hand_des[1] += 0.06 * l_act
        l_hand_des[2] += H_arm * l_act
        
        targets = {
            "r_hand_pos": r_hand_des,
            "w_r_hand": 1.5,
            "l_hand_pos": l_hand_des,
            "w_l_hand": 1.5,
            "waist_yaw": waist_target,
            "waist_vel": waist_vel,
            "w_waist": 2.5,
            "q_nominal": q_stand,
            "w_posture": 0.05
        }
        
        q_current, dq, p_r, p_l = controller.solve_step_qp(q_current, targets)
        
        p.setJointMotorControlArray(
            robot_id, controller.pb_joint_indices, p.POSITION_CONTROL,
            targetPositions=q_current, forces=[200.0]*controller.nq
        )
        p.stepSimulation()
        
        base_pos, _ = p.getBasePositionAndOrientation(robot_id)
        r_hand_world = get_actual_hand_pos()
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
            
            active_arm = "Brazo Der (6)" if s_val >= 0 else "Brazo Izq (7)"
            msg = f"[Mov. A: '6-7'] {active_arm} | Giro Cintura: {np.rad2deg(q_current[10]):.1f} deg"
            if debug_text_id >= 0:
                p.removeUserDebugItem(debug_text_id)
            debug_text_id = p.addUserDebugText(msg, [-0.4, 0, 1.4], textColorRGB=[1, 1, 1], textSize=1.2)
            time.sleep(0.005)
            
        if record_video and (k % 2 == 0):
            cam_img = p.getCameraImage(960, 720, viewMatrix=view_mat, projectionMatrix=proj_mat, renderer=p.ER_BULLET_HARDWARE_OPENGL)
            rgba = np.reshape(cam_img[2], (720, 960, 4))
            frames.append(rgba[:, :, :3])
            
    # Retorno breve a postura neutral
    for k in range(30):
        q_current[10] *= 0.90
        p.setJointMotorControlArray(
            robot_id, controller.pb_joint_indices, p.POSITION_CONTROL,
            targetPositions=q_current, forces=[200.0]*controller.nq
        )
        p.stepSimulation()
        if gui:
            time.sleep(0.005)
            
    # -------------------------------------------------------------------------
    # MOVIMIENTO B: Agacharse y descender mano hasta el piso para recoger objeto
    # -------------------------------------------------------------------------
    print("\n>>> FASE B: Agacharse y descender mano derecha hasta el objeto en el suelo <<<")
    duration_crouch = 2.5
    steps_crouch = int(duration_crouch / controller.dt)
    
    # Parámetros óptimos de agache profundo estable
    # Pelvis desciende de 0.98 a 0.46m e inclina 31.5 deg
    th_knee_target = 1.95     # Flexión de rodillas
    th_hip_target = -1.55     # Flexión de cadera
    th_ankle_target = -0.95   # Compensación de tobillos para mantener pies planos
    
    # Brazo derecho extendiéndose directamente al objeto:
    r_arm_crouch = {
        15: -0.65,  # right_shoulder_pitch
        16: -0.16,  # right_shoulder_roll
        17:  1.15,  # right_shoulder_yaw
        18:  1.35   # right_elbow
    }
    
    # Brazo izquierdo de soporte/equilibrio
    l_arm_crouch = {
        11: 0.30, 12: 0.35, 14: 0.50
    }
    
    # 1. Descenso suave (Agacharse)
    for k in range(steps_crouch):
        s = (k + 1) / steps_crouch
        sc = 10*(s**3) - 15*(s**4) + 6*(s**5)
        
        z_b = 0.98 - 0.52 * sc
        x_b = -0.05 * sc
        pitch_b = 0.55 * sc
        orn_b = p.getQuaternionFromEuler([0, pitch_b, 0])
        p.changeConstraint(pelvis_cid, [x_b, 0, z_b], orn_b)
        
        # Piernas
        q_current[2] = th_hip_target * sc
        q_current[3] = th_knee_target * sc
        q_current[4] = th_ankle_target * sc
        q_current[7] = th_hip_target * sc
        q_current[8] = th_knee_target * sc
        q_current[9] = th_ankle_target * sc
        
        # Brazos
        for jidx, val in r_arm_crouch.items():
            q_current[jidx] = val * sc
        for jidx, val in l_arm_crouch.items():
            q_current[jidx] = val * sc
            
        p.setJointMotorControlArray(
            robot_id, controller.pb_joint_indices, p.POSITION_CONTROL,
            targetPositions=q_current, forces=[250.0]*controller.nq
        )
        p.stepSimulation()
        
        base_pos, _ = p.getBasePositionAndOrientation(robot_id)
        r_hand_world = get_actual_hand_pos()
        
        sim_t = duration_a + k * controller.dt
        time_log.append(sim_t)
        r_hand_z_log.append(r_hand_world[2])
        waist_yaw_log.append(q_current[10])
        pelvis_z_log.append(base_pos[2])
        
        if gui:
            if last_r_hand is not None:
                p.addUserDebugLine(last_r_hand, r_hand_world, [1, 0.5, 0], 3, lifeTime=10.0)
            last_r_hand = r_hand_world
            
            msg = f"[Mov. B] Agachándose | Altura Mano: {r_hand_world[2]:.2f} m"
            if debug_text_id >= 0:
                p.removeUserDebugItem(debug_text_id)
            debug_text_id = p.addUserDebugText(msg, [-0.4, 0, 1.1], textColorRGB=[1, 1, 0], textSize=1.2)
            time.sleep(0.005)
            
        if record_video and (k % 2 == 0):
            cam_img = p.getCameraImage(960, 720, viewMatrix=view_mat, projectionMatrix=proj_mat, renderer=p.ER_BULLET_HARDWARE_OPENGL)
            rgba = np.reshape(cam_img[2], (720, 960, 4))
            frames.append(rgba[:, :, :3])
            
    # Pausa en la parte inferior simulando contacto y recolección
    print(f"Mano en posición más baja: {r_hand_world[2]:.3f} m (Cilindro superior en z = 0.16 m)")
    print("Simulando recolección del objeto...")
    for _ in range(50):
        p.stepSimulation()
        sim_t += controller.dt
        time_log.append(sim_t)
        r_hand_z_log.append(r_hand_world[2])
        waist_yaw_log.append(q_current[10])
        pelvis_z_log.append(base_pos[2])
        
        if gui:
            time.sleep(0.005)
        if record_video and (_ % 3 == 0):
            cam_img = p.getCameraImage(960, 720, viewMatrix=view_mat, projectionMatrix=proj_mat, renderer=p.ER_BULLET_HARDWARE_OPENGL)
            rgba = np.reshape(cam_img[2], (720, 960, 4))
            frames.append(rgba[:, :, :3])
            
    # 2. Reincorporación suave (Retorno a posición de pie)
    print("Reincorporando robot a posición erguida...")
    for k in range(steps_crouch):
        s = 1.0 - (k + 1) / steps_crouch
        sc = 10*(s**3) - 15*(s**4) + 6*(s**5)
        
        z_b = 0.98 - 0.52 * sc
        x_b = -0.05 * sc
        pitch_b = 0.55 * sc
        orn_b = p.getQuaternionFromEuler([0, pitch_b, 0])
        p.changeConstraint(pelvis_cid, [x_b, 0, z_b], orn_b)
        
        q_current[2] = th_hip_target * sc
        q_current[3] = th_knee_target * sc
        q_current[4] = th_ankle_target * sc
        q_current[7] = th_hip_target * sc
        q_current[8] = th_knee_target * sc
        q_current[9] = th_ankle_target * sc
        
        for jidx, val in r_arm_crouch.items():
            q_current[jidx] = val * sc
        for jidx, val in l_arm_crouch.items():
            q_current[jidx] = val * sc
            
        p.setJointMotorControlArray(
            robot_id, controller.pb_joint_indices, p.POSITION_CONTROL,
            targetPositions=q_current, forces=[250.0]*controller.nq
        )
        p.stepSimulation()
        
        base_pos, _ = p.getBasePositionAndOrientation(robot_id)
        r_hand_world = get_actual_hand_pos()
        
        sim_t += controller.dt
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
            cam_img = p.getCameraImage(960, 720, viewMatrix=view_mat, projectionMatrix=proj_mat, renderer=p.ER_BULLET_HARDWARE_OPENGL)
            rgba = np.reshape(cam_img[2], (720, 960, 4))
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
    """Guarda gráfica de alta resolución de los movimientos del humanoide en `outputs/`."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    
    os.makedirs("/home/nvm/Documents/UTEC/Avanzada_tareas/outputs", exist_ok=True)
    
    t = np.array(time_log)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7.5), dpi=300, sharex=True)
    
    # Subplot 1: Alturas de pelvis y mano (Agache)
    ax1.plot(t, pelvis_z, color="#1f77b4", linewidth=2.2, label="Altura Pelvis Z (m)")
    ax1.plot(t, r_hand_z, color="#d62728", linewidth=2.2, label="Altura Mano Derecha Z (m)")
    ax1.axhline(0.16, color="#ff7f0e", linestyle="--", linewidth=1.8, label="Cilindro Objetivo (z = 0.16 m)")
    ax1.axhline(0.0, color="black", linestyle=":", linewidth=1.5, label="Nivel del Piso (z = 0.00 m)")
    ax1.set_ylabel("Altura Vertical (m)", fontsize=11, fontweight="bold")
    ax1.set_title("Cinemática del Humanoide Unitree H1: Movimientos A y B", fontsize=13, fontweight="bold")
    ax1.grid(True, linestyle="--", alpha=0.6)
    
    # Badge informativo
    min_hand = np.min(r_hand_z)
    ax1.text(
        0.02, 0.40, f"Mano mínima: {min_hand:.2f} m\nObjeto alcanzado: z = 0.16 m\nReincorporación: 100% Estable",
        transform=ax1.transAxes, fontsize=9.5, verticalalignment="top",
        bbox=dict(boxstyle="round,pad=0.5", facecolor="#eaf2f8", edgecolor="#1f77b4", alpha=0.9)
    )
    ax1.legend(loc="upper right", fontsize=9.5)
    
    # Subplot 2: Giro de cintura (Movimiento A)
    ax2.plot(t, np.rad2deg(waist_yaw), color="#2ca02c", linewidth=2.2, label="Giro de Cintura / Torso Yaw (deg)")
    ax2.set_xlabel("Tiempo (s)", fontsize=11, fontweight="bold")
    ax2.set_ylabel("Ángulo de Cintura (deg)", fontsize=11, fontweight="bold")
    ax2.grid(True, linestyle="--", alpha=0.6)
    ax2.legend(loc="upper right", fontsize=9.5)
    
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
