#!/usr/bin/env python3
"""
Script para ejecutar simulaciones y extraer capturas de alta calidad
para incluir tanto en REPORTE.md como en el reporte LaTeX (PDF).
"""

import os
import sys
import numpy as np
import pybullet as p
import pybullet_data
from PIL import Image, ImageDraw, ImageFont
import pinocchio as pin
from robot_descriptions import a1_description, h1_description

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "part1_cinematica_cuadrupedo")))
from cinematica_directa_jacobiano import A1KinematicsManual

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "part2_control_cuadrupedo")))
from control_cinematico_a1 import A1KinematicController, generate_swing_trajectory

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "part3_control_humanoide")))
from control_cinematico_h1 import H1KinematicController


def capture_frame(width=1024, height=768, view_matrix=None, proj_matrix=None):
    """Captura un fotograma de PyBullet con alta resolución."""
    if view_matrix is None or proj_matrix is None:
        img = p.getCameraImage(width, height, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    else:
        img = p.getCameraImage(width, height, viewMatrix=view_matrix, projectionMatrix=proj_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    rgba = np.reshape(img[2], (height, width, 4))
    return Image.fromarray(rgba[:, :, :3])


def annotate_image(image, text):
    """Agrega un banner explicativo semi-transparente en la parte superior."""
    img_draw = image.copy()
    draw = ImageDraw.Draw(img_draw)
    
    # Dibujar rectángulo de fondo para el texto
    draw.rectangle([(0, 0), (img_draw.width, 40)], fill=(20, 25, 35))
    draw.text((20, 10), text, fill=(255, 255, 255))
    return img_draw


def generate_quadruped_snapshots():
    print(">>> Extrayendo capturas de la Parte 2 (Unitree A1)...")
    physicsClient = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(0.01)
    
    plane_id = p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(a1_description.URDF_PATH, [0, 0, 0.32], useFixedBase=False)
    
    controller = A1KinematicController(dt=0.01)
    q_nominal = np.array([0.0, 0.75, -1.50] * 4, dtype=np.float64)
    for idx, val in zip(controller.pb_joint_indices, q_nominal):
        p.resetJointState(robot_id, idx, val)
        
    base_stabilizer = p.createConstraint(robot_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0.31])
    p.changeConstraint(base_stabilizer, maxForce=35.0)
    
    # Asentamiento
    for _ in range(30):
        p.stepSimulation()
        
    view_mat = p.computeViewMatrixFromYawPitchRoll([0.15, 0, 0.25], 1.25, 45, -25, 0, 2)
    proj_mat = p.computeProjectionMatrixFOV(60, 1024/768, 0.1, 10.0)
    
    p_feet_init, _ = controller.kin.forward_kinematics_full(q_nominal)
    p_targets = p_feet_init.copy()
    current_q = q_nominal.copy()
    
    step_sequence = [0, 3, 1, 2]
    leg_names = ["FL (Delantera Izq)", "RR (Trasera Der)", "FR (Delantera Der)", "RL (Trasera Izq)"]
    leg_colors = {0: [1, 0, 0], 3: [1, 0.5, 0], 1: [0, 1, 0], 2: [0, 0, 1]}
    
    snapshots = {}
    last_pt = None
    
    for step_num, leg_idx in enumerate(step_sequence):
        p_start_leg = p_targets[leg_idx*3 : leg_idx*3+3].copy()
        p_end_leg = p_start_leg.copy()
        p_end_leg[0] += 0.08
        
        steps_count = 120
        for k in range(steps_count):
            tau = (k + 1) / steps_count
            p_leg_des, v_leg_des = generate_swing_trajectory(tau, p_start_leg, p_end_leg, step_height=0.055)
            p_targets[leg_idx*3 : leg_idx*3+3] = p_leg_des
            v_targets = np.zeros(12)
            v_targets[leg_idx*3 : leg_idx*3+3] = v_leg_des / 1.2
            
            current_q, _, _ = controller.solve_ik_qp(current_q, p_targets, v_targets, q_nominal=q_nominal)
            p.setJointMotorControlArray(robot_id, controller.pb_joint_indices, p.POSITION_CONTROL, targetPositions=current_q, forces=[45.0]*12)
            p.stepSimulation()
            
            # Dibujar traza
            foot_pos = p.getLinkState(robot_id, controller.pb_foot_links[leg_idx])[0]
            if last_pt is not None:
                p.addUserDebugLine(last_pt, foot_pos, leg_colors[leg_idx], 3, lifeTime=0)
            last_pt = foot_pos
            
            # Capturar en mitad del paso (máxima elevación)
            if k == steps_count // 2:
                img = capture_frame(1024, 768, view_mat, proj_mat)
                annotated = annotate_image(img, f"Paso {step_num+1}/4: Pata {leg_names[step_num]} en Elevacion (Swing)")
                out_path = f"/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part2_step_{step_num+1}.png"
                annotated.save(out_path)
                snapshots[step_num] = annotated
                print(f"Guardado: {out_path}")
                
        last_pt = None
        for _ in range(15):
            p.stepSimulation()
            
    # Crear imagen compuesta de los 4 pasos (grid 2x2)
    composite = Image.new("RGB", (2048, 1536))
    composite.paste(snapshots[0], (0, 0))
    composite.paste(snapshots[1], (1024, 0))
    composite.paste(snapshots[2], (0, 768))
    composite.paste(snapshots[3], (1024, 768))
    comp_path = "/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part2_all_steps_grid.png"
    composite.save(comp_path)
    print(f"Guardado composite: {comp_path}")
    
    p.disconnect()


def generate_humanoid_snapshots():
    print(">>> Extrayendo capturas de la Parte 3 (Unitree H1)...")
    physicsClient = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(0.01)
    
    plane_id = p.loadURDF("plane.urdf")
    
    # Objeto en el piso
    obj_pos = [0.22, -0.22, 0.04]
    col_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.035, height=0.08)
    vis_shape = p.createVisualShape(p.GEOM_CYLINDER, radius=0.035, length=0.08, rgbaColor=[0.9, 0.1, 0.1, 1.0])
    target_obj = p.createMultiBody(0.1, col_shape, vis_shape, basePosition=obj_pos)
    
    robot_id = p.loadURDF(h1_description.URDF_PATH, [0, 0, 0.98], useFixedBase=False)
    p.createConstraint(robot_id, 4, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0.04, 0.20, 0.04])
    p.createConstraint(robot_id, 9, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0.04, -0.20, 0.04])
    
    controller = H1KinematicController(dt=0.01)
    q_current = np.zeros(controller.nq)
    q_current[11] = 0.2; q_current[12] = 0.2; q_current[14] = 0.3
    q_current[15] = 0.2; q_current[16] = -0.2; q_current[18] = 0.3
    
    for idx, val in enumerate(q_current):
        p.resetJointState(robot_id, idx, val)
        
    for _ in range(30):
        p.stepSimulation()
        
    view_mat = p.computeViewMatrixFromYawPitchRoll([0.15, 0, 0.55], 1.7, 50, -18, 0, 2)
    proj_mat = p.computeProjectionMatrixFOV(60, 1024/768, 0.1, 10.0)
    
    # 1. Captura Postura Inicial Erguida
    img_stand = capture_frame(1024, 768, view_mat, proj_mat)
    annotated_stand = annotate_image(img_stand, "Humanoide Unitree H1: Postura Erguida Inicial y Objeto Objetivo")
    out_stand = "/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part3_standing.png"
    annotated_stand.save(out_stand)
    print(f"Guardado: {out_stand}")
    
    # 2. Ejecutar Movimiento A (Brazos + Cintura)
    p_r_init, p_l_init = controller.get_hand_positions(q_current)
    img_mov_a = None
    
    for k in range(200):
        t = k * 0.01
        freq = 0.5
        waist_target = 0.60 * np.sin(2.0 * np.pi * freq * t)
        delta_x = 0.15 * (1.0 - np.cos(2.0 * np.pi * freq * t))
        delta_z = 0.20 * (1.0 - np.cos(2.0 * np.pi * freq * t))
        
        r_des = p_r_init + np.array([delta_x, -0.05 * np.sin(2*np.pi*freq*t), delta_z])
        l_des = p_l_init + np.array([delta_x,  0.05 * np.sin(2*np.pi*freq*t), delta_z])
        
        q_nom = q_current.copy()
        q_nom[10] = waist_target
        
        targets = {"r_hand_pos": r_des, "w_r_hand": 2.0, "l_hand_pos": l_des, "w_l_hand": 2.0, "q_nominal": q_nom, "w_posture": 0.25}
        q_current, _, _, _ = controller.solve_step_qp(q_current, targets)
        p.setJointMotorControlArray(robot_id, controller.pb_joint_indices, p.POSITION_CONTROL, targetPositions=q_current, forces=[150.0]*controller.nq)
        p.stepSimulation()
        
        if k == 100:  # Pico de rotación y elevación
            img = capture_frame(1024, 768, view_mat, proj_mat)
            img_mov_a = annotate_image(img, "Movimiento A: Elevacion de Brazos Coordinada con Giro de Cintura (+34.4 deg)")
            out_a = "/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part3_dual_arms_waist.png"
            img_mov_a.save(out_a)
            print(f"Guardado: {out_a}")
            
    # Retorno a neutral
    for _ in range(40):
        q_current[10] *= 0.95
        p.setJointMotorControlArray(robot_id, controller.pb_joint_indices, p.POSITION_CONTROL, targetPositions=q_current, forces=[150.0]*controller.nq)
        p.stepSimulation()
        
    # 3. Movimiento B: Agache profundo y mano al piso
    th_knee_target = 1.95
    th_hip_target = -1.65
    th_ankle_target = -0.45
    r_arm_crouch = {15: 0.15, 16: -0.15, 17: 1.10, 18: 1.55}
    
    img_crouch = None
    for k in range(300):
        s = (k + 1) / 300
        s_curve = 10*(s**3) - 15*(s**4) + 6*(s**5)
        
        q_current[2] = th_hip_target * s_curve; q_current[3] = th_knee_target * s_curve; q_current[4] = th_ankle_target * s_curve
        q_current[7] = th_hip_target * s_curve; q_current[8] = th_knee_target * s_curve; q_current[9] = th_ankle_target * s_curve
        
        for jidx, val in r_arm_crouch.items():
            q_current[jidx] = val * s_curve
            
        p.setJointMotorControlArray(robot_id, controller.pb_joint_indices, p.POSITION_CONTROL, targetPositions=q_current, forces=[180.0]*controller.nq)
        p.stepSimulation()
        
    # Capturar fondo (mano tocando objeto en piso)
    img = capture_frame(1024, 768, view_mat, proj_mat)
    r_elbow_pos = p.getLinkState(robot_id, 18)[0]
    hand_z = r_elbow_pos[2] - 0.18
    img_crouch = annotate_image(img, f"Movimiento B: Agache Profundo - Mano en el Piso (z = {hand_z:.2f} m) Recogiendo Objeto")
    out_crouch = "/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part3_crouch_floor.png"
    img_crouch.save(out_crouch)
    print(f"Guardado: {out_crouch}")
    
    # 4. Reincorporación
    img_return = None
    for k in range(300):
        s = 1.0 - (k + 1) / 300
        s_curve = 10*(s**3) - 15*(s**4) + 6*(s**5)
        q_current[2] = th_hip_target * s_curve; q_current[3] = th_knee_target * s_curve; q_current[4] = th_ankle_target * s_curve
        q_current[7] = th_hip_target * s_curve; q_current[8] = th_knee_target * s_curve; q_current[9] = th_ankle_target * s_curve
        for jidx, val in r_arm_crouch.items():
            q_current[jidx] = val * s_curve
        p.setJointMotorControlArray(robot_id, controller.pb_joint_indices, p.POSITION_CONTROL, targetPositions=q_current, forces=[180.0]*controller.nq)
        p.stepSimulation()
        
    img = capture_frame(1024, 768, view_mat, proj_mat)
    img_return = annotate_image(img, "Movimiento B: Reincorporacion Exitosa a Postura de Pie tras la Recoleccion")
    out_return = "/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part3_stand_return.png"
    img_return.save(out_return)
    print(f"Guardado: {out_return}")
    
    # Imagen compuesta humanoide (grid 2x2)
    comp_h1 = Image.new("RGB", (2048, 1536))
    comp_h1.paste(annotated_stand, (0, 0))
    comp_h1.paste(img_mov_a, (1024, 0))
    comp_h1.paste(img_crouch, (0, 768))
    comp_h1.paste(img_return, (1024, 768))
    comp_h1_path = "/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part3_motion_sequence_grid.png"
    comp_h1.save(comp_h1_path)
    print(f"Guardado composite humanoide: {comp_h1_path}")
    
    p.disconnect()


if __name__ == "__main__":
    generate_quadruped_snapshots()
    generate_humanoid_snapshots()
    print(">>> Extracción de todas las capturas completada con éxito. <<<")
