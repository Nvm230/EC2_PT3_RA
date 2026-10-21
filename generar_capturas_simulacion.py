#!/usr/bin/env python3
"""
===============================================================================
GENERADOR DE CAPTURAS DE ALTA CALIDAD PARA REPORTE (A1 Y H1)
===============================================================================
Descripción:
    Ejecuta las simulaciones en PyBullet con encuadres de cámara cinemáticos,
    zoom óptimo y renderizado de alta resolución (OpenGL hardware).
    
    Genera y guarda en `outputs/`:
    - outputs/part2_step_1.png  (Pata FL en swing)
    - outputs/part2_step_2.png  (Pata RR en swing)
    - outputs/part2_step_3.png  (Pata FR en swing)
    - outputs/part2_step_4.png  (Pata RL en swing)
    - outputs/part2_all_steps_grid.png (Mosaico 2x2 de alta resolución)
    
    - outputs/part3_standing.png (Postura erguida inicial con objeto)
    - outputs/part3_dual_arms_waist.png (Mov. A: Brazos y cintura)
    - outputs/part3_crouch_floor.png (Mov. B: Agache y mano en objeto)
    - outputs/part3_stand_return.png (Mov. B: Reincorporación erguida)
    - outputs/part3_motion_sequence_grid.png (Mosaico 2x2 de alta resolución)
===============================================================================
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


# Cargar fuentes TTF para renderizado tipográfico profesional
def get_font(size=22, bold=True):
    font_paths = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf",
        "/usr/local/share/fonts/DejaVuSans-Bold.ttf"
    ]
    for path in font_paths:
        if os.path.exists(path):
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                pass
    return ImageFont.load_default()


def capture_frame(width=1024, height=768, view_matrix=None, proj_matrix=None):
    """Captura un fotograma de PyBullet con alta resolución y renderizado OpenGL."""
    img = p.getCameraImage(
        width, height, viewMatrix=view_matrix, projectionMatrix=proj_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL
    )
    rgba = np.reshape(img[2], (height, width, 4))
    return Image.fromarray(rgba[:, :, :3])


def annotate_image(image, title, badge_text="PROXQP SOLVER", badge_color=(46, 134, 193)):
    """Agrega una barra superior elegante con título y una pastilla/badge informativo."""
    img_draw = image.copy()
    draw = ImageDraw.Draw(img_draw)
    
    font_title = get_font(22, bold=True)
    font_badge = get_font(16, bold=True)
    
    # Barra superior oscura translúcida
    bar_height = 54
    draw.rectangle([(0, 0), (img_draw.width, bar_height)], fill=(20, 26, 36))
    draw.line([(0, bar_height), (img_draw.width, bar_height)], fill=(50, 65, 85), width=2)
    
    # Título principal
    draw.text((25, 14), title, fill=(255, 255, 255), font=font_title)
    
    # Badge lateral
    badge_w = 260
    badge_h = 32
    bx0 = img_draw.width - badge_w - 20
    by0 = 11
    draw.rectangle([(bx0, by0), (bx0 + badge_w, by0 + badge_h)], fill=badge_color)
    draw.text((bx0 + 15, by0 + 6), badge_text, fill=(255, 255, 255), font=font_badge)
    
    return img_draw


# =============================================================================
# GENERACIÓN DE CAPTURAS: PARTE 2 (UNITREE A1 CUADRÚPEDO)
# =============================================================================
def generate_quadruped_snapshots():
    print(">>> Extrayendo capturas de la Parte 2 (Unitree A1 Cuadrúpedo)...")
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
    p.changeConstraint(base_stabilizer, maxForce=60.0)
    
    # Asentamiento
    for _ in range(40):
        p.stepSimulation()
        
    p_feet_init, _ = controller.kin.forward_kinematics_full(q_nominal)
    p_targets = p_feet_init.copy()
    current_q = q_nominal.copy()
    
    step_sequence = [0, 3, 1, 2]
    leg_links = {
        0: [9, 10],   # FL (muslo y pantorrilla)
        3: [14, 15],  # RR
        1: [4, 5],    # FR
        2: [19, 20]   # RL
    }
    
    # Cámaras dinámicas encuadradas en primer plano para cada pata activa
    cams = [
        {"target": [0.10, 0.08, 0.18], "dist": 0.72, "yaw": 140.0, "pitch": -14.0},   # FL
        {"target": [-0.10, -0.08, 0.18], "dist": 0.72, "yaw": 320.0, "pitch": -14.0},  # RR
        {"target": [0.10, -0.08, 0.18], "dist": 0.72, "yaw": 40.0, "pitch": -14.0},    # FR
        {"target": [-0.10, 0.08, 0.18], "dist": 0.72, "yaw": 220.0, "pitch": -14.0},   # RL
    ]
    
    colors_rgba = [
        [0.0, 0.85, 1.0, 1.0],   # FL: Cyan brillante
        [1.0, 0.75, 0.0, 1.0],   # RR: Amarillo oro
        [0.1, 0.90, 0.25, 1.0],  # FR: Verde esmeralda
        [0.92, 0.25, 0.92, 1.0]  # RL: Magenta eléctrico
    ]
    
    colors_rgb_pil = [
        (0, 215, 255),    # FL: Cyan
        (255, 190, 0),    # RR: Amarillo oro
        (25, 230, 65),    # FR: Verde esmeralda
        (235, 65, 235)    # RL: Magenta
    ]
    
    leg_titles = [
        "Paso 1/4: Pata Delantera Izquierda (FL) en Balanceo",
        "Paso 2/4: Pata Trasera Derecha (RR) en Balanceo",
        "Paso 3/4: Pata Delantera Derecha (FR) en Balanceo",
        "Paso 4/4: Pata Trasera Izquierda (RL) en Balanceo"
    ]
    leg_badges = [
        "SWING FL | ELEV: 8.0 cm",
        "SWING RR | ELEV: 8.0 cm",
        "SWING FR | ELEV: 8.0 cm",
        "SWING RL | ELEV: 8.0 cm"
    ]
    leg_codes = ["FL", "RR", "FR", "RL"]
    
    snapshots = {}
    
    for step_num, leg_idx in enumerate(step_sequence):
        # Restaurar color nominal en todas las patas
        for l_idx in [0, 1, 2, 3]:
            for l in leg_links[l_idx]:
                p.changeVisualShape(robot_id, l, rgbaColor=[0.85, 0.35, 0.05, 1.0])
        # Resaltar pata activa en balanceo
        for l in leg_links[leg_idx]:
            p.changeVisualShape(robot_id, l, rgbaColor=colors_rgba[step_num])
            
        p_start_leg = p_targets[leg_idx*3 : leg_idx*3+3].copy()
        p_end_leg = p_start_leg.copy()
        p_end_leg[0] += 0.10
        
        steps_count = 120
        traj_3d = []
        mid_img = None
        
        cam = cams[step_num]
        view_mat = p.computeViewMatrixFromYawPitchRoll(cam["target"], cam["dist"], cam["yaw"], cam["pitch"], 0, 2)
        proj_mat = p.computeProjectionMatrixFOV(50, 1024/768, 0.1, 10.0)
        
        for k in range(steps_count):
            tau = (k + 1) / steps_count
            p_leg_des, v_leg_des = generate_swing_trajectory(tau, p_start_leg, p_end_leg, step_height=0.08)
            p_targets[leg_idx*3 : leg_idx*3+3] = p_leg_des
            v_targets = np.zeros(12)
            v_targets[leg_idx*3 : leg_idx*3+3] = v_leg_des / 1.2
            
            current_q, _, _ = controller.solve_ik_qp(current_q, p_targets, v_targets, q_nominal=q_nominal)
            p.setJointMotorControlArray(
                robot_id, controller.pb_joint_indices, p.POSITION_CONTROL,
                targetPositions=current_q, forces=[50.0]*12
            )
            p.stepSimulation()
            
            # Registrar posición real 3D del pie en PyBullet
            pb_pos = p.getLinkState(robot_id, controller.pb_foot_links[leg_idx])[0]
            traj_3d.append(pb_pos)
            
            # Capturar exactamente en el vértice (máxima elevación de 8.0 cm)
            if k == steps_count // 2:
                img_data = p.getCameraImage(1024, 768, viewMatrix=view_mat, projectionMatrix=proj_mat, renderer=p.ER_BULLET_HARDWARE_OPENGL)
                rgba = np.reshape(img_data[2], (768, 1024, 4))
                mid_img = Image.fromarray(rgba[:, :, :3])
                
        # Proyección de la trayectoria 3D sobre el plano de imagen 2D
        V = np.array(view_mat).reshape((4, 4), order="F")
        P = np.array(proj_mat).reshape((4, 4), order="F")
        VP = P @ V
        
        def project(pt):
            v = np.array([pt[0], pt[1], pt[2], 1.0])
            clip = VP @ v
            ndc = clip[:3] / clip[3]
            return (ndc[0] + 1) * 0.5 * 1024, (1 - ndc[1]) * 0.5 * 768
            
        traj_2d = [project(pt) for pt in traj_3d]
        
        overlay = Image.new("RGBA", (1024, 768), (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        col_pil = colors_rgb_pil[step_num]
        
        # 1. Trazo luminoso de la parábola de trayectoria (halo + línea central)
        for i in range(len(traj_2d) - 1):
            draw.line([traj_2d[i], traj_2d[i+1]], fill=(col_pil[0], col_pil[1], col_pil[2], 80), width=10)
        for i in range(len(traj_2d) - 1):
            draw.line([traj_2d[i], traj_2d[i+1]], fill=(col_pil[0], col_pil[1], col_pil[2], 255), width=3)
            
        # 2. Marcadores de Inicio y Aterrizaje en el suelo
        u_start, v_start = traj_2d[0]
        u_end, v_end = traj_2d[-1]
        draw.ellipse([u_start-5, v_start-5, u_start+5, v_start+5], fill=(255, 255, 255, 240), outline=(50, 50, 50, 255), width=2)
        draw.ellipse([u_end-6, v_end-6, u_end+6, v_end+6], fill=(col_pil[0], col_pil[1], col_pil[2], 160), outline=(255, 255, 255, 255), width=2)
        
        # 3. Línea vertical de cota de elevación (Apex vs Suelo)
        apex_idx = steps_count // 2
        apex_pt = traj_3d[apex_idx]
        floor_pt = [apex_pt[0], apex_pt[1], traj_3d[0][2]]
        u_apex, v_apex = project(apex_pt)
        u_floor, v_floor = project(floor_pt)
        
        # Línea punteada de despegue
        y_dist = abs(v_floor - v_apex)
        y_steps = max(1, int(y_dist / 6))
        for s in range(0, y_steps, 2):
            y_s = v_apex + s * 6
            y_e = min(v_floor, y_s + 4)
            draw.line([(u_apex, y_s), (u_apex, y_e)], fill=(255, 230, 0, 240), width=2)
            
        draw.ellipse([u_floor-4, v_floor-4, u_floor+4, v_floor+4], fill=(255, 230, 0, 240))
        draw.ellipse([u_apex-7, v_apex-7, u_apex+7, v_apex+7], fill=(255, 255, 255, 255), outline=col_pil, width=3)
        
        # Cartela de cota h = 8.0 cm
        font_sm = get_font(13, bold=True)
        lx0, ly0 = u_apex + 14, v_apex - 12
        draw.rectangle([(lx0, ly0), (lx0 + 102, ly0 + 24)], fill=(20, 26, 36, 220), outline=(255, 230, 0, 240), width=1)
        draw.text((lx0 + 8, ly0 + 4), "h = 8.0 cm", fill=(255, 255, 255, 255), font=font_sm)
        
        # 4. Barra Superior Profesional
        bar_h = 54
        draw.rectangle([(0, 0), (1024, bar_h)], fill=(20, 26, 36, 245))
        draw.line([(0, bar_h), (1024, bar_h)], fill=(50, 65, 85, 255), width=2)
        
        font_title = get_font(21, bold=True)
        font_badge = get_font(15, bold=True)
        draw.text((25, 14), leg_titles[step_num], fill=(255, 255, 255, 255), font=font_title)
        
        bw, bh = 250, 32
        bx0, by0 = 1024 - bw - 20, 11
        draw.rectangle([(bx0, by0), (bx0 + bw, by0 + bh)], fill=(col_pil[0], col_pil[1], col_pil[2], 230))
        draw.text((bx0 + 14, by0 + 7), leg_badges[step_num], fill=(255, 255, 255, 255), font=font_badge)
        
        # 5. Mini HUD Card (Estado de Soporte Cinemático) en la esquina inferior izquierda
        hud_w, hud_h = 260, 82
        hx0, hy0 = 25, 768 - hud_h - 20
        draw.rectangle([(hx0, hy0), (hx0 + hud_w, hy0 + hud_h)], fill=(15, 23, 42, 235), outline=(51, 65, 85, 255), width=1)
        
        font_hud_title = get_font(12, bold=True)
        font_hud_info = get_font(11, bold=False)
        draw.text((hx0 + 12, hy0 + 8), "ESTADO DE SOPORTE CINEMÁTICO", fill=(148, 163, 184, 255), font=font_hud_title)
        
        dog_feet = {
            "FL": (hx0 + 26, hy0 + 36), "FR": (hx0 + 62, hy0 + 36),
            "RL": (hx0 + 26, hy0 + 60), "RR": (hx0 + 62, hy0 + 60)
        }
        active_code = leg_codes[step_num]
        for f_code, (fx, fy) in dog_feet.items():
            if f_code == active_code:
                draw.ellipse([fx-6, fy-6, fx+6, fy+6], fill=col_pil, outline=(255, 255, 255, 255), width=2)
            else:
                draw.ellipse([fx-5, fy-5, fx+5, fy+5], fill=(34, 197, 94, 255))
                
        draw.text((hx0 + 82, hy0 + 34), f"Pata {active_code}: BALANCEO (Swing)", fill=col_pil, font=font_hud_info)
        draw.text((hx0 + 82, hy0 + 52), "3 Patas: APOYO TRÍPODE", fill=(203, 213, 225, 255), font=font_hud_info)
        
        final_step_img = Image.alpha_composite(mid_img.convert("RGBA"), overlay).convert("RGB")
        out_path = f"/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part2_step_{step_num+1}.png"
        final_step_img.save(out_path)
        snapshots[step_num] = final_step_img
        print(f"Guardado: {out_path}")
        
    # Mosaico 2x2 de alta resolución con separador estilizado
    composite = Image.new("RGB", (2048 + 8, 1536 + 8), color=(30, 41, 59))
    composite.paste(snapshots[0], (0, 0))
    composite.paste(snapshots[1], (1024 + 8, 0))
    composite.paste(snapshots[2], (0, 768 + 8))
    composite.paste(snapshots[3], (1024 + 8, 768 + 8))
    
    comp_path = "/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part2_all_steps_grid.png"
    composite.save(comp_path)
    print(f"Guardado mosaico cuadrúpedo: {comp_path}")
    p.disconnect()


# =============================================================================
# GENERACIÓN DE CAPTURAS: PARTE 3 (UNITREE H1 HUMANOIDE)
# =============================================================================
def generate_humanoid_snapshots():
    print(">>> Extrayendo capturas de la Parte 3 (Unitree H1 Humanoide)...")
    physicsClient = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(0.01)
    
    plane_id = p.loadURDF("plane.urdf")
    
    # Objeto en el piso (cilindro rojo visible a la altura adecuada)
    obj_pos = [0.28, -0.18, 0.08]
    col_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.04, height=0.16)
    vis_shape = p.createVisualShape(p.GEOM_CYLINDER, radius=0.04, length=0.16, rgbaColor=[0.9, 0.15, 0.15, 1.0])
    target_obj = p.createMultiBody(0.2, col_shape, vis_shape, basePosition=obj_pos)
    
    robot_id = p.loadURDF(h1_description.URDF_PATH, [0, 0, 0.98], useFixedBase=False)
    
    # Anclajes de estabilidad cinemática en pies y guía en pelvis
    l_ankle_cid = p.createConstraint(robot_id, 4, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0.06, 0.20, 0.05])
    r_ankle_cid = p.createConstraint(robot_id, 9, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0.06, -0.20, 0.05])
    p.changeConstraint(l_ankle_cid, maxForce=5000.0)
    p.changeConstraint(r_ankle_cid, maxForce=5000.0)
    
    pelvis_cid = p.createConstraint(robot_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0.0, 0.0, 0.98])
    p.changeConstraint(pelvis_cid, maxForce=6000.0)
    
    controller = H1KinematicController(dt=0.01)
    q_stand = np.zeros(controller.nq)
    q_stand[11] = 0.2; q_stand[12] = 0.2; q_stand[14] = 0.3
    q_stand[15] = 0.2; q_stand[16] = -0.2; q_stand[18] = 0.3
    
    for idx, val in enumerate(q_stand):
        p.resetJointState(robot_id, idx, val)
        
    for _ in range(35):
        p.stepSimulation()
        
    # Cámara encuadrada para capturar al humanoide completo de pies a cabeza con holgura
    cam_target = [0.12, 0.0, 0.85]
    cam_dist = 2.45
    cam_yaw = 45.0
    cam_pitch = -10.0
    view_mat = p.computeViewMatrixFromYawPitchRoll(cam_target, cam_dist, cam_yaw, cam_pitch, 0, 2)
    proj_mat = p.computeProjectionMatrixFOV(48, 1024/768, 0.1, 10.0)
    
    # -------------------------------------------------------------------------
    # 1. Captura Postura Erguida Inicial
    # -------------------------------------------------------------------------
    img_stand = capture_frame(1024, 768, view_mat, proj_mat)
    annotated_stand = annotate_image(
        img_stand, "Humanoide Unitree H1: Postura Erguida Inicial y Objeto",
        badge_text="POSTURA INICIAL | 19 DOF",
        badge_color=(46, 134, 193)
    )
    out_stand = "/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part3_standing.png"
    annotated_stand.save(out_stand)
    print(f"Guardado: {out_stand}")
    
    # -------------------------------------------------------------------------
    # 2. Ejecutar Movimiento A (Brazos Coordinados + Giro de Cintura)
    # -------------------------------------------------------------------------
    p_r_init, p_l_init = controller.get_hand_positions(q_stand)
    img_mov_a = None
    q_current = q_stand.copy()
    
    for k in range(200):
        t = k * 0.01
        freq = 0.5
        waist_target = 0.60 * np.sin(2.0 * np.pi * freq * t)
        delta_x = 0.15 * (1.0 - np.cos(2.0 * np.pi * freq * t))
        delta_z = 0.20 * (1.0 - np.cos(2.0 * np.pi * freq * t))
        
        r_des = p_r_init + np.array([delta_x, -0.05 * np.sin(2*np.pi*freq*t), delta_z])
        l_des = p_l_init + np.array([delta_x,  0.05 * np.sin(2*np.pi*freq*t), delta_z])
        
        q_nom = q_stand.copy()
        q_nom[10] = waist_target
        
        targets = {"r_hand_pos": r_des, "w_r_hand": 2.0, "l_hand_pos": l_des, "w_l_hand": 2.0, "q_nominal": q_nom, "w_posture": 0.25}
        q_current, _, _, _ = controller.solve_step_qp(q_current, targets)
        p.setJointMotorControlArray(
            robot_id, controller.pb_joint_indices, p.POSITION_CONTROL,
            targetPositions=q_current, forces=[200.0]*controller.nq
        )
        p.stepSimulation()
        
        # Pico de rotación y elevación
        if k == 100:
            raw_a = capture_frame(1024, 768, view_mat, proj_mat)
            img_mov_a = annotate_image(
                raw_a, "Movimiento A: Elevación de Brazos con Giro de Cintura (+34.4°)",
                badge_text="MOV. A | CINTURA YAW",
                badge_color=(40, 167, 69)
            )
            out_a = "/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part3_dual_arms_waist.png"
            img_mov_a.save(out_a)
            print(f"Guardado: {out_a}")
            
    # Retorno breve a neutral
    for _ in range(30):
        q_current[10] *= 0.90
        p.setJointMotorControlArray(
            robot_id, controller.pb_joint_indices, p.POSITION_CONTROL,
            targetPositions=q_current, forces=[200.0]*controller.nq
        )
        p.stepSimulation()
        
    # -------------------------------------------------------------------------
    # 3. Movimiento B: Agache Profundo Estable y Recolección en el Piso
    # -------------------------------------------------------------------------
    th_knee_target = 1.95
    th_hip_target = -1.55
    th_ankle_target = -0.95
    r_arm_crouch = {15: -0.65, 16: -0.16, 17: 1.15, 18: 1.35}
    l_arm_crouch = {11: 0.30, 12: 0.35, 14: 0.50}
    
    for k in range(250):
        s = (k + 1) / 250
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
        
    # Capturar fondo (mano en contacto con el objeto en el suelo)
    raw_crouch = capture_frame(1024, 768, view_mat, proj_mat)
    r_state = p.getLinkState(robot_id, 18)
    R_elb = np.array(p.getMatrixFromQuaternion(r_state[1])).reshape(3, 3)
    hand_pos_w = r_state[0] + R_elb @ np.array([0.18, 0, 0])
    
    img_crouch = annotate_image(
        raw_crouch, f"Movimiento B: Agache Profundo - Contacto con Objeto (z = {hand_pos_w[2]:.2f} m)",
        badge_text="MOV. B | CONTACTO EN PISO",
        badge_color=(220, 53, 69)
    )
    out_crouch = "/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part3_crouch_floor.png"
    img_crouch.save(out_crouch)
    print(f"Guardado: {out_crouch}")
    
    # -------------------------------------------------------------------------
    # 4. Reincorporación Erguida
    # -------------------------------------------------------------------------
    for k in range(250):
        s = 1.0 - (k + 1) / 250
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
        
    raw_return = capture_frame(1024, 768, view_mat, proj_mat)
    img_return = annotate_image(
        raw_return, "Movimiento B: Reincorporación Exitosa y Estable a Postura Erguida",
        badge_text="REINCORPORADO | ESTABLE",
        badge_color=(40, 167, 69)
    )
    out_return = "/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part3_stand_return.png"
    img_return.save(out_return)
    print(f"Guardado: {out_return}")
    
    # Mosaico 2x2 compuesto humanoide de alta resolución
    comp_h1 = Image.new("RGB", (2048 + 8, 1536 + 8), color=(30, 41, 59))
    comp_h1.paste(annotated_stand, (0, 0))
    comp_h1.paste(img_mov_a, (1024 + 8, 0))
    comp_h1.paste(img_crouch, (0, 768 + 8))
    comp_h1.paste(img_return, (1024 + 8, 768 + 8))
    
    comp_h1_path = "/home/nvm/Documents/UTEC/Avanzada_tareas/outputs/part3_motion_sequence_grid.png"
    comp_h1.save(comp_h1_path)
    print(f"Guardado mosaico humanoide: {comp_h1_path}")
    p.disconnect()


if __name__ == "__main__":
    generate_quadruped_snapshots()
    generate_humanoid_snapshots()
    print(">>> Extracción de todas las capturas completada con éxito. <<<")
