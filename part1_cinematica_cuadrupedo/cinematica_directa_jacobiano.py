#!/usr/bin/env python3
"""
===============================================================================
PARTE 1: MODELADO CINEMÁTICO DIRECTO Y JACOBIANO DE CUERPO COMPLETO (UNITREE A1)
===============================================================================
Descripción:
    Este script implementa desde cero (primeros principios matemáticos y álgebra lineal
    pura usando únicamente NumPy) la cinemática directa (FK) y el Jacobiano geométrico
    de cuerpo completo del robot cuadrúpedo Unitree A1 (diferente de Laikago).
    
    Posteriormente, compara los resultados con la librería de referencia Pinocchio,
    evaluando la precisión numérica (norma del error y diferencias absolutas).

Robot Cuadrúpedo Seleccionado:
    - Unitree A1 (12 Grados de Libertad actuated: 3 DOF por pata x 4 patas)
    - FL: Front-Left (Pata delantera izquierda)
    - FR: Front-Right (Pata delantera derecha)
    - RL: Rear-Left (Pata trasera izquierda)
    - RR: Rear-Right (Pata trasera derecha)
===============================================================================
"""

import sys
import numpy as np
import pinocchio as pin
from robot_descriptions import a1_description


class A1KinematicsManual:
    """
    Implementación analítica / geométrica de la cinemática directa y el Jacobiano
    de cuerpo completo del robot cuadrúpedo Unitree A1 SIN usar librerías de cinemática.
    """

    def __init__(self):
        # Parámetros geométricos exactos del URDF de Unitree A1
        # Distancia longitudinal y lateral de la base a las articulaciones de cadera (m)
        self.d_x = 0.1805   # mitad de la longitud entre caderas
        self.d_y = 0.0470   # mitad del ancho entre hombros
        
        # Longitudes de los eslabones (m)
        self.l_thigh_lateral = 0.0838  # offset en Y desde cadera a muslo
        self.l_thigh = 0.2000          # longitud del fémur/muslo (en -Z)
        self.l_calf = 0.2000           # longitud de la espinilla/pantorrilla (en -Z)
        
        # Definición de las 4 patas: nombre, signos de posición (sx, sy), índice inicial en q (12-vector)
        # Orden en Pinocchio: FL (0:3), FR (3:6), RL (6:9), RR (9:12)
        self.legs_info = [
            {"name": "FL", "sx": +1.0, "sy": +1.0, "q_start": 0},
            {"name": "FR", "sx": +1.0, "sy": -1.0, "q_start": 3},
            {"name": "RL", "sx": -1.0, "sy": +1.0, "q_start": 6},
            {"name": "RR", "sx": -1.0, "sy": -1.0, "q_start": 9},
        ]

    @staticmethod
    def _rot_x(theta):
        """Matriz de rotación elemental 3x3 alrededor del eje X."""
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([
            [1.0, 0.0, 0.0],
            [0.0,   c,  -s],
            [0.0,   s,   c]
        ], dtype=np.float64)

    @staticmethod
    def _rot_y(theta):
        """Matriz de rotación elemental 3x3 alrededor del eje Y."""
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([
            [  c, 0.0,   s],
            [0.0, 1.0, 0.0],
            [ -s, 0.0,   c]
        ], dtype=np.float64)

    def forward_kinematics_leg(self, leg_idx, q_leg):
        """
        Calcula la cinemática directa de una pata individual respecto a la base del robot.
        
        Entradas:
            leg_idx: Índice de la pata (0: FL, 1: FR, 2: RL, 3: RR)
            q_leg: [q1, q2, q3] ángulos de (cadera roll, muslo pitch, pantorrilla pitch)
            
        Retorna:
            p_foot: Posición 3D del pie en el sistema de coordenadas del cuerpo (base)
            joint_positions: Diccionario con posiciones 3D de cada articulación (para Jacobiano)
            joint_axes: Diccionario con vectores unitarios 3D de cada eje de rotación
            R_foot: Matriz de rotación 3x3 de la orientación del pie
        """
        info = self.legs_info[leg_idx]
        sx = info["sx"]
        sy = info["sy"]
        
        q1, q2, q3 = q_leg[0], q_leg[1], q_leg[2]
        
        # 1. Base a Articulación de Cadera (Hip Joint)
        # Offset fijo desde el centro de la base al punto de rotación de la cadera
        p_hip = np.array([sx * self.d_x, sy * self.d_y, 0.0], dtype=np.float64)
        # El eje de rotación de la cadera (hip roll) es el eje X de la base
        z1 = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        R1 = self._rot_x(q1)
        
        # 2. Cadera a Articulación de Muslo (Thigh Joint)
        # Offset lateral en Y respecto al marco de la cadera
        p_thigh_offset_local = np.array([0.0, sy * self.l_thigh_lateral, 0.0], dtype=np.float64)
        p_thigh = p_hip + R1 @ p_thigh_offset_local
        # El eje de rotación del muslo (thigh pitch) rota alrededor de Y en el marco rotado por R1
        z2 = R1 @ np.array([0.0, 1.0, 0.0], dtype=np.float64)
        R2 = R1 @ self._rot_y(q2)
        
        # 3. Muslo a Articulación de Pantorrilla/Rodilla (Calf Joint)
        # Offset hacia abajo en Z (-l_thigh) respecto al marco del muslo
        p_calf_offset_local = np.array([0.0, 0.0, -self.l_thigh], dtype=np.float64)
        p_calf = p_thigh + R2 @ p_calf_offset_local
        # El eje de rotación de la pantorrilla (calf pitch) rota alrededor de Y en el marco rotado por R2
        z3 = R2 @ np.array([0.0, 1.0, 0.0], dtype=np.float64)
        R3 = R2 @ self._rot_y(q3)
        
        # 4. Pantorrilla a Pie (Foot Tip / Contact Point)
        p_foot_offset_local = np.array([0.0, 0.0, -self.l_calf], dtype=np.float64)
        p_foot = p_calf + R3 @ p_foot_offset_local
        
        joint_positions = {"hip": p_hip, "thigh": p_thigh, "calf": p_calf}
        joint_axes = {"z1": z1, "z2": z2, "z3": z3}
        
        return p_foot, joint_positions, joint_axes, R3

    def compute_leg_jacobian(self, leg_idx, q_leg):
        """
        Calcula el Jacobiano geométrico analítico de una pata (3x3 lineal, 3x3 angular).
        
        Fórmula geométrica por producto vectorial:
            J_v_i = z_i x (p_foot - p_i)   [Velocidad lineal]
            J_w_i = z_i                    [Velocidad angular]
        """
        p_foot, j_pos, j_axes, R_foot = self.forward_kinematics_leg(leg_idx, q_leg)
        
        z1, z2, z3 = j_axes["z1"], j_axes["z2"], j_axes["z3"]
        p1, p2, p3 = j_pos["hip"], j_pos["thigh"], j_pos["calf"]
        
        # Componentes lineales
        J_v1 = np.cross(z1, p_foot - p1)
        J_v2 = np.cross(z2, p_foot - p2)
        J_v3 = np.cross(z3, p_foot - p3)
        J_v = np.column_stack([J_v1, J_v2, J_v3])  # 3x3
        
        # Componentes angulares
        J_w = np.column_stack([z1, z2, z3])        # 3x3
        
        # Jacobiano 6x3 completo de la pata
        J_6d = np.vstack([J_v, J_w])
        
        return J_v, J_6d, p_foot

    def forward_kinematics_full(self, q):
        """
        Cinemática directa de cuerpo completo para las 4 patas.
        
        Entrada:
            q: vector de 12 elementos [q_FL (3), q_FR (3), q_RL (3), q_RR (3)]
        Retorna:
            p_feet: vector de 12 elementos con las posiciones [x,y,z] de los 4 pies
            dict_feet: diccionario {'FL': p_FL, 'FR': p_FR, 'RL': p_RL, 'RR': p_RR}
        """
        dict_feet = {}
        feet_list = []
        for i, info in enumerate(self.legs_info):
            q_start = info["q_start"]
            q_leg = q[q_start:q_start+3]
            p_foot, _, _, _ = self.forward_kinematics_leg(i, q_leg)
            dict_feet[info["name"]] = p_foot
            feet_list.append(p_foot)
            
        p_feet = np.concatenate(feet_list)
        return p_feet, dict_feet

    def full_body_jacobian(self, q):
        """
        Calcula el Jacobiano de cuerpo completo para las 4 patas respecto a la base.
        
        Al ser una estructura cinemática en árbol donde cada pata es una cadena
        independiente conectada a la base común, la matriz Jacobiana resultante
        es una matriz diagonal por bloques de dimensiones 12x12 (lineal) y 24x12 (6D):
        
        J_full = [ J_FL   0      0      0   ]
                 [  0    J_FR    0      0   ]
                 [  0     0     J_RL    0   ]
                 [  0     0      0     J_RR ]
        """
        J_full_lin = np.zeros((12, 12), dtype=np.float64)
        J_full_6d = np.zeros((24, 12), dtype=np.float64)
        
        for i, info in enumerate(self.legs_info):
            q_start = info["q_start"]
            q_leg = q[q_start:q_start+3]
            J_v, J_6d, _ = self.compute_leg_jacobian(i, q_leg)
            
            # Bloque lineal de 3x3 en la diagonal
            J_full_lin[i*3:i*3+3, q_start:q_start+3] = J_v
            
            # Bloque 6D de 6x3
            J_full_6d[i*6:i*6+3, q_start:q_start+3] = J_v
            J_full_6d[i*6+3:i*6+6, q_start:q_start+3] = J_6d[3:6, :]
            
        return J_full_lin, J_full_6d

    def full_body_floating_jacobian(self, q, p_base=None, R_base=None):
        """
        Jacobiano de cuerpo completo considerando BASE FLOTANTE (Floating Base):
        El estado contiene 6 DOF para la base (3 lineales + 3 angulares) + 12 articulares.
        
        Fórmula para cada pie k:
            v_world,k = v_b + w_b x (p_k - p_b) + R_b * J_leg,k * q_dot_k
            J_fb_k = [ I_3,   -[p_world,k - p_b]_x,   R_b * J_leg,k ]  (3 x 18)
        """
        if p_base is None:
            p_base = np.zeros(3)
        if R_base is None:
            R_base = np.eye(3)
            
        J_fb_lin = np.zeros((12, 18), dtype=np.float64)
        
        for i, info in enumerate(self.legs_info):
            q_start = info["q_start"]
            q_leg = q[q_start:q_start+3]
            J_v, _, p_foot_local = self.compute_leg_jacobian(i, q_leg)
            
            # Posición en el marco global
            p_foot_world = p_base + R_base @ p_foot_local
            r = p_foot_world - p_base
            
            # Matriz antisimétrica [r]_x para el producto cruzado w_b x r = -r x w_b
            r_skew = np.array([
                [ 0.0, -r[2],  r[1]],
                [ r[2],  0.0, -r[0]],
                [-r[1],  r[0],  0.0]
            ])
            
            # Bloque base lineal: I_3
            J_fb_lin[i*3:i*3+3, 0:3] = np.eye(3)
            # Bloque base angular: -[r]_x
            J_fb_lin[i*3:i*3+3, 3:6] = -r_skew
            # Bloque articulaciones: R_b * J_v
            J_fb_lin[i*3:i*3+3, 6 + q_start:6 + q_start + 3] = R_base @ J_v
            
        return J_fb_lin


# =============================================================================
# VALIDACIÓN Y COMPARACIÓN CON PINOCCHIO
# =============================================================================

class KinematicsValidator:
    """
    Compara las funciones analíticas manuales con Pinocchio.
    """

    def __init__(self):
        self.manual = A1KinematicsManual()
        # Modelo fijo de Pinocchio (12 DOF)
        self.pin_model = pin.buildModelFromUrdf(a1_description.URDF_PATH)
        self.pin_data = self.pin_model.createData()
        
        # Mapeo de IDs de marcos de los pies en Pinocchio
        self.foot_names = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
        self.frame_ids = [self.pin_model.getFrameId(name) for name in self.foot_names]

    def test_configuration(self, q, test_name="Prueba"):
        """
        Evalúa una configuración q en ambos modelos y calcula el error numérico.
        """
        # --- Cálculo Manual ---
        p_feet_man, dict_feet_man = self.manual.forward_kinematics_full(q)
        J_man_lin, J_man_6d = self.manual.full_body_jacobian(q)
        
        # --- Cálculo con Pinocchio ---
        pin.forwardKinematics(self.pin_model, self.pin_data, q)
        pin.updateFramePlacements(self.pin_model, self.pin_data)
        pin.computeJointJacobians(self.pin_model, self.pin_data, q)
        
        p_feet_pin_list = []
        J_pin_lin_list = []
        
        for fid in self.frame_ids:
            # Posición en Pinocchio
            pos = self.pin_data.oMf[fid].translation.copy()
            p_feet_pin_list.append(pos)
            
            # Jacobiano en Pinocchio expresado en marco alineado al mundo/base
            # (LOCAL_WORLD_ALIGNED representa la derivada temporal de la posición en la base)
            J_frame = pin.getFrameJacobian(
                self.pin_model, self.pin_data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )
            J_pin_lin_list.append(J_frame[0:3, :])
            
        p_feet_pin = np.concatenate(p_feet_pin_list)
        J_pin_lin = np.vstack(J_pin_lin_list)
        
        # --- Cálculo de Errores ---
        error_fk_max = np.max(np.abs(p_feet_man - p_feet_pin))
        error_fk_norm = np.linalg.norm(p_feet_man - p_feet_pin)
        
        error_jac_max = np.max(np.abs(J_man_lin - J_pin_lin))
        error_jac_norm = np.linalg.norm(J_man_lin - J_pin_lin)
        
        results = {
            "test_name": test_name,
            "q": q,
            "p_man": p_feet_man,
            "p_pin": p_feet_pin,
            "error_fk_max": error_fk_max,
            "error_fk_norm": error_fk_norm,
            "error_jac_max": error_jac_max,
            "error_jac_norm": error_jac_norm,
            "J_man": J_man_lin,
            "J_pin": J_pin_lin
        }
        return results

    def run_benchmark(self, num_samples=100):
        """
        Ejecuta un benchmark estadístico sobre num_samples configuraciones aleatorias.
        """
        print("=" * 80)
        print("BENCHMARK CINEMÁTICO: MANUAL vs PINOCCHIO (UNITREE A1)")
        print(f"Evaluando {num_samples} configuraciones articulares...")
        print("=" * 80)
        
        # Rango articular típico de Unitree A1
        q_low = np.array([-0.8, -1.0, -2.6] * 4)
        q_high = np.array([0.8,  3.5, -0.9] * 4)
        
        errors_fk = []
        errors_jac = []
        
        # 1. Caso cero
        q_zero = np.zeros(12)
        res_zero = self.test_configuration(q_zero, "Configuración Cero")
        errors_fk.append(res_zero["error_fk_max"])
        errors_jac.append(res_zero["error_jac_max"])
        
        # 2. Caso posición nominal de pie (standing nominal)
        q_stand = np.array([0.0, 0.75, -1.50] * 4)
        res_stand = self.test_configuration(q_stand, "Postura Nominal de Pie")
        errors_fk.append(res_stand["error_fk_max"])
        errors_jac.append(res_stand["error_jac_max"])
        
        # 3. Muestras aleatorias en el espacio de trabajo
        np.random.seed(42)
        for i in range(num_samples - 2):
            q_rand = np.random.uniform(q_low, q_high)
            res = self.test_configuration(q_rand, f"Aleatorio_{i}")
            errors_fk.append(res["error_fk_max"])
            errors_jac.append(res["error_jac_max"])
            
        print("\n--- Resultados Detallados: Postura Nominal de Pie ---")
        for i, foot in enumerate(self.foot_names):
            p_m = res_stand["p_man"][i*3:i*3+3]
            p_p = res_stand["p_pin"][i*3:i*3+3]
            diff = np.linalg.norm(p_m - p_p)
            print(f"  {foot:8s} | Manual: [{p_m[0]:.5f}, {p_m[1]:.5f}, {p_m[2]:.5f}] | "
                  f"Pinocchio: [{p_p[0]:.5f}, {p_p[1]:.5f}, {p_p[2]:.5f}] | Diff: {diff:.2e} m")
            
        print("\n--- Resumen Estadístico ({0} configuraciones evaluadas) ---".format(num_samples))
        print(f"  Error Máximo Cinemática Directa (FK) : {np.max(errors_fk):.3e} m")
        print(f"  Error Medio Cinemática Directa (FK)  : {np.mean(errors_fk):.3e} m")
        print(f"  Error Máximo Jacobiano de Cuerpo (J) : {np.max(errors_jac):.3e}")
        print(f"  Error Medio Jacobiano de Cuerpo (J)  : {np.mean(errors_jac):.3e}")
        print("=" * 80)
        
        assert np.max(errors_fk) < 1e-12, "Fallo: Error de FK superior a tolerancia de máquina"
        assert np.max(errors_jac) < 1e-12, "Fallo: Error de Jacobiano superior a tolerancia de máquina"
        print(">> VALIDACIÓN EXITOSA: Ambos métodos coinciden con precisión de punto flotante de máquina (IEEE 754) <<")
        return res_stand, errors_fk, errors_jac


def main():
    validator = KinematicsValidator()
    res_stand, errors_fk, errors_jac = validator.run_benchmark(num_samples=100)
    
    # Mostrar matriz Jacobiana de muestra (primeras 6 filas: FL y FR)
    print("\nVisualización Parcial del Jacobiano de Cuerpo Completo J (12x12) [Postura Nominal]:")
    print(np.round(res_stand["J_man"], 4))


if __name__ == "__main__":
    main()
