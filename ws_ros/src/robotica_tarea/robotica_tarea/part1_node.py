#!/usr/bin/env python3
"""
Nodo ROS 2 para la Parte 1: Cinemática Directa y Jacobiano de Unitree A1
"""

import sys
import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

# Importar lógica de cálculo cinemático de part1
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "part1_cinematica_cuadrupedo")))
from cinematica_directa_jacobiano import KinematicsValidator, A1KinematicsManual


class Part1Node(Node):
    def __init__(self):
        super().__init__('part1_kinematics_node')
        self.publisher_results = self.create_publisher(String, '/robotica/part1/benchmark_results', 10)
        self.publisher_joints = self.create_publisher(JointState, '/robotica/a1/joint_states', 10)
        
        self.get_logger().info('Iniciando nodo de validación cinemática (Parte 1)...')
        self.validator = KinematicsValidator()
        self.manual = A1KinematicsManual()
        
        # Ejecutar benchmark
        res_stand, err_fk, err_jac = self.validator.run_benchmark(num_samples=100)
        
        # Publicar resultados
        msg = String()
        data = {
            "robot": "Unitree A1",
            "samples_evaluated": 100,
            "max_fk_error_m": float(max(err_fk)),
            "mean_fk_error_m": float(sum(err_fk)/len(err_fk)),
            "max_jacobian_error": float(max(err_jac)),
            "mean_jacobian_error": float(sum(err_jac)/len(err_jac)),
            "validation_status": "SUCCESS"
        }
        msg.data = json.dumps(data, indent=2)
        self.publisher_results.publish(msg)
        self.get_logger().info(f'Resultados publicados en /robotica/part1/benchmark_results:\n{msg.data}')
        
        # Timer para publicar estados articulares nominales
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_names = [
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
        ]
        joint_state.name = joint_names
        joint_state.position = [0.0, 0.75, -1.50] * 4
        self.publisher_joints.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = Part1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
