#!/usr/bin/env python3
"""
Nodo ROS 2 para la Parte 2: Control Cinemático de Unitree A1 en PyBullet con ProxQP
"""

import sys
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

# Importar lógica de simulación de part2
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "part2_control_cuadrupedo")))
from control_cinematico_a1 import run_quadruped_gait_simulation


class Part2Node(Node):
    def __init__(self):
        super().__init__('part2_quadruped_control_node')
        self.declare_parameter('gui', True)
        self.declare_parameter('record', False)
        
        gui = self.get_parameter('gui').get_parameter_value().bool_value
        record = self.get_parameter('record').get_parameter_value().bool_value
        
        # Si no hay DISPLAY, usar headless
        if not os.environ.get("DISPLAY"):
            gui = False
            
        self.get_logger().info(f'Iniciando nodo de control cuadrúpedo (Parte 2) [GUI={gui}]...')
        
        # Ejecutar simulación
        run_quadruped_gait_simulation(gui=gui, record_video=record)
        self.get_logger().info('Secuencia de marcha finalizada exitosamente.')


def main(args=None):
    rclpy.init(args=args)
    node = Part2Node()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
