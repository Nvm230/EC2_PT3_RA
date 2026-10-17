#!/usr/bin/env python3
"""
Nodo ROS 2 para la Parte 3: Control Cinemático de Humanoide Unitree H1 en PyBullet
"""

import sys
import os
import rclpy
from rclpy.node import Node

# Importar lógica de simulación de part3
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "part3_control_humanoide")))
from control_cinematico_h1 import run_humanoid_simulation


class Part3Node(Node):
    def __init__(self):
        super().__init__('part3_humanoid_control_node')
        self.declare_parameter('gui', True)
        self.declare_parameter('record', False)
        
        gui = self.get_parameter('gui').get_parameter_value().bool_value
        record = self.get_parameter('record').get_parameter_value().bool_value
        
        if not os.environ.get("DISPLAY"):
            gui = False
            
        self.get_logger().info(f'Iniciando nodo de control humanoide (Parte 3) [GUI={gui}]...')
        run_humanoid_simulation(gui=gui, record_video=record)
        self.get_logger().info('Secuencia de movimientos humanoide finalizada.')


def main(args=None):
    rclpy.init(args=args)
    node = Part3Node()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
