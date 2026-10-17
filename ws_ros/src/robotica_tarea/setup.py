from setuptools import setup
import os
from glob import glob

package_name = 'robotica_tarea'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Estudiante UTEC',
    maintainer_email='estudiante@utec.edu.pe',
    description='Tarea de Robotica Avanzada: Cinematica y Control en PyBullet con ProxQP',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'part1_node = robotica_tarea.part1_node:main',
            'part2_node = robotica_tarea.part2_node:main',
            'part3_node = robotica_tarea.part3_node:main',
        ],
    },
)
