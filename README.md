# Guia de Ejecucion y Explicacion de Actividades - Tarea de Robotica Avanzada

Este repositorio contiene la solucion completa de la tarea de robotica avanzada sobre modelado cinematico, calculo de Jacobiano analitico y control cinematico mediante optimizacion con **ProxQP** en el simulador **PyBullet**, con validacion en **Pinocchio** y soporte para **ROS 2 Humble**.

---

## 1. Estructura del Proyecto

```text
/home/nvm/Documents/UTEC/Avanzada_tareas/
├── README.md                           <- Este documento: guia y explicacion de ejecucion
├── REPORTE.md                          <- Reporte academico formal con formulacion y resultados
├── part1_cinematica_cuadrupedo/
│   ├── cinematica_directa_jacobiano.py <- Cinematica directa y Jacobiano 12x12 desde cero vs Pinocchio
│   └── generar_graficas_part1.py       <- Script de benchmarking y generacion de graficas 3D
├── part2_control_cuadrupedo/
│   └── control_cinematico_a1.py        <- Marcha secuencial de 4 patas en PyBullet con ProxQP
├── part3_control_humanoide/
│   └── control_cinematico_h1.py        <- Movimientos de brazos, cintura y agache a piso en H1
├── ws_ros/                             <- Espacio de trabajo ROS 2 Humble
│   ├── src/robotica_tarea/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── robotica_tarea/
│   │   │   ├── part1_node.py           <- Nodo ROS 2: Validacion cinematica
│   │   │   ├── part2_node.py           <- Nodo ROS 2: Control cuadrupedo A1
│   │   │   └── part3_node.py           <- Nodo ROS 2: Control humanoide H1
│   │   └── launch/
│   │       ├── part1.launch.py
│   │       ├── part2.launch.py
│   │       └── part3.launch.py
└── outputs/                            <- Graficas (PNG), animaciones (GIF) y videos (MP4)
    ├── part1_errores_benchmark.png
    ├── part1_estructura_robot_3d.png
    ├── part2_a1_gait.gif
    ├── part2_a1_gait.mp4
    ├── part2_a1_tracking.png
    ├── part3_h1_humanoid.gif
    ├── part3_h1_humanoid.mp4
    └── part3_h1_tracking.png
```

---

## 2. Entorno de Ejecucion (Distrobox ros2_humble)

El entorno ya se encuentra completamente configurado dentro del contenedor distrobox `ros2_humble` con soporte de GPU NVIDIA y las siguientes librerias instaladas:
- **Python 3.10**
- **Pinocchio 4.1.0** (`pin`)
- **PyBullet 3.2.7**
- **ProxQP 0.7.3** (`proxsuite`)
- **Robot Descriptions 3.3.0** (`robot_descriptions` con URDFs de Unitree A1 y H1)
- **ROS 2 Humble Desktop**

Para ingresar al contenedor desde cualquier terminal:
```bash
distrobox enter ros2_humble
```

---

## 3. Como Ejecutar Cada Actividad

Se proporcionan dos formas de ejecucion para cada actividad:
1. **Ejecucion directa como script de Python** (rapida, interactiva y con argumentos configurables).
2. **Ejecucion a traves de ROS 2** en el workspace `ws_ros`.

---

### Actividad 1: Cinematica Directa y Jacobiano de Cuerpo Completo (Unitree A1)

**Objetivo:** Modelar desde primeros principios matematicos (algebra lineal y matrices homogeneas sin librerias externas) la cinematica directa de las 4 patas del robot cuadrupedo Unitree A1 y derivar el Jacobiano geometrico de cuerpo completo ($12 \times 12$). Comparar numericamente contra Pinocchio.

#### Opcion A: Script Python Directo
```bash
distrobox enter ros2_humble -- python3 /home/nvm/Documents/UTEC/Avanzada_tareas/part1_cinematica_cuadrupedo/cinematica_directa_jacobiano.py
```

Para generar las graficas estadisticas del benchmark y el diagrama 3D del robot:
```bash
distrobox enter ros2_humble -- python3 /home/nvm/Documents/UTEC/Avanzada_tareas/part1_cinematica_cuadrupedo/generar_graficas_part1.py
```

#### Opcion B: Mediante ROS 2
```bash
distrobox enter ros2_humble -- bash -c "source /opt/ros/humble/setup.bash && source /home/nvm/Documents/UTEC/Avanzada_tareas/ws_ros/install/setup.bash && ros2 run robotica_tarea part1_node"
```
O usando el archivo launch:
```bash
distrobox enter ros2_humble -- bash -c "source /opt/ros/humble/setup.bash && source /home/nvm/Documents/UTEC/Avanzada_tareas/ws_ros/install/setup.bash && ros2 launch robotica_tarea part1.launch.py"
```

---

### Actividad 2: Control Cinematico de Robot Cuadrupedo en PyBullet (Unitree A1)

**Objetivo:** Implementar el control cinematico por inversion cinematica diferencial en lazo cerrado (CLIK) formulado como un problema de programacion cuadratica resuelto en cada ciclo de simulacion con **ProxQP**. El robot ejecuta una marcha secuencial (*Crawl Gait*) donde cada una de las 4 patas da al menos 1 paso completo de manera secuencial (FL $\to$ RR $\to$ FR $\to$ RL).

#### Opcion A: Script Python Directo con Interfaz Grafica (GUI)
```bash
distrobox enter ros2_humble -- python3 /home/nvm/Documents/UTEC/Avanzada_tareas/part2_control_cuadrupedo/control_cinematico_a1.py --gui
```

Si deseas ejecutarlo en modo sin cabeza (headless) y grabar automaticamente el video MP4 y el GIF animado en `outputs/`:
```bash
distrobox enter ros2_humble -- python3 /home/nvm/Documents/UTEC/Avanzada_tareas/part2_control_cuadrupedo/control_cinematico_a1.py --headless --record
```

#### Opcion B: Mediante ROS 2
```bash
distrobox enter ros2_humble -- bash -c "source /opt/ros/humble/setup.bash && source /home/nvm/Documents/UTEC/Avanzada_tareas/ws_ros/install/setup.bash && ros2 run robotica_tarea part2_node"
```
O usando launch:
```bash
distrobox enter ros2_humble -- bash -c "source /opt/ros/humble/setup.bash && source /home/nvm/Documents/UTEC/Avanzada_tareas/ws_ros/install/setup.bash && ros2 launch robotica_tarea part2.launch.py"
```

---

### Actividad 3: Control Cinematico de Robot Humanoide en PyBullet (Unitree H1)

**Objetivo:** Realizar el control cinematico en PyBullet del robot humanoide Unitree H1 (19 grados de libertad articulados) mediante ProxQP y Pinocchio, ejecutando:
1. **Movimiento A:** Movimiento coordinado de ambos brazos acompañado del giro oscilatorio de cintura (`torso_joint`).
2. **Movimiento B:** Agache profundo (flexion coordinada de caderas, rodillas y tobillos con descenso de pelvis) y descenso de la mano derecha hasta el nivel del piso para recoger un objeto colocado en el suelo, seguido de reincorporacion a la postura erguida.

#### Opcion A: Script Python Directo con Interfaz Grafica (GUI)
```bash
distrobox enter ros2_humble -- python3 /home/nvm/Documents/UTEC/Avanzada_tareas/part3_control_humanoide/control_cinematico_h1.py --gui
```

Para ejecutar en modo sin ventana y generar video MP4 / GIF en `outputs/`:
```bash
distrobox enter ros2_humble -- python3 /home/nvm/Documents/UTEC/Avanzada_tareas/part3_control_humanoide/control_cinematico_h1.py --headless --record
```

#### Opcion B: Mediante ROS 2
```bash
distrobox enter ros2_humble -- bash -c "source /opt/ros/humble/setup.bash && source /home/nvm/Documents/UTEC/Avanzada_tareas/ws_ros/install/setup.bash && ros2 run robotica_tarea part3_node"
```
O usando launch:
```bash
distrobox enter ros2_humble -- bash -c "source /opt/ros/humble/setup.bash && source /home/nvm/Documents/UTEC/Avanzada_tareas/ws_ros/install/setup.bash && ros2 launch robotica_tarea part3.launch.py"
```

---

## 4. Archivos y Resultados Generados

Los resultados visuales y multimedia se guardan de forma persistente en la carpeta `outputs/`:

| Archivo | Tipo | Descripcion |
| :--- | :--- | :--- |
| `part1_errores_benchmark.png` | Imagen | Histograma de distribucion de error analitico vs Pinocchio ($< 10^{-16}$) |
| `part1_estructura_robot_3d.png` | Imagen | Visualizacion geometrica 3D del tronco y las 4 cadenas cinematicas de A1 |
| `part2_a1_gait.gif` | Animacion | Animacion del robot Unitree A1 ejecutando los 4 pasos secuenciales |
| `part2_a1_gait.mp4` | Video | Video en alta definicion de la simulacion de locomocion en PyBullet |
| `part2_a1_tracking.png` | Imagen | Curvas de error de seguimiento cartesiano en mm y evolucion articular |
| `part3_h1_humanoid.gif` | Animacion | Animacion del humanoide H1 (brazos, giro de cintura y recojo del objeto) |
| `part3_h1_humanoid.mp4` | Video | Video HD de las secuencias A y B del humanoide en PyBullet |
| `part3_h1_tracking.png` | Imagen | Curvas de altura de pelvis, altura de mano respecto al piso y angulo de cintura |

Consulte el documento [REPORTE.md](file:///home/nvm/Documents/UTEC/Avanzada_tareas/REPORTE.md) para ver la formulacion teorica completa, el analisis de resultados, tablas de evaluacion y conclusiones.
