# Guía de Ejecución y Explicación de Actividades - Tarea de Robótica Avanzada

Este repositorio contiene la solución completa de la tarea de robótica avanzada sobre modelado cinemático, cálculo de Jacobiano analítico y control cinemático mediante optimización con **ProxQP** en el simulador **PyBullet**, con validación en **Pinocchio** y soporte para **ROS 2 Humble**.

---

## 📋 Estructura del Proyecto

```text
/home/nvm/Documents/UTEC/Avanzada_tareas/
├── README.md                           <- Este documento: guía y explicación de ejecución
├── REPORTE.md                          <- Reporte académico formal con formulación y resultados
├── part1_cinematica_cuadrupedo/
│   ├── cinematica_directa_jacobiano.py <- Cinemática directa y Jacobiano 12x12 desde cero vs Pinocchio
│   └── generar_graficas_part1.py       <- Script de benchmarking y generación de gráficas 3D
├── part2_control_cuadrupedo/
│   └── control_cinematico_a1.py        <- Marcha secuencial de 4 patas en PyBullet con ProxQP
├── part3_control_humanoide/
│   └── control_cinematico_h1.py        <- Movimientos de brazos, cintura y agache a piso en H1
├── ws_ros/                             <- Espacio de trabajo ROS 2 Humble
│   ├── src/robotica_tarea/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── robotica_tarea/
│   │   │   ├── part1_node.py           <- Nodo ROS 2: Validación cinemática
│   │   │   ├── part2_node.py           <- Nodo ROS 2: Control cuadrúpedo A1
│   │   │   └── part3_node.py           <- Nodo ROS 2: Control humanoide H1
│   │   └── launch/
│   │       ├── part1.launch.py
│   │       ├── part2.launch.py
│   │       └── part3.launch.py
└── outputs/                            <- Gráficas (PNG), animaciones (GIF) y videos (MP4)
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

## 🛠️ Entorno de Ejecución (Distrobox `ros2_humble`)

El entorno ya se encuentra completamente configurado dentro del contenedor distrobox `ros2_humble` con soporte de GPU NVIDIA y las siguientes librerías instaladas:
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

## 🚀 Cómo Ejecutar Cada Actividad

Se proporcionan dos formas de ejecución para cada actividad:
1. **Ejecución directa como script de Python** (rápida, interactiva y con argumentos configurables).
2. **Ejecución a través de ROS 2** en el workspace `ws_ros`.

---

### 🔹 Actividad 1: Cinemática Directa y Jacobiano de Cuerpo Completo (Unitree A1)

**Objetivo:** Modelar desde primeros principios matemáticos (álgebra lineal y matrices homogéneas sin librerías externas) la cinemática directa de las 4 patas del robot cuadrúpedo Unitree A1 y derivar el Jacobiano geométrico de cuerpo completo ($12 \times 12$). Comparar numéricamente contra Pinocchio.

#### Opción A: Script Python Directo
```bash
distrobox enter ros2_humble -- python3 /home/nvm/Documents/UTEC/Avanzada_tareas/part1_cinematica_cuadrupedo/cinematica_directa_jacobiano.py
```

Para generar las gráficas estadísticas del benchmark y el diagrama 3D del robot:
```bash
distrobox enter ros2_humble -- python3 /home/nvm/Documents/UTEC/Avanzada_tareas/part1_cinematica_cuadrupedo/generar_graficas_part1.py
```

#### Opción B: Mediante ROS 2
```bash
distrobox enter ros2_humble -- bash -c "source /opt/ros/humble/setup.bash && source /home/nvm/Documents/UTEC/Avanzada_tareas/ws_ros/install/setup.bash && ros2 run robotica_tarea part1_node"
```
O usando el archivo launch:
```bash
distrobox enter ros2_humble -- bash -c "source /opt/ros/humble/setup.bash && source /home/nvm/Documents/UTEC/Avanzada_tareas/ws_ros/install/setup.bash && ros2 launch robotica_tarea part1.launch.py"
```

---

### 🔹 Actividad 2: Control Cinemático de Robot Cuadrúpedo en PyBullet (Unitree A1)

**Objetivo:** Implementar el control cinemático por inversión cinemática diferencial en lazo cerrado (CLIK) formulado como un problema de programación cuadrática resuelto en cada ciclo de simulación con **ProxQP**. El robot ejecuta una marcha secuencial (*Crawl Gait*) donde cada una de las 4 patas da al menos 1 paso completo de manera secuencial (FL $\to$ RR $\to$ FR $\to$ RL).

#### Opción A: Script Python Directo con Interfaz Gráfica (GUI)
```bash
distrobox enter ros2_humble -- python3 /home/nvm/Documents/UTEC/Avanzada_tareas/part2_control_cuadrupedo/control_cinematico_a1.py --gui
```

Si deseas ejecutarlo en modo sin cabeza (headless) y grabar automáticamente el video MP4 y el GIF animado en `outputs/`:
```bash
distrobox enter ros2_humble -- python3 /home/nvm/Documents/UTEC/Avanzada_tareas/part2_control_cuadrupedo/control_cinematico_a1.py --headless --record
```

#### Opción B: Mediante ROS 2
```bash
distrobox enter ros2_humble -- bash -c "source /opt/ros/humble/setup.bash && source /home/nvm/Documents/UTEC/Avanzada_tareas/ws_ros/install/setup.bash && ros2 run robotica_tarea part2_node"
```
O usando launch:
```bash
distrobox enter ros2_humble -- bash -c "source /opt/ros/humble/setup.bash && source /home/nvm/Documents/UTEC/Avanzada_tareas/ws_ros/install/setup.bash && ros2 launch robotica_tarea part2.launch.py"
```

---

### 🔹 Actividad 3: Control Cinemático de Robot Humanoide en PyBullet (Unitree H1)

**Objetivo:** Realizar el control cinemático en PyBullet del robot humanoide Unitree H1 (19 grados de libertad articulados) mediante ProxQP y Pinocchio, ejecutando:
1. **Movimiento A:** Movimiento coordinado de ambos brazos acompañado del giro oscilatorio de cintura (`torso_joint`).
2. **Movimiento B:** Agache profundo (flexión coordinada de caderas, rodillas y tobillos con descenso de pelvis) y descenso de la mano derecha hasta el nivel del piso para recoger un objeto colocado en el suelo, seguido de reincorporación a la postura erguida.

#### Opción A: Script Python Directo con Interfaz Gráfica (GUI)
```bash
distrobox enter ros2_humble -- python3 /home/nvm/Documents/UTEC/Avanzada_tareas/part3_control_humanoide/control_cinematico_h1.py --gui
```

Para ejecutar en modo sin ventana y generar video MP4 / GIF en `outputs/`:
```bash
distrobox enter ros2_humble -- python3 /home/nvm/Documents/UTEC/Avanzada_tareas/part3_control_humanoide/control_cinematico_h1.py --headless --record
```

#### Opción B: Mediante ROS 2
```bash
distrobox enter ros2_humble -- bash -c "source /opt/ros/humble/setup.bash && source /home/nvm/Documents/UTEC/Avanzada_tareas/ws_ros/install/setup.bash && ros2 run robotica_tarea part3_node"
```
O usando launch:
```bash
distrobox enter ros2_humble -- bash -c "source /opt/ros/humble/setup.bash && source /home/nvm/Documents/UTEC/Avanzada_tareas/ws_ros/install/setup.bash && ros2 launch robotica_tarea part3.launch.py"
```

---

## 📊 Archivos y Resultados Generados

Los resultados visuales y multimedia se guardan de forma persistente en la carpeta `outputs/`:

| Archivo | Tipo | Descripción |
| :--- | :--- | :--- |
| `part1_errores_benchmark.png` | Imagen | Histograma de distribución de error analítico vs Pinocchio ($< 10^{-16}$) |
| `part1_estructura_robot_3d.png` | Imagen | Visualización geométrica 3D del tronco y las 4 cadenas cinemáticas de A1 |
| `part2_a1_gait.gif` | Animación | Animación del robot Unitree A1 ejecutando los 4 pasos secuenciales |
| `part2_a1_gait.mp4` | Video | Video en alta definición de la simulación de locomoción en PyBullet |
| `part2_a1_tracking.png` | Imagen | Curvas de error de seguimiento cartesiano en mm y evolución articular |
| `part3_h1_humanoid.gif` | Animación | Animación del humanoide H1 (brazos, giro de cintura y recojo del objeto) |
| `part3_h1_humanoid.mp4` | Video | Video HD de las secuencias A y B del humanoide en PyBullet |
| `part3_h1_tracking.png` | Imagen | Curvas de altura de pelvis, altura de mano respecto al piso y ángulo de cintura |

Consulte el documento [REPORTE.md](file:///home/nvm/Documents/UTEC/Avanzada_tareas/REPORTE.md) para ver la formulación teórica completa, el análisis de resultados, tablas de evaluación y conclusiones.
