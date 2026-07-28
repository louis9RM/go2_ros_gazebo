<!-- Sugerencia: reemplaza este espacio por una imagen o GIF del Go2 caminando en Gazebo.
     Colócala en docs/ (p. ej. docs/go2_gazebo.gif) y descomenta la línea siguiente. -->
<!-- ![Unitree Go2 caminando en Gazebo](docs/go2_gazebo.gif) -->

# 🐾 Go2 ROS 2 + Gazebo — Locomoción cuadrúpeda con RL

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Ignition%2FFortress-FF6600?logo=gazebo&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)
![PyTorch](https://img.shields.io/badge/PyTorch-policy-EE4C2C?logo=pytorch&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green)

> Simulación del robot cuadrúpedo **Unitree Go2** en **ROS 2 + Gazebo**, controlado por una **política de locomoción entrenada con aprendizaje por refuerzo (RL)** que se ejecuta en tiempo real desde un nodo de ROS 2.

---

## ¿Qué es esto y qué NO es?

Este repositorio es un **entorno de simulación funcional** de un robot que camina, no un producto de inspección minera terminado. Soy explícito sobre esto porque la honestidad técnica importa:

**Lo que SÍ es:**
- Un cuadrúpedo Unitree Go2 que se **spawnea y simula en Gazebo** con física, articulaciones controladas e IMU.
- Un **nodo de ROS 2 en Python + PyTorch** (`go2_rl_driver`) que carga una red neuronal entrenada, arma la observación del robot a 50 Hz, infiere las acciones y las publica hacia las 12 articulaciones de las patas.
- Un puente ROS ↔ Gazebo que conecta sensores, odometría y comandos de velocidad (`/cmd_vel`).

**Lo que NO es (todavía):**
- **No** es un stack de inspección autónoma minera listo para producción.
- **No** hace percepción del terreno en línea: la política espera un escaneo de altura del suelo (187 valores) que **actualmente se alimenta con ceros** (ver [Estado del proyecto](#estado-del-proyecto)). Es decir, el robot camina sobre la propiocepción y el comando de velocidad, pero **aún no "ve" el relieve**.
- **No** incluye lidar ni cámara activos: ambos sensores están **definidos pero comentados** en el URDF.
- La política **no fue entrenada en este repo**: proviene de un proyecto de terceros (ver [Autoría y créditos](#autoría-y-créditos)); aquí se **despliega e integra** en ROS 2 + Gazebo.

---

## Relevancia para automatización e inspección minera

En minería —especialmente en el Perú— hay entornos donde un robot con ruedas u orugas simplemente **no puede operar**: labores subterráneas estrechas, terreno rocoso e irregular, escaleras, taludes, zonas post-voladura y áreas de riesgo geomecánico donde exponer personal es peligroso.

Un **robot cuadrúpedo camina donde uno con ruedas no llega**. Casos de uso concretos:

- **Inspección de labores subterráneas** y galerías de difícil acceso.
- **Monitoreo en terreno irregular** (relaves, pilas de lixiviación, taludes).
- **Rondas de inspección en zonas de riesgo** para personas (gases, inestabilidad, post-voladura).
- **Lectura de instrumentación y detección de anomalías** en áreas remotas de la operación.

Esto no es especulativo: empresas mineras y de inspección industrial **ya usan robots cuadrúpedos** como el **Spot de Boston Dynamics** y el propio **Unitree Go2** para rondas autónomas y monitoreo en instalaciones de riesgo. Este proyecto es la **base técnica** —simulación, control de locomoción por RL e integración en ROS 2— sobre la que se construyen esas capacidades, sin exagerar lo que hoy hace.

---

## Arquitectura / cómo funciona

```
        ┌────────────────────────┐         /cmd_vel (Twist)
        │   Gazebo (Ignition)    │◄──────── comando de velocidad del usuario
        │  Física + Go2 spawneado│
        └───────────┬────────────┘
                    │  sensores / estado (vía ros_gz_bridge)
     /go2/imu  ─────┤  /joint_states  ─────┤  /model/go2/odometry
                    ▼
        ┌────────────────────────┐
        │   go2_rl_driver (nodo) │   50 Hz
        │   Observación (235) →   │
        │   Actor MLP (PyTorch)  │   235→512→256→128→12  (ELU)
        │   → 12 acciones         │
        └───────────┬────────────┘
                    │  /go2/joint/<pata>_<art>  (Float64, posición objetivo)
                    ▼
        ┌────────────────────────┐
        │ JointPositionController│   PID por articulación (Gazebo)
        └────────────────────────┘
```

**Nodo de control — [`go2_ros_gazebo/go2_rl_driver.py`](go2_ros_gazebo/go2_rl_driver.py):**
- Corre a **50 Hz** (`control_loop`).
- Construye un vector de **observación de 235 dimensiones**: velocidad lineal (3) + velocidad angular (3) + gravedad proyectada (3) + comando de velocidad (3) + posición articular relativa (12) + velocidad articular (12) + últimas acciones (12) + **escaneo de altura del terreno (187, hoy en ceros)**.
- Ejecuta la red **Actor** (MLP con activaciones ELU) para producir **12 acciones**, las escala (`action_scale = 0.25`) y las suma a la pose de reposo (parado) para obtener las posiciones objetivo de cada articulación.
- Publica cada objetivo en `/go2/joint/<articulación>`, donde el **JointPositionController** de Gazebo (PID: `p=500, i=0.1, d=10`) las sigue.

**Launch files ([`launch/`](launch/)):**

| Launch | Qué hace |
|---|---|
| [`rl_deploy.launch.py`](launch/rl_deploy.launch.py) | **Todo junto**: simulación + robot + nodo de control RL (con arranque diferido de 5 s). |
| [`spawn.launch.py`](launch/spawn.launch.py) | Publica el `robot_description`, spawnea el Go2 y levanta el `ros_gz_bridge`. |
| [`empty_world.launch.py`](launch/empty_world.launch.py) | Lanza Gazebo con el mundo [`worlds/demo_world.sdf`](worlds/demo_world.sdf). |
| [`rviz2.launch.py`](launch/rviz2.launch.py) | Visualización del robot en RViz2. |

---

## Sensores y percepción

| Recurso | Tópico | Estado |
|---|---|---|
| **IMU** | `/go2/imu` | ✅ Activo (100 Hz). Alimenta velocidad angular y gravedad proyectada de la observación. |
| **Odometría** | `/model/go2/odometry` | ✅ Activa (50 Hz, plugin de Gazebo). Da la velocidad lineal de la base. |
| **Estado articular** | `/joint_states` | ✅ Activo. Posición y velocidad de las 12 articulaciones. |
| **Comando de velocidad** | `/cmd_vel` | ✅ Entrada del usuario (lineal x/y, angular z). |
| **Lidar / radar** | `/go2/radar` | ⬜ Definido en el URDF pero **comentado** (no publica). El puente lo mapea, listo para activar. |
| **Cámara frontal** | `/go2/camera` | ⬜ Definida en el URDF pero **comentada** (no publica). |
| **Escaneo de altura del terreno** | — | 🚧 La política lo espera (187 valores) pero se alimenta con **ceros**; falta conectar percepción. |

---

## Estado del proyecto

| Componente | Estado |
|---|---|
| Spawn y simulación del Go2 en Gazebo | ✅ Funcional |
| Puente ROS ↔ Gazebo (sensores, odometría, articulaciones) | ✅ Funcional |
| IMU, odometría y `joint_states` | ✅ Funcional |
| Carga e inferencia de la política RL en PyTorch (nodo `go2_rl_driver`) | ✅ Funcional |
| Control de las 12 articulaciones vía `JointPositionController` | ✅ Funcional |
| Teleoperación por `/cmd_vel` | ✅ Funcional |
| Marcha estable / caminata robusta | 🚧 En ajuste — el nodo incluye chequeos de acciones anómalas; sin percepción de terreno la marcha es limitada |
| Percepción de terreno (escaneo de altura 187-D) | ⬜ Trabajo futuro — hoy alimentada con ceros |
| Lidar/radar y cámara | ⬜ Trabajo futuro — definidos pero comentados en el URDF |
| Entrenamiento de la política en este repo | ⬜ No incluido — la política proviene de un proyecto externo |

> Leyenda: ✅ funcional · 🚧 en desarrollo · ⬜ trabajo futuro

---

## Stack técnico

- **ROS 2** (probado con Humble, Python 3.10) — nodos, launch, `ros_gz_bridge`.
- **Gazebo (Ignition / Fortress)** — física, `JointPositionController`, `OdometryPublisher`, `JointStatePublisher`, sensor IMU.
- **Python 3.10** — nodo de control (`rclpy`).
- **PyTorch** — red neuronal Actor (MLP 235→512→256→128→12, ELU) que ejecuta la política de locomoción por RL en CPU.
- **NumPy**, **robot_state_publisher**, **xacro**, **RViz2**.
- Descripción del robot: **URDF + mallas `.dae`** del Unitree Go2.

---

## Cómo ejecutarlo

> Requiere ROS 2 (Humble), Gazebo (Ignition/Fortress), `ros_gz_sim`, `ros_gz_bridge` y PyTorch instalado en el entorno de Python de ROS.

**1. Clonar dentro de un workspace de ROS 2:**

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/louis9RM/go2_ros_gazebo.git
```

**2. Compilar con colcon:**

```bash
cd ~/ros2_ws
colcon build --packages-select go2_ros_gazebo
source install/setup.bash
```

**3. Lanzar todo (simulación + robot + control RL):**

```bash
ros2 launch go2_ros_gazebo rl_deploy.launch.py
```

**4. Teleoperar el robot (en otra terminal):**

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

Launches individuales útiles para depurar:

```bash
ros2 launch go2_ros_gazebo empty_world.launch.py   # solo Gazebo + mundo
ros2 launch go2_ros_gazebo spawn.launch.py         # robot + puente, sin control
ros2 launch go2_ros_gazebo rviz2.launch.py         # visualización en RViz2
```

---

## Estructura del repositorio

```
go2_ros_gazebo/
├── go2_ros_gazebo/go2_rl_driver.py   # Nodo de control: observación + inferencia RL + publicación
├── launch/                           # rl_deploy, spawn, empty_world, rviz2
├── urdf/go2_description.urdf         # Descripción del Go2 (Unitree) + plugins de Gazebo
├── dae/                              # Mallas 3D del robot (Unitree)
├── worlds/demo_world.sdf             # Mundo de simulación
├── checkpoints/model_7850.pt         # Pesos de la política RL (~6.8 MB)
├── inspect_model.py                  # Utilidad para inspeccionar el checkpoint
├── package.xml · setup.py            # Metadatos del paquete ROS 2
└── LICENSE                           # MIT
```

---

## Autoría y créditos

**Autor:** **Ever Ramos** — Ingeniero Electrónico (Universidad Nacional de Ingeniería, UNI) · M.Sc. en Ingeniería de Software (Universidad Nacional Mayor de San Marcos, UNMSM).

Trabajo propio en este repositorio: integración del robot en **ROS 2 + Gazebo**, el nodo de control `go2_rl_driver` (construcción de la observación, inferencia de la política y mapeo a las articulaciones), los launch files, el puente ROS ↔ Gazebo y la configuración del mundo de simulación.

**Créditos a terceros:**
- **Modelo del robot Unitree Go2** (URDF y mallas `.dae`): propiedad de **Unitree Robotics**, derivado de sus descripciones públicas (`unitree_ros` / `go2_description`).
- **Política de locomoción RL** (`checkpoints/model_7850.pt`): la arquitectura y los pesos siguen el proyecto **[go2_omniverse](https://github.com/abizovnuralem/go2_omniverse)**, que entrena la política del Go2 en **NVIDIA Isaac Sim / Isaac Lab**. Aquí **no se re-entrena**: el checkpoint se carga y se despliega en ROS 2 + Gazebo.

> **Obtención del checkpoint:** el archivo `checkpoints/model_7850.pt` (~6.8 MB) se incluye en el repo por conveniencia. Si necesitas re-generarlo o entrenar el tuyo, revisa el proyecto **go2_omniverse** (entrenamiento en Isaac Lab) y coloca el `.pt` resultante en `checkpoints/`.

---

## Licencia

Código de este repositorio bajo licencia **MIT** (ver [`LICENSE`](LICENSE)). El modelo del Go2 (Unitree) y la política RL (go2_omniverse) se rigen por las licencias de sus proyectos de origen.
