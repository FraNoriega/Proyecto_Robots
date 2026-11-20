# Robot móvil Ackermann — Trayectorias F, J y W

Simulación de un robot con cinemática Ackermann diseñado en Autodesk Fusion 360, exportado a URDF/Xacro y ejecutado en Gazebo con ROS 2 Jazzy. El controlador recorre las iniciales **F, J y W**, visualiza la trayectoria en RViz y permite generar y guardar el mapa del entorno con SLAM Toolbox y `map_saver_cli`.

## Requisitos

- Ubuntu con **ROS 2 Jazzy** y **Gazebo Sim** compatibles.
- `colcon`, `ros_gz`, `xacro`, `robot_state_publisher`, RViz 2, SLAM Toolbox y Nav2 Map Server.

Para instalar las dependencias principales que falten:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions \
  ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server
```

## 1. Compilar el proyecto

Desde la carpeta raíz del workspace (en los ejemplos, `~/robot_fjw_ws`):

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot_fjw_ws
colcon build --symlink-install
source install/setup.bash
```

## 2. Ejecutar la simulación

**Abrir una terminal diferente para cada proceso** y mantenerlas abiertas durante el recorrido. Ejecutar los siguientes comandos en orden.

### Terminal 1 — Gazebo

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot_fjw_ws
source install/setup.bash
ros2 launch robot_fjw_description gazebo.launch.py
```

### Terminal 2 — Mapeo SLAM

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot_fjw_ws
source install/setup.bash
ros2 launch robot_fjw_description mapping.launch.py
```

### Terminal 3 — RViz (trayectoria y mapa)

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot_fjw_ws
source install/setup.bash
ros2 launch robot_fjw_description rviz_entrega.launch.py
```

En RViz, usar **Fixed Frame: `map`** y vista superior **TopDownOrtho**. Se visualizan las letras planificadas, el recorrido real y el mapa publicado en `/map`.

### Terminal 4 — Ejecutar las letras F, J y W

Una vez que Gazebo, SLAM y RViz estén funcionando:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot_fjw_ws
source install/setup.bash
ros2 run robot_fjw_control ejecutar_fjw --ros-args \
  -p use_sim_time:=true \
  -p max_speed:=0.15 \
  -p lookahead:=0.30
```

El robot ejecuta las trayectorias **F → J → W**. El movimiento se observa en Gazebo y las trayectorias se representan en RViz. No ejecutar simultáneamente otro controlador de `/cmd_vel`.

## 3. Guardar el mapa generado

**Terminal 5 —** Al completar el recorrido, con Gazebo y SLAM Toolbox todavía abiertos:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot_fjw_ws
source install/setup.bash
bash scripts/guardar_mapa_final.sh
```

El script llama a `nav2_map_server map_saver_cli` para guardar el mapa publicado en `/map`. Verificar que se hayan generado ambos archivos:

```bash
ls -lh mapas/mapa_fjw.pgm mapas/mapa_fjw.yaml
```

El repositorio debe incluir los archivos reales `mapas/mapa_fjw.pgm` y `mapas/mapa_fjw.yaml` generados durante la simulación.
