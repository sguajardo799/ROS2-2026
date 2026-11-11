# gazebo_introduction

Paquete de ejemplo para introducir la simulación de un robot móvil diferencial con **Gazebo (Harmonic)** y **ROS 2 Jazzy**: descripción del robot en URDF (con masa e inercia), visualización en RViz y simulación física en Gazebo.

## Estructura

```
gazebo_introduction/
├── launch/
│   ├── visualization.launch.py   # robot_state_publisher + joint_state_publisher_gui + rviz2
│   └── simulation.launch.py      # Gazebo + spawn + bridge + visualization.launch.py
├── models/
│   ├── diff_robot.urdf           # Robot diferencial con masa, inercia y plugins de Gazebo
│   ├── diff_robot_camera.urdf    # Igual al anterior + cámara frontal
│   └── diff_robot_depth.urdf     # Igual al anterior + cámara de profundidad frontal
├── package.xml
└── setup.py
```

## Archivos

### `models/diff_robot.urdf`

Robot diferencial compuesto por:

| Link              | Geometría                   | Masa   | Descripción                          |
|-------------------|-----------------------------|--------|--------------------------------------|
| `base_link`       | Caja 0.4 × 0.3 × 0.1 m      | 2.0 kg | Chasis del robot                     |
| `rueda_izquierda` | Cilindro r = 0.1, h = 0.05  | 0.5 kg | Rueda motriz izquierda               |
| `rueda_derecha`   | Cilindro r = 0.1, h = 0.05  | 0.5 kg | Rueda motriz derecha                 |
| `rueda_loca`      | Esfera r = 0.05             | 0.2 kg | Rueda de apoyo delantera (sin fricción) |

- **Joints**: las ruedas motrices usan joints `continuous` (giro sin límite) en el eje Y; la rueda loca va unida con un joint `fixed`.
- **Inercia**: cada link incluye un bloque `<inertial>` calculado con las fórmulas de sólidos simples (caja, cilindro y esfera), indicadas como comentario en el archivo. Gazebo necesita masa e inercia para simular la física; RViz las ignora.
- **Bloques `<gazebo>`**: información solo para el simulador:
  - `mu1`/`mu2 = 0` en la rueda loca para que deslice libremente.
  - Plugin `DiffDrive`: recibe velocidades en `/cmd_vel`, mueve las ruedas y publica la odometría en `/odom` y la transformada `odom → base_link` en `/tf`.
  - Plugin `JointStatePublisher`: publica la posición de las ruedas en `/joint_states`.

### `models/diff_robot_camera.urdf`

Copia de `diff_robot.urdf` que agrega una cámara en la parte frontal del chasis:

- Link `camara`: caja 0.05 × 0.1 × 0.05 m, 0.1 kg (con su inercia), unida a `base_link` con el joint `fixed` `joint_camara`, mirando hacia +X.
- Sensor `camera` de Gazebo (640 × 480, 30 Hz, FOV 60°) que publica `/camera/image` y `/camera/camera_info` con `frame_id: camara`.
- Plugin `Sensors`: necesario para que Gazebo genere las imágenes, ya que el mundo `empty.sdf` no lo incluye.

> Al convertir el URDF, Gazebo muestra un warning por `gz_frame_id`; es esperado y no afecta el funcionamiento.

### `models/diff_robot_depth.urdf`

Copia de `diff_robot_camera.urdf` donde la cámara se reemplaza por una **cámara de profundidad** (`type="depth_camera"`):

- Mismo link `camara` y joint `joint_camara` en la parte frontal.
- 320 × 240, 15 Hz, FOV 60°, rango de 0.05 a 10 m (formato `R_FLOAT32`, distancia en metros por píxel).
- Publica la imagen de profundidad en `/depth_camera` y la nube de puntos en `/depth_camera/points`, ambas con `frame_id: camara`.
- La nube se expresa con X hacia adelante, igual que el link `camara`, por lo que se ve bien orientada en RViz sin transformaciones extra.

### `launch/visualization.launch.py`

Copia del launch de `ros_introduction`, con algunos argumentos extra. Lanza:

- `robot_state_publisher`: lee el URDF y publica las transformadas (TF) de cada link.
- `joint_state_publisher_gui`: ventana con sliders para mover los joints manualmente.
- `rviz2`: visualizador (se abre sin configuración).

| Argumento      | Default                     | Descripción                                        |
|----------------|-----------------------------|----------------------------------------------------|
| `model`        | `diff_robot.urdf`           | Nombre del archivo URDF dentro de `models/`        |
| `use_gui`      | `true`                      | Lanza o no `joint_state_publisher_gui`             |
| `use_sim_time` | `false`                     | Usa el reloj de la simulación (`/clock`)           |

### `launch/simulation.launch.py`

Integra la simulación con la visualización.

| Argumento | Default           | Descripción                                                         |
|-----------|-------------------|---------------------------------------------------------------------|
| `model`   | `diff_robot.urdf` | Nombre del archivo URDF dentro de `models/` (se pasa a `visualization.launch.py`) |
| `world`   | `empty.sdf`       | Mundo de Gazebo: nombre de un mundo instalado o ruta absoluta a un `.sdf` |

Lanza:

1. `visualization.launch.py` con `use_gui:=false` (los joints los publica Gazebo, no los sliders) y `use_sim_time:=true`.
2. Gazebo con el mundo indicado en `world` (por defecto `empty.sdf`).
3. `ros_gz_sim create`: inserta el robot en Gazebo leyendo el tópico `/robot_description`.
4. `ros_gz_bridge parameter_bridge`: conecta los tópicos de Gazebo con ROS 2:

| Tópico          | Tipo ROS 2                     | Dirección       |
|-----------------|--------------------------------|-----------------|
| `/clock`        | `rosgraph_msgs/msg/Clock`      | Gazebo → ROS 2  |
| `/cmd_vel`      | `geometry_msgs/msg/Twist`      | ROS 2 → Gazebo  |
| `/odom`         | `nav_msgs/msg/Odometry`        | Gazebo → ROS 2  |
| `/tf`           | `tf2_msgs/msg/TFMessage`       | Gazebo → ROS 2  |
| `/joint_states` | `sensor_msgs/msg/JointState`   | Gazebo → ROS 2  |
| `/camera/image` | `sensor_msgs/msg/Image`        | Gazebo → ROS 2  |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo → ROS 2  |
| `/depth_camera` | `sensor_msgs/msg/Image`        | Gazebo → ROS 2  |
| `/depth_camera/points` | `sensor_msgs/msg/PointCloud2` | Gazebo → ROS 2  |

Los tópicos `/camera/*` solo tienen datos al usar `diff_robot_camera.urdf`, y los `/depth_camera*` al usar `diff_robot_depth.urdf`.

## Uso

### Compilar

Asegurar tener disponible ROS2 en terminal:
```bash
source /opt/ros/jazzy/setup.bash
```

Desde la raíz del workspace:

```bash
colcon build --packages-select gazebo_introduction
source install/setup.bash
```

### Visualización (solo RViz)

```bash
ros2 launch gazebo_introduction visualization.launch.py
```

Para usar otro modelo de `models/`:

```bash
ros2 launch gazebo_introduction visualization.launch.py model:=diff_robot_camera.urdf
```

En RViz: **Fixed Frame** = `base_link` y agregar un display **RobotModel** con el tópico `/robot_description`. Mover las ruedas con los sliders de `joint_state_publisher_gui`.

### Simulación (Gazebo + RViz)

```bash
ros2 launch gazebo_introduction simulation.launch.py
```

En RViz: **Fixed Frame** = `odom` y agregar un display **RobotModel** con el tópico `/robot_description`.

Para mover el robot, en otra terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

o publicando directamente una velocidad:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}"
```

### Simulación con cámara

```bash
ros2 launch gazebo_introduction simulation.launch.py model:=diff_robot_camera.urdf
```

En RViz, además del RobotModel, agregar un display **Image** con el tópico `/camera/image`. Como el mundo está vacío, insertar figuras en Gazebo (barra superior: caja, esfera, cilindro) frente al robot para verlas en la cámara.

Para revisar la frecuencia de la cámara:

```bash
ros2 topic hz /camera/image
```

### Simulación con cámara de profundidad

```bash
ros2 launch gazebo_introduction simulation.launch.py model:=diff_robot_depth.urdf
```

En RViz (Fixed Frame = `odom`), además del RobotModel, agregar:

- Display **PointCloud2** con el tópico `/depth_camera/points` (se recomienda *Style* = `Points` y *Size* pequeño).
- Opcional: display **Image** con el tópico `/depth_camera` para ver la imagen de profundidad.

Igual que con la cámara RGB, insertar figuras en Gazebo frente al robot para verlas en la nube de puntos.

### Cambiar el mundo

Con un mundo de los que vienen instalados con Gazebo (en `/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/`) basta con el nombre:

```bash
ros2 launch gazebo_introduction simulation.launch.py world:=shapes.sdf
```

Con un mundo descargado, usar la ruta absoluta:

```bash
ros2 launch gazebo_introduction simulation.launch.py world:=/ruta/absoluta/mi_mundo.sdf
```

Se puede combinar con `model`:

```bash
ros2 launch gazebo_introduction simulation.launch.py model:=diff_robot_depth.urdf world:=shapes.sdf
```

Consideraciones:

- El robot aparece en el origen `(0, 0, 0.1)`. Si el mundo tiene un objeto en ese punto (por ejemplo, la caja de `shapes.sdf`), el robot será empujado por la física al iniciar.
- Evitar mundos que ya incluyan su propio robot o el plugin `gz::sim::systems::Sensors`, ya que este último se duplica con el de los URDF con cámara.
- Si el mundo usa modelos o texturas propias (carpetas `models/`, `meshes/`), agregar su carpeta a la variable `GZ_SIM_RESOURCE_PATH` antes de lanzar:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/ruta/a/la/carpeta/del/mundo
```

Para revisar la odometría:

```bash
ros2 topic echo /odom
```
