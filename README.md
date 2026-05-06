# Detección de cambio de carril y seguimiento

Repositorio para el trabajo de la asignatura **Control en Vehículos** del **MIERA** de la **Universidad de Sevilla**.

## Idea del proyecto

El objetivo es simular un sistema de detección y mantenimiento de carril utilizando cámaras virtuales sobre un vehículo simulado en ROS2 Humble y Gazebo.

El flujo previsto es:

1. Preprocesamiento de imagen y detección de líneas de carril mediante transformada de Hough.
2. Estimación del error lateral o `cross-track error`.
3. Control en cascada:
   - Bucle externo de visión para calcular la referencia de `yaw rate`.
   - Bucle interno para seguimiento de `yaw rate`.
4. Validación del seguimiento de carril en un entorno simulado.

## Equipo

- Agustín Mayor
- Rafael Muñoz
- José Francisco López

## Stack técnico

- Ubuntu 22.04
- Docker
- Autoware Core para ROS2 Humble
- Gazebo
- Python
- OpenCV
- MATLAB/Simulink

## Puesta en marcha

Descargar la imagen oficial de Autoware Core para ROS2 Humble:

```bash
docker pull ghcr.io/autowarefoundation/autoware:core-humble
```

Entrar en el contenedor:

```bash
./run_docker.sh
```

Compilar el workspace ROS2 dentro del contenedor:

```bash
colcon build --symlink-install
```

Realizar el source del workspace:

```bash
source install/setup.bash
```

Lanzar la simulación base dentro del contenedor:

```bash
ros2 launch deteccion_seguimiento_carril_sim sim.launch.py
```

Más detalles en `docs/docker.md`.