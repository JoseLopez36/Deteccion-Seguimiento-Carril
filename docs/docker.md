# Entorno Docker con Autoware Core

El desarrollo se realizará sobre la imagen oficial de Autoware Core para ROS2 Humble:

```text
ghcr.io/autowarefoundation/autoware:core-humble
```

La documentación oficial indica que el esquema de etiquetas es:

```text
<stage>-<ros_distro>[-<date>|-<version>]
```

Para este proyecto se usa `core-humble`.

## Requisitos previos

- Ubuntu 22.04.
- Docker instalado.
- Acceso a servidor gráfico X11 si se usa Gazebo/RViz.
- Driver NVIDIA y NVIDIA Container Toolkit si se quiere aceleración GPU.

## Descargar la imagen

```bash
docker pull ghcr.io/autowarefoundation/autoware:core-humble
```

## Entrar en el contenedor

Arrancar una shell dentro del contenedor:

```bash
./run_docker.sh
```

El repositorio local queda montado dentro del contenedor en:

```text
/workspace/deteccion-seguimiento-carril
```

## Compilar dentro del contenedor

```bash
colcon build --symlink-install
source install/setup.bash
```

## Lanzar simulación dentro del contenedor

```bash
ros2 launch deteccion_seguimiento_carril_sim sim.launch.py
```

## Referencias

- <https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/core-docker-installation/>
- <https://github.com/autowarefoundation/autoware/blob/main/docker/README.md>