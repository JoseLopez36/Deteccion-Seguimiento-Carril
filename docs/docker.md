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
- Docker Compose disponible.
- Acceso a servidor gráfico X11 si se usa Gazebo/RViz.
- Driver NVIDIA y NVIDIA Container Toolkit si se quiere aceleración GPU.

## Descargar la imagen

```bash
docker pull ghcr.io/autowarefoundation/autoware:core-humble
```

## Entrar en el contenedor

Permitir acceso X11 al contenedor:

```bash
xhost +local:docker
```

Arrancar una shell dentro del contenedor:

```bash
docker compose run --rm autoware-core
```

El repositorio local queda montado dentro del contenedor en:

```text
/workspace/deteccion-seguimiento-carril
```

## Compilar dentro del contenedor

```bash
./scripts/docker_build.sh
```

## Lanzar simulación dentro del contenedor

```bash
./scripts/docker_sim.sh
```

## Referencias

- <https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/core-docker-installation/>
- <https://github.com/autowarefoundation/autoware/blob/main/docker/README.md>