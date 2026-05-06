# Simulación en ROS2 Humble y Gazebo

## Objetivo

Levantar una simulación reproducible con un vehículo equipado con cámara frontal circulando en un entorno con marcas de carril.

## Componentes iniciales

- Mundo Gazebo con carretera recta y marcas de carril.
- Modelo URDF/Xacro del vehículo.
- Cámara frontal simulada.
- Launch file para arrancar Gazebo, publicar el robot y cargar el mundo.

## Comandos previstos con Docker

Descargar la imagen de Autoware Core para ROS2 Humble:

```bash
docker pull ghcr.io/autowarefoundation/autoware:core-humble
```

Permitir acceso gráfico desde el contenedor:

```bash
xhost +local:docker
```

Entrar en el contenedor:

```bash
docker compose run --rm autoware-core
```

Compilar dentro del contenedor:

```bash
./scripts/docker_build.sh
```

Lanzar la simulación dentro del contenedor:

```bash
./scripts/docker_sim.sh
```

## Próximos pasos técnicos

- Completar el modelo cinemático/dinámico del vehículo.
- Ajustar posición y parámetros de cámara.
- Añadir texturas o geometría de carril más realista.
- Definir la interfaz final para el control de dirección.
