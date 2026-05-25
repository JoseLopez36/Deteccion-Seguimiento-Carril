# Detección de carril y seguimiento

Repositorio para el trabajo de la asignatura **Control en Vehículos** del **MIERA** — Universidad de Sevilla.

## Descripción

Sistema de percepción y control para un vehículo autónomo simulado en **CARLA Simulator** con **ROS2 Humble**:

1. Detección de líneas de carril mediante transformada de Hough (OpenCV).
2. Estimación del error lateral (`cross-track error`) y conversión a metros usando los intrínsecos de cámara.
3. Control PID en cascada: bucle de velocidad (crucero) + bucle de dirección (mantenimiento de carril).

## Equipo

- Agustín Mayor
- Rafael Muñoz
- José Francisco López

## Estructura del paquete

```
src/deteccion_seguimiento_carril/
├── config/
│   └── params.yaml                  # Parámetros de todos los nodos
├── launch/
│   ├── run.launch.py                # Lanzamiento principal
│   └── lane_dataset.launch.py       # Recolección de dataset
└── scripts/
    ├── lane_detection.py            # Librería de detección (Hough + OpenCV)
    ├── lane_detection_node.py       # Nodo ROS2: imagen → error lateral
    ├── vehicle_control_node.py      # Nodo ROS2: error lateral → control PID
    ├── annotation_generator_node.py # Nodo ROS2: anotaciones visuales para Foxglove
    └── lane_dataset_node.py         # Nodo ROS2: recolección de dataset con ground truth semántico
```

## Instalación

### NVIDIA Container Toolkit

```bash
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### CARLA Simulator

```bash
docker pull carlasim/carla:0.9.15

# Con ventana
xhost +local:docker
docker run --rm --privileged --gpus all --net=host \
  -e DISPLAY=$DISPLAY -e XDG_RUNTIME_DIR=/tmp/runtime-carla \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --user $(id -u):$(id -g) --workdir /home/carla \
  -it carlasim/carla:0.9.15 \
  ./CarlaUE4.sh -windowed -carla-rpc-port=2001 -nosound

# Headless
docker run --rm --privileged --gpus all --net=host \
  --user $(id -u):$(id -g) --workdir /home/carla \
  -it carlasim/carla:0.9.15 \
  ./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2001 -nosound
```

### Imagen Docker del proyecto

```bash
docker build -t deteccion_seguimiento_carril docker/
```

## Puesta en marcha

Con CARLA corriendo, lanzar todos los servicios desde la raíz del repositorio:

```bash
xhost +local:docker
docker compose -f tools/docker-compose.yaml up
```

| Servicio | Contenedor | Descripción |
|---|---|---|
| `ros_bridge` | `carla_ros_bridge` | Bridge CARLA → ROS2 + spawn del ego-vehicle |
| `navegacion` | `deteccion_seguimiento_carril` | Compila y lanza `run.launch.py` |
| `foxglove` | `foxglove_bridge` | WebSocket en `ws://localhost:8765` para Foxglove Studio |

Para lanzar servicios por separado:

```bash
docker compose -f tools/docker-compose.yaml up ros_bridge
docker compose -f tools/docker-compose.yaml up navegacion
docker compose -f tools/docker-compose.yaml up foxglove
```

Shell interactiva en un contenedor:

```bash
docker exec -it carla_ros_bridge bash
docker exec -it deteccion_seguimiento_carril bash
docker exec -it foxglove_bridge bash
```

## Visualización con Foxglove Studio

1. Abre [Foxglove Studio](https://studio.foxglove.dev) (navegador o app).
2. **Open connection → Foxglove WebSocket** → `ws://172.17.0.1:8765`
3. **View → Import layout from file** → selecciona `tools/foxglove.json`

El layout incluye:
- **Izquierda**: `/carla/ego_vehicle/rgb_view/image` — vista exterior
- **Derecha**: `/carla/ego_vehicle/rgb_front/image` — cámara frontal con anotaciones

## Generación de Dataset

Recolección de imágenes con ground truth semántico de CARLA en modo conducción manual (`W/A/S/D`, activar con `B`):

```bash
xhost +local:docker
docker compose -f tools/docker-compose-lane-dataset.yaml up
```

- **Salida**: `dataset/lanes/images/` (RGB) y `dataset/lanes/masks/` (máscaras binarias)

### Visualización del dataset

```bash
python3 tools/visualize_lanes_dataset.py dataset/lanes
python3 tools/visualize_lanes_dataset.py dataset/lanes --slideshow --delay 1000
python3 tools/visualize_lanes_dataset.py dataset/lanes --output dataset/lanes/visualized
```

En Docker:

```bash
xhost +local:docker
docker run --rm -it --network host -e DISPLAY=${DISPLAY} \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd)/dataset/lanes:/dataset:ro \
  -v $(pwd)/tools:/tools:ro \
  deteccion_seguimiento_carril \
  python3 /tools/visualize_lanes_dataset.py /dataset
```