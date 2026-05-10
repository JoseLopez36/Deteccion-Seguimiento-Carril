# Detección de cambio de carril y seguimiento

Repositorio para el trabajo de la asignatura **Control en Vehículos** del **MIERA** de la **Universidad de Sevilla**.

## Idea del proyecto

El objetivo es simular un sistema de detección y mantenimiento de carril utilizando cámaras virtuales sobre un vehículo simulado en ROS2 Jazzy y Gazebo.

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

## Instalación

### 1. Preparar Autoware en el host

Clonar Autoware:

```bash
git clone https://github.com/autowarefoundation/autoware.git ~/autoware
cd ~/autoware
```

Preparar las herramientas de instalación del Docker de Autoware:

```bash
bash ansible/scripts/install-ansible.sh
ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml
ansible-playbook autoware.dev_env.install_docker -K
```

Más info: [Documentación de instalación de Autoware con Docker](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/)

### 2. Descargar los datos de planificación

Descargar el mapa `sample-map-planning` y los modelos ML en `~/autoware_data`:

```bash
mkdir ~/autoware_data
cd ~/autoware_data
mkdir maps ml_models
ansible-playbook autoware.dev_env.install_dev_env --tags demo_artifacts --ask-become-pass
ansible-playbook autoware.dev_env.install_dev_env --tags ml_models --ask-become-pass
```

El script deja los datos en:

```text
~/autoware_data/maps/sample-map-planning/
~/autoware_data/ml_models/
```

Más info: [Documentación de planning simulation](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/)

### 3. Descargar y ejecutar el contenedor

Descargar la imagen de Autoware para ROS2 Jazzy:

```bash
docker pull ghcr.io/autowarefoundation/autoware:universe-cuda-jazzy
```

Ejecutar el contenedor:

```bash
./tools/run_docker.sh
```

Probar el simulador de planificación de Autoware:

```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_data/maps/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

### 4. Instalar y ejecutar AWSIM

AWSIM permite ejecutar una simulación fotorrealista conectada con Autoware. Requiere una GPU NVIDIA RTX y drivers NVIDIA compatibles. Descargar AWSIM Demo y el mapa de Shinjuku en [AWSIM Quick Start Demo](https://autowarefoundation.github.io/AWSIM/GettingStarted/QuickStartDemo/).

Ejecutar AWSIM desde el host:

```bash
./AWSIM-demo.x86_64 --json_path AWSIM-config.json
```

Lanzar Autoware conectado a AWSIM:

```bash
xhost +local:docker

cd tools/
HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose run --rm awsim
```

Más info: [AWSIM Quick Start Demo](https://autowarefoundation.github.io/AWSIM/GettingStarted/QuickStartDemo/)

## Puesta en marcha

Compilar el paquete ROS2 dentro del contenedor de desarrollo:

```bash
./tools/run_docker.sh
```

```bash
cd /home/aw/workspace
rm -rf build/deteccion_seguimiento_carril install/deteccion_seguimiento_carril log
colcon build --symlink-install --packages-select deteccion_seguimiento_carril
source install/setup.bash
```

Con AWSIM ya ejecutándose, lanzar el sistema de seguimiento de carril usando Docker Compose:

```bash
cd tools/
HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose run --rm deteccion_seguimiento_carril
```

El servicio `deteccion_seguimiento_carril` de `tools/docker-compose.yaml` monta el workspace, activa `/opt/autoware/setup.bash`, activa `install/setup.bash` y ejecuta `ros2 launch deteccion_seguimiento_carril run.launch.py`.

El nodo se suscribe a la cámara de AWSIM en `/sensing/camera/traffic_light/image_raw` y publica comandos de control en `/control/command/control_cmd`.
