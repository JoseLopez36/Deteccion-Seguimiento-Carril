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

## Puesta en marcha

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

## AWSIM

AWSIM permite ejecutar una simulación fotorrealista conectada con Autoware. Requiere una GPU NVIDIA RTX y drivers NVIDIA compatibles. Descargar AWSIM Demo y el mapa de Shinjuku en [AWSIM Quick Start Demo](https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo/).

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

Más info: [AWSIM Quick Start Demo](https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo/)