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

## Puesta en marcha

### 1. Preparar Autoware en el host

Clonar Autoware v1.8.0:

```bash
git clone https://github.com/autowarefoundation/autoware.git ~/autoware
cd ~/autoware
git checkout 1.8.0
```

Preparar las herramientas de instalación del Docker de Autoware:

```bash
bash ansible/scripts/install-ansible.sh
ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml
ansible-playbook autoware.dev_env.install_docker -K
```

Más info: [Documentación de instalación de Autoware con Docker](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/)

### 2. Descargar y ejecutar el contenedor

Descargar la imagen de Autoware v1.8.0 para ROS2 Humble:

```bash
docker pull ghcr.io/autowarefoundation/autoware:universe-cuda-humble-1.8.0
```

Ejecutar el contenedor:

```bash
./docker/run_docker.sh
```

Probar el simulador de planificación de Autoware:

```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_data/maps/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```