# Detección de cambio de carril y seguimiento

Repositorio para el trabajo de la asignatura **Control en Vehículos** del máster **MIERA** de la **Universidad de Sevilla**.

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
- ROS2 Humble
- Gazebo
- Pixi
- Python
- OpenCV
- MATLAB/Simulink

## Puesta en marcha

Instalar Pixi si no está disponible:

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

Preparar el entorno:

```bash
pixi install
```

Compilar el workspace ROS2:

```bash
pixi run build
```

Cargar el entorno compilado:

```bash
source install/setup.bash
```

Lanzar la simulación base:

```bash
pixi run sim
```