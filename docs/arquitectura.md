# Arquitectura del proyecto

## Objetivo funcional

El sistema simulado debe permitir evaluar un algoritmo de seguimiento de carril basado en visión y control en cascada.

## Bloques principales

1. **Simulación Gazebo**
   - Vehículo simulado.
   - Cámara frontal virtual.
   - Entorno con carretera y marcas de carril.
   - Publicación de sensores y estado del vehículo.

2. **Percepción**
   - Captura de imagen desde la cámara virtual.
   - Preprocesamiento.
   - Detección de líneas de carril mediante transformada de Hough.
   - Cálculo del error lateral respecto al centro del carril.

3. **Control**
   - Bucle externo: convierte el error lateral en referencia de `yaw rate`.
   - Bucle interno: sigue la referencia de `yaw rate` actuando sobre la dirección o mando equivalente del vehículo.

4. **Validación**
   - Registro de variables relevantes.
   - Comparación entre trayectoria deseada y trayectoria seguida.
   - Métricas de error lateral, estabilidad y suavidad de control.

## Interfaces ROS2 previstas

- `/camera/image_raw`: imagen de cámara frontal.
- `/lane_detection/debug_image`: imagen con líneas detectadas.
- `/lane_following/cross_track_error`: error lateral estimado.
- `/lane_following/yaw_rate_reference`: referencia de velocidad de guiñada.
- `/cmd_vel` o interfaz equivalente del vehículo: mando de control.
- `/odom`: odometría del vehículo.

## Decisiones pendientes

- Modelo final del vehículo.
- Interfaz exacta de control.
- Mundo Gazebo definitivo.
- Reparto de nodos ROS2 entre los miembros del equipo.