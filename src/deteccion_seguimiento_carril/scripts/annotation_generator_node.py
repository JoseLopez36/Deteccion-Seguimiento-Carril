#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from builtin_interfaces.msg import Time
from foxglove_msgs.msg import Point2

from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image
from foxglove_msgs.msg import ImageAnnotations, PointsAnnotation, TextAnnotation


class AnnotationGeneratorNode(Node):
    """
    Nodo que genera anotaciones visuales para Foxglove Studio.

    Suscripciones:
      - /carla/ego_vehicle/rgb_front/image  (sensor_msgs/Image)  — timestamp de referencia
      - /lane_detection/lane_error          (std_msgs/Float32)   — error lateral en píxeles
      - /lane_detection/lane_state          (std_msgs/String)    — JSON con líneas y estado

    Publicaciones:
      - /foxglove/annotations  (foxglove_msgs/ImageAnnotations)  — anotaciones para Foxglove
    """

    def __init__(self):
        super().__init__('annotation_generator_node')

        # --- Parámetros ---
        self.declare_parameter('image_topic', '/carla/ego_vehicle/rgb_front/image')
        self.declare_parameter('lane_error_topic', '/lane_detection/lane_error')
        self.declare_parameter('lane_state_topic', '/lane_detection/lane_state')
        self.declare_parameter('annotations_topic', '/foxglove/annotations')
        self.declare_parameter('image_width',  800)
        self.declare_parameter('image_height', 600)

        self.image_topic       = self.get_parameter('image_topic').value
        self.lane_error_topic  = self.get_parameter('lane_error_topic').value
        self.lane_state_topic  = self.get_parameter('lane_state_topic').value
        self.annotations_topic = self.get_parameter('annotations_topic').value
        self.image_width       = int(self.get_parameter('image_width').value)
        self.image_height      = int(self.get_parameter('image_height').value)

        # --- QoS ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- Estado interno ---
        self.lane_error = 0.0
        self.lane_state: dict = {}

        # --- Publicadores ---
        self.annotations_pub = self.create_publisher(
            ImageAnnotations,
            self.annotations_topic,
            reliable_qos,
        )

        # --- Suscriptores ---
        self.create_subscription(Image,   self.image_topic,       self._on_image,      sensor_qos)
        self.create_subscription(Float32, self.lane_error_topic,  self._on_lane_error, reliable_qos)
        self.create_subscription(String,  self.lane_state_topic,  self._on_lane_state, reliable_qos)

        self.get_logger().info('annotation_generator_node iniciado.')

    # ------------------------------------------------------------------
    # Callbacks de estado
    # ------------------------------------------------------------------

    def _on_lane_error(self, msg: Float32):
        self.lane_error = msg.data

    def _on_lane_state(self, msg: String):
        try:
            self.lane_state = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    # ------------------------------------------------------------------
    # Callback principal: se ejecuta por cada frame de cámara
    # ------------------------------------------------------------------

    def _on_image(self, msg: Image):
        annotations = self._build_annotations(msg.header.stamp)
        self.annotations_pub.publish(annotations)

    # ------------------------------------------------------------------
    # Construcción de anotaciones
    # ------------------------------------------------------------------

    def _build_annotations(self, stamp: Time) -> ImageAnnotations:
        ann = ImageAnnotations()
        s   = self.lane_state

        w = float(s.get('image_width',  self.image_width))
        h = float(s.get('image_height', self.image_height))
        cx = w / 2.0

        left  = s.get('left')   # [x_bot, y_bot, x_top, y_top] or None
        right = s.get('right')

        # --- Línea izquierda del carril ---
        if left:
            left_line = PointsAnnotation()
            left_line.timestamp = stamp
            left_line.type      = PointsAnnotation.LINE_STRIP
            left_line.thickness = 4.0
            left_line.outline_color.r = 1.0
            left_line.outline_color.g = 0.39
            left_line.outline_color.b = 0.0
            left_line.outline_color.a = 1.0
            p0 = Point2(); p0.x = float(left[0]); p0.y = float(left[1])
            p1 = Point2(); p1.x = float(left[2]); p1.y = float(left[3])
            left_line.points.extend([p0, p1])
            ann.points.append(left_line)

        # --- Línea derecha del carril ---
        if right:
            right_line = PointsAnnotation()
            right_line.timestamp = stamp
            right_line.type      = PointsAnnotation.LINE_STRIP
            right_line.thickness = 4.0
            right_line.outline_color.r = 0.0
            right_line.outline_color.g = 0.78
            right_line.outline_color.b = 1.0
            right_line.outline_color.a = 1.0
            p0 = Point2(); p0.x = float(right[0]); p0.y = float(right[1])
            p1 = Point2(); p1.x = float(right[2]); p1.y = float(right[3])
            right_line.points.extend([p0, p1])
            ann.points.append(right_line)

        # --- Polígono de relleno del carril (cuando ambas líneas detectadas) ---
        if left and right:
            poly = PointsAnnotation()
            poly.timestamp = stamp
            poly.type      = PointsAnnotation.LINE_LOOP
            poly.thickness = 2.0
            poly.outline_color.r = 0.0
            poly.outline_color.g = 0.71
            poly.outline_color.b = 0.0
            poly.outline_color.a = 0.6
            poly.fill_color.r = 0.0
            poly.fill_color.g = 0.71
            poly.fill_color.b = 0.0
            poly.fill_color.a = 0.15
            pb = Point2(); pb.x = float(left[0]);  pb.y = float(left[1])
            pt = Point2(); pt.x = float(left[2]);  pt.y = float(left[3])
            qt = Point2(); qt.x = float(right[2]); qt.y = float(right[3])
            qb = Point2(); qb.x = float(right[0]); qb.y = float(right[1])
            poly.points.extend([pb, pt, qt, qb])
            ann.points.append(poly)

        # --- Línea de desviación del centro del carril ---
        deviation = PointsAnnotation()
        deviation.timestamp = stamp
        deviation.type      = PointsAnnotation.LINE_STRIP
        deviation.thickness = 2.0
        deviation.outline_color.r = 1.0
        deviation.outline_color.g = 1.0
        deviation.outline_color.b = 0.0
        deviation.outline_color.a = 1.0
        lane_center_x = cx - self.lane_error
        dp0 = Point2(); dp0.x = cx;            dp0.y = h * 0.7
        dp1 = Point2(); dp1.x = lane_center_x; dp1.y = h * 0.7
        deviation.points.extend([dp0, dp1])
        ann.points.append(deviation)

        # --- Texto: zona y error lateral ---
        zone     = s.get('zone',     'UNKNOWN')
        lateral  = s.get('lateral',  0.5)
        offset   = s.get('offset_px', self.lane_error)

        zone_text = TextAnnotation()
        zone_text.timestamp = stamp
        zone_text.position.x = 10.0
        zone_text.position.y = 30.0
        zone_text.text = f'Zona: {zone}'
        zone_text.font_size = 16.0
        zone_text.text_color.r = 0.0
        zone_text.text_color.g = 1.0
        zone_text.text_color.b = 0.4
        zone_text.text_color.a = 1.0
        zone_text.background_color.a = 0.0
        ann.texts.append(zone_text)

        error_text = TextAnnotation()
        error_text.timestamp = stamp
        error_text.position.x = 10.0
        error_text.position.y = 55.0
        error_text.text = f'Lateral: {lateral:.2f}  Offset: {int(offset):+d}px'
        error_text.font_size = 14.0
        error_text.text_color.r = 1.0
        error_text.text_color.g = 1.0
        error_text.text_color.b = 0.0
        error_text.text_color.a = 1.0
        error_text.background_color.a = 0.0
        ann.texts.append(error_text)

        det_text = TextAnnotation()
        det_text.timestamp = stamp
        det_text.position.x = 10.0
        det_text.position.y = 75.0
        det_text.text = f"L:{int(s.get('left_detected', False))}  R:{int(s.get('right_detected', False))}"
        det_text.font_size = 12.0
        det_text.text_color.r = 0.6
        det_text.text_color.g = 0.6
        det_text.text_color.b = 0.6
        det_text.text_color.a = 1.0
        det_text.background_color.a = 0.0
        ann.texts.append(det_text)

        return ann


def main(args=None):
    rclpy.init(args=args)
    node = AnnotationGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()