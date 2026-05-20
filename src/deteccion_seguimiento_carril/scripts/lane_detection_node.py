#!/usr/bin/env python3

import json
import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from cv_bridge import CvBridge

from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(__file__))
from lane_events import LaneDetector


class LaneDetectionNode(Node):
    """
    Nodo de detección de carriles mediante OpenCV y transformada de Hough.
    Utiliza el módulo lane_events (LaneDetector) como librería de detección.

    Suscripciones:
      - /carla/ego_vehicle/rgb_front/image  (sensor_msgs/Image)

    Publicaciones:
      - /lane_detection/lane_error   (std_msgs/Float32)  — error lateral en píxeles (offset_px)
      - /lane_detection/lane_state   (std_msgs/String)   — JSON con líneas y estado completo
    """

    def __init__(self):
        super().__init__('lane_detection_node')

        # --- Parámetros ---
        self.declare_parameter('image_topic',      '/carla/ego_vehicle/rgb_front/image')
        self.declare_parameter('lane_error_topic', '/lane_detection/lane_error')
        self.declare_parameter('lane_state_topic', '/lane_detection/lane_state')

        self.image_topic      = self.get_parameter('image_topic').value
        self.lane_error_topic = self.get_parameter('lane_error_topic').value
        self.lane_state_topic = self.get_parameter('lane_state_topic').value

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

        # --- Utilidades ---
        self.bridge = CvBridge()

        # --- Detector (usado como librería: sin hilo ni VideoCapture) ---
        self._detector = LaneDetector(show_window=False)

        # --- Publicadores ---
        self.error_pub = self.create_publisher(Float32, self.lane_error_topic, reliable_qos)
        self.state_pub = self.create_publisher(String,  self.lane_state_topic, reliable_qos)

        # --- Suscriptores ---
        self.create_subscription(Image, self.image_topic, self._on_image, sensor_qos)

        self.get_logger().info('lane_detection_node iniciado.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w  = frame.shape[:2]

        left, right = self._detector._detect(frame, h, w)
        state       = self._detector._analyze(w, left, right, fps=0.0)

        if state.zone == 'UNKNOWN':
            self.get_logger().warn('LANE_LOST: no se detectan líneas de carril', throttle_duration_sec=2.0)
        else:
            self.get_logger().info(
                f'zone={state.zone}  lateral={state.lateral:.2f}  '
                f'offset={state.offset_px:+d}px  '
                f'L={int(state.left_detected)}  R={int(state.right_detected)}',
                throttle_duration_sec=1.0,
            )

        # Publicar error lateral
        error_msg      = Float32()
        error_msg.data = float(state.offset_px)
        self.error_pub.publish(error_msg)

        # Publicar estado completo como JSON
        state_payload = {
            'zone':           state.zone,
            'lateral':        state.lateral,
            'offset_px':      state.offset_px,
            'lane_width_px':  state.lane_width_px,
            'left_detected':  state.left_detected,
            'right_detected': state.right_detected,
            'left':           list(left)  if left  else None,
            'right':          list(right) if right else None,
            'image_width':    w,
            'image_height':   h,
        }
        state_msg      = String()
        state_msg.data = json.dumps(state_payload)
        self.state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()