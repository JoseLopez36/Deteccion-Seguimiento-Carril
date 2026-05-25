#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from cv_bridge import CvBridge

import json

from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image, CameraInfo

from lane_detection import LaneDetector, LaneConfig


class LaneDetectionNode(Node):
    """
    Nodo de detección de carriles mediante visión clásica (Hough + OpenCV).

    Suscripciones:
      - /carla/ego_vehicle/rgb_front/image       (sensor_msgs/Image)
      - /carla/ego_vehicle/rgb_front/camera_info (sensor_msgs/CameraInfo)

    Publicaciones:
      - /lane_detection/lane_error      (std_msgs/Float32)  — error lateral en metros
      - /lane_detection/lane_state      (std_msgs/String)   — JSON con estado del carril
    """

    def __init__(self):
        super().__init__('lane_detection_node')

        # --- Parámetros de tópicos ---
        self.declare_parameter('image_topic', '/carla/ego_vehicle/rgb_front/image')
        self.declare_parameter('camera_info_topic', '/carla/ego_vehicle/rgb_front/camera_info')
        self.declare_parameter('lane_error_topic', '/lane_detection/lane_error')
        self.declare_parameter('lane_state_topic', '/lane_detection/lane_state')

        self.image_topic        = self.get_parameter('image_topic').value
        self.camera_info_topic  = self.get_parameter('camera_info_topic').value
        self.lane_error_topic   = self.get_parameter('lane_error_topic').value
        self.lane_state_topic   = self.get_parameter('lane_state_topic').value

        # --- Parámetros del detector ---
        self.declare_parameter('canny_low',        50)
        self.declare_parameter('canny_high',       150)
        self.declare_parameter('hough_rho',        2)
        self.declare_parameter('hough_threshold',  50)
        self.declare_parameter('hough_min_len',    40)
        self.declare_parameter('hough_max_gap',    100)
        self.declare_parameter('min_slope',        0.4)
        self.declare_parameter('smoothing',        8)
        self.declare_parameter('horizon',          0.58)
        self.declare_parameter('center_threshold', 0.20)

        cfg = LaneConfig(
            canny_low        = int(self.get_parameter('canny_low').value),
            canny_high       = int(self.get_parameter('canny_high').value),
            hough_rho        = int(self.get_parameter('hough_rho').value),
            hough_threshold  = int(self.get_parameter('hough_threshold').value),
            hough_min_len    = int(self.get_parameter('hough_min_len').value),
            hough_max_gap    = int(self.get_parameter('hough_max_gap').value),
            min_slope        = float(self.get_parameter('min_slope').value),
            smoothing        = int(self.get_parameter('smoothing').value),
            horizon          = float(self.get_parameter('horizon').value),
            center_threshold = float(self.get_parameter('center_threshold').value),
        )

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

        # --- Intrínsecos de la cámara (se rellenan al recibir camera_info) ---
        self._fx = None  # focal length en píxeles; None hasta recibir camera_info

        # --- Detector de carril (OpenCV/Hough) ---
        self.bridge    = CvBridge()
        self._detector = LaneDetector(cfg=cfg)

        # --- Publicadores ---
        self.error_pub = self.create_publisher(Float32, self.lane_error_topic, reliable_qos)
        self.state_pub = self.create_publisher(String, self.lane_state_topic, reliable_qos)

        # --- Suscriptores ---
        self.create_subscription(Image, self.image_topic, self._on_image, sensor_qos)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._on_camera_info, sensor_qos)

        self.get_logger().info('lane_detection_node iniciado.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_camera_info(self, msg: CameraInfo):
        if self._fx is None:
            self._fx = msg.k[0]  # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.get_logger().info(f'camera_info recibido: fx={self._fx:.2f} px')

    def _on_image(self, msg: Image):
        self.get_logger().debug('Received image')
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = frame.shape[:2]
            self.get_logger().debug(f'Frame: {w}x{h}')

            left, right = self._detector._detect(frame, h, w)
            lane_state  = self._detector._analyze(w, left, right)

            lateral_error_px = float(lane_state.offset_px)

            # Convertir a metros si ya se recibió camera_info
            if self._fx is not None and self._fx > 0.0:
                lateral_error_m = lateral_error_px / self._fx
            else:
                lateral_error_m = lateral_error_px  # fallback hasta recibir camera_info

            error_msg = Float32()
            error_msg.data = float(lateral_error_m)
            self.error_pub.publish(error_msg)
            self.get_logger().debug(f'Published lane_error: {lateral_error_m:.4f}m (raw: {lateral_error_px:.1f}px)')

            state_dict = {
                'zone':           lane_state.zone,
                'lateral':        lane_state.lateral,
                'offset_px':      lateral_error_px,
                'lane_width_px':  lane_state.lane_width_px,
                'left_detected':  lane_state.left_detected,
                'right_detected': lane_state.right_detected,
                'left':           list(left)  if left  else None,
                'right':          list(right) if right else None,
                'image_width':    w,
                'image_height':   h,
            }
            state_msg = String()
            state_msg.data = json.dumps(state_dict)
            self.state_pub.publish(state_msg)
            self.get_logger().debug(f'Published lane_state: {state_msg.data}')

        except Exception as e:
            self.get_logger().error(f'Error in _on_image: {e}')


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