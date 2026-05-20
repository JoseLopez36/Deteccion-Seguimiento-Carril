#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float32
from carla_msgs.msg import CarlaEgoVehicleControl


class VehicleControlNode(Node):
    """
    Nodo de control del vehículo.

    Suscripciones:
      - /lane_detection/lane_error          (std_msgs/Float32)              — error lateral en píxeles
      - /carla/ego_vehicle/speedometer      (std_msgs/Float32)

    Publicaciones:
      - /carla/ego_vehicle/vehicle_control_cmd (carla_msgs/CarlaEgoVehicleControl)
    """

    def __init__(self):
        super().__init__('vehicle_control_node')

        # --- Parámetros ---
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('target_speed', 5.0)          # m/s
        self.declare_parameter('max_steering_angle', 0.5)    # rad
        self.declare_parameter('kp_throttle', 0.3)
        self.declare_parameter('lane_error_topic', '/lane_detection/lane_error')
        self.declare_parameter('speedometer_topic', '/carla/ego_vehicle/speedometer')
        self.declare_parameter('vehicle_control_topic', '/carla/ego_vehicle/vehicle_control_cmd')

        self.control_rate         = float(self.get_parameter('control_rate').value)
        self.target_speed         = float(self.get_parameter('target_speed').value)
        self.max_steering_angle   = float(self.get_parameter('max_steering_angle').value)
        self.kp_throttle          = float(self.get_parameter('kp_throttle').value)
        self.lane_error_topic     = self.get_parameter('lane_error_topic').value
        self.speedometer_topic    = self.get_parameter('speedometer_topic').value
        self.vehicle_control_topic = self.get_parameter('vehicle_control_topic').value

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
        self.lane_error    = 0.0
        self.speed_limit   = self.target_speed
        self.current_speed = 0.0

        # --- Publicadores ---
        self.control_pub = self.create_publisher(
            CarlaEgoVehicleControl,
            self.vehicle_control_topic,
            reliable_qos,
        )

        # --- Suscriptores ---
        self.create_subscription(
            Float32,
            self.lane_error_topic,
            self._on_lane_error,
            reliable_qos,
        )
        self.create_subscription(
            Float32,
            self.speedometer_topic,
            self._on_speedometer,
            sensor_qos,
        )

        # --- Timer de control ---
        self.create_timer(1.0 / self.control_rate, self._control_loop)

        self.get_logger().info('vehicle_control_node iniciado.')

    # ------------------------------------------------------------------
    # Callbacks de suscripción
    # ------------------------------------------------------------------

    def _on_lane_error(self, msg: Float32):
        self.lane_error = msg.data

    def _on_speedometer(self, msg: Float32):
        self.current_speed = msg.data

    # ------------------------------------------------------------------
    # Bucle de control
    # ------------------------------------------------------------------

    def _control_loop(self):
        # Control de crucero
        speed_error = self.speed_limit - self.current_speed
        throttle = float(max(0.0, min(1.0, self.kp_throttle * speed_error)))
        brake = 0.0

        # TODO: Control de dirección
        target_steering = 0.0
        steering = float(max(-self.max_steering_angle, min(self.max_steering_angle, target_steering)))

        # Aplicar control
        cmd = CarlaEgoVehicleControl()
        cmd.hand_brake = False
        cmd.reverse = False
        cmd.manual_gear_shift = False
        cmd.throttle = throttle
        cmd.brake = brake
        cmd.steer = steering

        self.control_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()