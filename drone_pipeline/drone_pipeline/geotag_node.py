import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from drone_msgs.msg import RawDetection, Detection


CAMERA_HFOV_DEG = 81.0
CAMERA_VFOV_DEG = 65.0


class GeotagNode(Node):

    def __init__(self):
        super().__init__('geotag_node')

        self.declare_parameter('drone_id', 'barbarik')

        self.drone_id = (
            self.get_parameter('drone_id')
            .get_parameter_value()
            .string_value
        )

        self.current_lat = None
        self.current_lon = None
        self.current_alt = None
        self.current_heading = None

        # MAVROS telemetry uses BEST_EFFORT
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        mavros_prefix = f'/{self.drone_id}'

        raw_topic = f'/{self.drone_id}/raw_detections'
        final_topic = f'/{self.drone_id}/detections'

        # --------------------------------------------------
        # TELEMETRY
        # --------------------------------------------------

        self.create_subscription(
            NavSatFix,
            f'{mavros_prefix}/global_position/global',
            self.gps_callback,
            qos
        )

        self.create_subscription(
            Float64,
            f'{mavros_prefix}/global_position/rel_alt',
            self.alt_callback,
            qos
        )

        self.create_subscription(
            Float64,
            f'{mavros_prefix}/global_position/compass_hdg',
            self.heading_callback,
            qos
        )

        # --------------------------------------------------
        # DETECTION INPUT
        # --------------------------------------------------

        self.create_subscription(
            RawDetection,
            raw_topic,
            self.detection_callback,
            10
        )

        # --------------------------------------------------
        # GEOTAGGED DETECTION OUTPUT
        # --------------------------------------------------

        self.publisher_ = self.create_publisher(
            Detection,
            final_topic,
            10
        )

        self.get_logger().info(
            f'Geotag node started for "{self.drone_id}" '
            f'using {mavros_prefix}'
        )

    # ======================================================
    # TELEMETRY CALLBACKS
    # ======================================================

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def alt_callback(self, msg):
        self.current_alt = msg.data

    def heading_callback(self, msg):
        self.current_heading = msg.data

    # ======================================================
    # DETECTION CALLBACK
    # ======================================================

    def detection_callback(self, msg):

        self.get_logger().info(
            f'RECEIVED RAW DETECTION: '
            f'id={msg.detection_id}, '
            f'pixel=({msg.pixel_x}, {msg.pixel_y}), '
            f'class={msg.class_name}'
        )

        # Need complete telemetry before geotagging
        if None in (
            self.current_lat,
            self.current_lon,
            self.current_alt,
            self.current_heading
        ):
            self.get_logger().warn(
                'No GPS/altitude/heading yet, '
                'skipping geotag for this detection'
            )
            return

        target_lat, target_lon = self.compute_geotag(
            msg.pixel_x,
            msg.pixel_y,
            msg.image_width,
            msg.image_height,
            self.current_lat,
            self.current_lon,
            self.current_alt,
            self.current_heading
        )

        # Create final Detection message
        out = Detection()

        out.header = msg.header
        out.drone_id = self.drone_id
        out.latitude = target_lat
        out.longitude = target_lon
        out.altitude = self.current_alt
        out.confidence = msg.confidence
        out.class_name = msg.class_name
        out.detection_id = msg.detection_id

        self.publisher_.publish(out)

        self.get_logger().info(
            f'[{self.drone_id}] '
            f'Geotagged detection #{msg.detection_id}: '
            f'({target_lat:.6f}, {target_lon:.6f})'
        )

    # ======================================================
    # PIXEL → GPS
    # ======================================================

    def compute_geotag(
        self,
        pixel_x,
        pixel_y,
        img_w,
        img_h,
        drone_lat,
        drone_lon,
        altitude,
        heading_deg
    ):

        # Horizontal and vertical angular resolution
        deg_per_pixel_x = CAMERA_HFOV_DEG / img_w
        deg_per_pixel_y = CAMERA_VFOV_DEG / img_h

        # Image centre
        dx = pixel_x - (img_w / 2.0)
        dy = pixel_y - (img_h / 2.0)

        # Angular offset from image centre
        angle_x = dx * deg_per_pixel_x
        angle_y = dy * deg_per_pixel_y

        # Convert angular offset to ground distance
        offset_right_m = (
            altitude *
            math.tan(math.radians(angle_x))
        )

        offset_forward_m = (
            altitude *
            math.tan(math.radians(-angle_y))
        )

        # Rotate camera-relative offsets into
        # North/East using drone heading
        heading_rad = math.radians(heading_deg)

        north_offset = (
            offset_forward_m * math.cos(heading_rad)
            -
            offset_right_m * math.sin(heading_rad)
        )

        east_offset = (
            offset_forward_m * math.sin(heading_rad)
            +
            offset_right_m * math.cos(heading_rad)
        )

        # Convert metres to latitude/longitude
        meters_per_deg_lat = 111320.0

        meters_per_deg_lon = (
            111320.0 *
            math.cos(math.radians(drone_lat))
        )

        target_lat = (
            drone_lat +
            north_offset / meters_per_deg_lat
        )

        target_lon = (
            drone_lon +
            east_offset / meters_per_deg_lon
        )

        return target_lat, target_lon


def main(args=None):

    rclpy.init(args=args)

    node = GeotagNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
