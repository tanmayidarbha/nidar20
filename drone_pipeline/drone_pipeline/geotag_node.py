import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from drone_msgs.msg import RawDetection, Detection


CAMERA_HFOV_DEG = 81.0
CAMERA_VFOV_DEG = 65.0


class GeotagNode(Node):

    def __init__(self):
        super().__init__('geotag_node')

        # Rudra is the scout.
        self.declare_parameter(
            'drone_id',
            'rudra'
        )

        self.drone_id = (
            self.get_parameter('drone_id')
            .get_parameter_value()
            .string_value
        )

        self.current_lat = None
        self.current_lon = None
        self.current_alt = None
        self.current_heading = None

        mavros_prefix = (
            f'/{self.drone_id}'
        )

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------

        self.create_subscription(
            NavSatFix,
            f'{mavros_prefix}/global_position/global',
            self.gps_callback,
            10
        )

        self.create_subscription(
            Float64,
            f'{mavros_prefix}/global_position/rel_alt',
            self.alt_callback,
            10
        )

        self.create_subscription(
            Float64,
            f'{mavros_prefix}/global_position/compass_hdg',
            self.heading_callback,
            10
        )

        self.create_subscription(
            RawDetection,
            f'/{self.drone_id}/raw_detections',
            self.detection_callback,
            10
        )

        # ---------------------------------------------------------
        # Publisher
        # ---------------------------------------------------------

        self.publisher_ = self.create_publisher(
            Detection,
            f'/{self.drone_id}/detections',
            10
        )

        self.get_logger().info(
            f'Geotag node started for "{self.drone_id}"'
        )

    # =============================================================
    # TELEMETRY
    # =============================================================

    def gps_callback(self, msg):

        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def alt_callback(self, msg):

        self.current_alt = msg.data

    def heading_callback(self, msg):

        self.current_heading = msg.data

    # =============================================================
    # DETECTION
    # =============================================================

    def detection_callback(self, msg):

        if None in (
            self.current_lat,
            self.current_lon,
            self.current_alt,
            self.current_heading
        ):

            self.get_logger().warn(
                'No GPS/altitude/heading yet. '
                'Skipping geotag.'
            )

            return

        target_lat, target_lon = (
            self.compute_geotag(
                msg.pixel_x,
                msg.pixel_y,
                msg.image_width,
                msg.image_height,
                self.current_lat,
                self.current_lon,
                self.current_alt,
                self.current_heading
            )
        )

        out = Detection()

        out.header = msg.header
        out.drone_id = self.drone_id
        out.latitude = target_lat
        out.longitude = target_lon
        out.altitude = self.current_alt
        out.confidence = msg.confidence
        out.class_name = msg.class_name
        out.detection_id = msg.detection_id

        self.publisher_.publish(
            out
        )

        self.get_logger().info(
            f'[{self.drone_id}] '
            f'Geotagged detection #{msg.detection_id}: '
            f'({target_lat:.7f}, {target_lon:.7f})'
        )

    # =============================================================
    # GEOLOCATION
    # =============================================================

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

        # Horizontal angular offset.
        angle_x = (
            (pixel_x - img_w / 2.0)
            / img_w
        ) * CAMERA_HFOV_DEG

        # Vertical angular offset.
        angle_y = (
            (pixel_y - img_h / 2.0)
            / img_h
        ) * CAMERA_VFOV_DEG

        # Camera orientation convention:
        # image centre = optical centre
        # right = camera right
        # forward = camera forward
        offset_right_m = (
            altitude
            * math.tan(
                math.radians(angle_x)
            )
        )

        offset_forward_m = (
            altitude
            * math.tan(
                math.radians(-angle_y)
            )
        )

        # Rotate camera-frame offsets into
        # North/East using drone heading.
        heading_rad = math.radians(
            heading_deg
        )

        north_offset = (
            offset_forward_m
            * math.cos(heading_rad)
            -
            offset_right_m
            * math.sin(heading_rad)
        )

        east_offset = (
            offset_forward_m
            * math.sin(heading_rad)
            +
            offset_right_m
            * math.cos(heading_rad)
        )

        meters_per_degree_lat = 111320.0

        meters_per_degree_lon = (
            111320.0
            * math.cos(
                math.radians(drone_lat)
            )
        )

        target_lat = (
            drone_lat
            + north_offset
            / meters_per_degree_lat
        )

        target_lon = (
            drone_lon
            + east_offset
            / meters_per_degree_lon
        )

        return target_lat, target_lon


def main(args=None):

    rclpy.init(args=args)

    node = GeotagNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
