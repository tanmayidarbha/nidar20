import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from drone_msgs.msg import RawDetection, Detection


# =============================================================
# CAMERA CALIBRATION
# =============================================================

# SIYI A8 Mini
#
# Calibration resolution: 1280 x 720
#
# Camera matrix:
#
# [ fx   0   cx ]
# [  0  fy   cy ]
# [  0   0    1 ]
#
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

FX = 724.967
FY = 725.028

CX = 626.364
CY = 352.733

CAMERA_MATRIX = np.array(
    [
        [FX, 0.0, CX],
        [0.0, FY, CY],
        [0.0, 0.0, 1.0]
    ],
    dtype=np.float64
)

# Distortion coefficients:
# [k1, k2, p1, p2, k3]
DIST_COEFFS = np.array(
    [
        -0.12134,
        0.12149,
        0.00351,
        -0.00423,
        -0.00894
    ],
    dtype=np.float64
)


class GeotagNode(Node):

    def __init__(self):

        super().__init__('geotag_node')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

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

        # ---------------------------------------------------------
        # Latest telemetry
        # ---------------------------------------------------------

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

        self.get_logger().info(
            'Using calibrated camera geometry for 1280x720.'
        )

        self.get_logger().info(
            'Camera assumed to be fixed straight downward.'
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

        # ---------------------------------------------------------
        # Make sure telemetry is available
        # ---------------------------------------------------------

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

        # ---------------------------------------------------------
        # Check image resolution
        # ---------------------------------------------------------

        if (
            msg.image_width != CAMERA_WIDTH
            or msg.image_height != CAMERA_HEIGHT
        ):

            self.get_logger().warn(
                f'Image resolution is '
                f'{msg.image_width}x{msg.image_height}, '
                f'but camera calibration is for '
                f'{CAMERA_WIDTH}x{CAMERA_HEIGHT}. '
                f'Skipping geotag.'
            )

            return

        # ---------------------------------------------------------
        # Calculate survivor GPS position
        # ---------------------------------------------------------

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

        # ---------------------------------------------------------
        # Create geotagged detection
        # ---------------------------------------------------------

        out = Detection()

        # Preserve original detection information.
        out.header = msg.header
        out.drone_id = self.drone_id

        # Calculated survivor location.
        out.latitude = target_lat
        out.longitude = target_lon

        # Drone altitude at time of detection.
        out.altitude = self.current_alt

        # Preserve detection information.
        out.confidence = msg.confidence
        out.class_name = msg.class_name
        out.detection_id = msg.detection_id

        # ---------------------------------------------------------
        # Publish
        # ---------------------------------------------------------

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

        # ---------------------------------------------------------
        # Convert detected pixel into an undistorted camera ray.
        # ---------------------------------------------------------

        point = np.array(
            [
                [
                    [float(pixel_x), float(pixel_y)]
                ]
            ],
            dtype=np.float64
        )

        undistorted = cv2.undistortPoints(
            point,
            CAMERA_MATRIX,
            DIST_COEFFS
        )

        # Normalized camera coordinates.
        #
        # x = camera right
        # y = image downward
        # z = camera optical axis
        #
        normalized_x = undistorted[0, 0, 0]
        normalized_y = undistorted[0, 0, 1]

        # ---------------------------------------------------------
        # Convert camera ray to ground offsets.
        #
        # Camera is assumed to point straight downward.
        #
        # Positive X:
        #     camera right
        #
        # Positive Y in image:
        #     downward in the image
        #
        # With a downward-facing camera whose top of image
        # corresponds to drone forward:
        #
        #     camera X -> drone right
        #     -camera Y -> drone forward
        # ---------------------------------------------------------

        offset_right_m = (
            altitude * normalized_x
        )

        offset_forward_m = (
            -altitude * normalized_y
        )

        # ---------------------------------------------------------
        # Rotate drone forward/right offsets into North/East
        # using drone heading.
        # ---------------------------------------------------------

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

        # ---------------------------------------------------------
        # Convert metre offsets into latitude/longitude.
        # ---------------------------------------------------------

        meters_per_degree_lat = 111320.0

        meters_per_degree_lon = (
            111320.0
            * math.cos(
                math.radians(drone_lat)
            )
        )

        target_lat = (
            drone_lat
            +
            north_offset
            / meters_per_degree_lat
        )

        target_lon = (
            drone_lon
            +
            east_offset
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
