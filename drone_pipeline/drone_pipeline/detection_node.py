import os

import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from cv_bridge import CvBridge
from ultralytics import YOLO

from drone_msgs.msg import RawDetection


MODEL_PATH = '/home/tanmayi_unix/nidar20_ws/models/best.pt'
CONFIDENCE_THRESHOLD = 0.5

DETECTION_SAVE_DIR = '/home/tanmayi_unix/nidar20_ws/detections'


class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')

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

        self.bridge = CvBridge()

        self.scan_started = False
        self.detection_counter = 0

        # ---------------------------------------------------------
        # Paths
        # ---------------------------------------------------------

        self.save_dir = os.path.join(
            DETECTION_SAVE_DIR,
            self.drone_id
        )

        os.makedirs(
            self.save_dir,
            exist_ok=True
        )

        # ---------------------------------------------------------
        # Topics
        # ---------------------------------------------------------

        image_topic = (
            f'/{self.drone_id}/image_raw'
        )

        scan_start_topic = (
            f'/{self.drone_id}/scan_start'
        )

        detection_topic = (
            f'/{self.drone_id}/raw_detections'
        )

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.scan_start_sub = self.create_subscription(
            Bool,
            scan_start_topic,
            self.scan_start_callback,
            10
        )

        self.detection_pub = self.create_publisher(
            RawDetection,
            detection_topic,
            10
        )

        # ---------------------------------------------------------
        # YOLO
        # ---------------------------------------------------------

        self.get_logger().info(
            f'Loading YOLO model from {MODEL_PATH}'
        )

        self.model = YOLO(
            MODEL_PATH
        )

        self.get_logger().info(
            f'Detection node started for "{self.drone_id}"'
        )

    # =============================================================
    # SCAN START
    # =============================================================

    def scan_start_callback(self, msg):

        if msg.data:

            if not self.scan_started:

                self.scan_started = True

                self.get_logger().info(
                    'Scan started. Detection is now active.'
                )

    # =============================================================
    # IMAGE
    # =============================================================

    def image_callback(self, msg):

        # Do not run YOLO before Rudra reaches the
        # scan-start waypoint.
        if not self.scan_started:
            return

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as e:

            self.get_logger().error(
                f'Failed to convert image: {e}'
            )

            return

        # ---------------------------------------------------------
        # YOLO inference
        # ---------------------------------------------------------

        try:

            results = self.model(
                frame,
                conf=CONFIDENCE_THRESHOLD,
                verbose=False
            )

        except Exception as e:

            self.get_logger().error(
                f'YOLO inference failed: {e}'
            )

            return

        if not results:
            return

        result = results[0]

        if result.boxes is None:
            return

        image_height, image_width = frame.shape[:2]

        for box in result.boxes:

            confidence = float(
                box.conf[0]
            )

            class_id = int(
                box.cls[0]
            )

            class_name = self.model.names[
                class_id
            ]

            # We only care about people.
            if class_name.lower() != 'person':
                continue

            x1, y1, x2, y2 = (
                box.xyxy[0].tolist()
            )

            pixel_x = (
                x1 + x2
            ) / 2.0

            pixel_y = (
                y1 + y2
            ) / 2.0

            self.detection_counter += 1

            detection_id = (
                self.detection_counter
            )

            # -----------------------------------------------------
            # Save detection image
            # -----------------------------------------------------

            annotated = frame.copy()

            cv2.rectangle(
                annotated,
                (int(x1), int(y1)),
                (int(x2), int(y2)),
                (0, 255, 0),
                2
            )

            cv2.putText(
                annotated,
                f'person {confidence:.2f}',
                (int(x1), max(20, int(y1) - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            image_filename = (
                f'detection_{detection_id}.jpg'
            )

            image_path = os.path.join(
                self.save_dir,
                image_filename
            )

            cv2.imwrite(
                image_path,
                annotated
            )

            # -----------------------------------------------------
            # Publish RawDetection
            # -----------------------------------------------------

            detection = RawDetection()

            detection.header = msg.header

            detection.detection_id = (
                detection_id
            )

            detection.pixel_x = (
                float(pixel_x)
            )

            detection.pixel_y = (
                float(pixel_y)
            )

            detection.confidence = (
                confidence
            )

            detection.class_name = (
                class_name
            )

            detection.image_width = (
                image_width
            )

            detection.image_height = (
                image_height
            )

            self.detection_pub.publish(
                detection
            )

            self.get_logger().info(
                f'Person detected: '
                f'id={detection_id}, '
                f'confidence={confidence:.2f}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = DetectionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
