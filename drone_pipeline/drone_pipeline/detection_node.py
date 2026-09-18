import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
from drone_msgs.msg import RawDetection

import os
import cv2


# ============================================================
# PATHS
# ============================================================

MODEL_PATH = '/home/tanmayi_unix/nidar20_ws/models/best.pt'

CONFIDENCE_THRESHOLD = 0.5


class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')

        # ----------------------------------------------------
        # Drone ID
        # ----------------------------------------------------

        self.declare_parameter('drone_id', 'barbarik')

        self.drone_id = (
            self.get_parameter('drone_id')
            .get_parameter_value()
            .string_value
        )

        # ----------------------------------------------------
        # Detection state
        # ----------------------------------------------------

        self.scan_started = False
        self.detection_counter = 0

        # ----------------------------------------------------
        # Save directory
        # ----------------------------------------------------

        self.save_dir = (
            f'/home/tanmayi_unix/nidar_ros_ws/detections/'
            f'{self.drone_id}'
        )

        os.makedirs(self.save_dir, exist_ok=True)

        # ----------------------------------------------------
        # YOLO
        # ----------------------------------------------------

        self.bridge = CvBridge()

        self.get_logger().info(
            f'Loading YOLO model from {MODEL_PATH}'
        )

        self.model = YOLO(MODEL_PATH)

        # ----------------------------------------------------
        # Topics
        # ----------------------------------------------------

        image_topic = f'/{self.drone_id}/image_raw'
        scan_start_topic = f'/{self.drone_id}/scan_start'
        detections_topic = f'/{self.drone_id}/raw_detections'

        # Camera input
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # Scan-start signal
        self.scan_start_subscription = self.create_subscription(
            Bool,
            scan_start_topic,
            self.scan_start_callback,
            10
        )

        # Detection output
        self.publisher_ = self.create_publisher(
            RawDetection,
            detections_topic,
            10
        )

        self.get_logger().info(
            f'Detection node started for "{self.drone_id}"'
        )

        self.get_logger().info(
            f'Waiting for {scan_start_topic} before detecting...'
        )

    # ========================================================
    # SCAN START
    # ========================================================

    def scan_start_callback(self, msg):

        if msg.data and not self.scan_started:

            self.scan_started = True

            self.get_logger().info(
                f'[{self.drone_id}] SCAN STARTED — YOLO detection ON'
            )

    # ========================================================
    # IMAGE CALLBACK
    # ========================================================

    def image_callback(self, msg):

        # Do absolutely nothing before scan starts
        if not self.scan_started:
            return

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        height, width = frame.shape[:2]

        # ----------------------------------------------------
        # YOLO inference
        # ----------------------------------------------------

        results = self.model(
            frame,
            verbose=False
        )[0]

        # ----------------------------------------------------
        # Process detections
        # ----------------------------------------------------

        for box in results.boxes:

            confidence = float(box.conf[0])

            if confidence < CONFIDENCE_THRESHOLD:
                continue

            class_id = int(box.cls[0])
            class_name = self.model.names[class_id]

            # ------------------------------------------------
            # ONLY PERSONS
            # ------------------------------------------------

            if class_name.lower() != 'person':
                continue

            # ------------------------------------------------
            # Bounding box center
            # ------------------------------------------------

            x1, y1, x2, y2 = box.xyxy[0].tolist()

            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0

            # ------------------------------------------------
            # Detection ID
            # ------------------------------------------------

            det_id = self.detection_counter

            image_path = os.path.join(
                self.save_dir,
                f'detection_{det_id}.jpg'
            )

            cv2.imwrite(
                image_path,
                frame
            )

            # ------------------------------------------------
            # Create ROS detection message
            # ------------------------------------------------

            det_msg = RawDetection()

            det_msg.header = msg.header

            det_msg.detection_id = det_id
            det_msg.pixel_x = center_x
            det_msg.pixel_y = center_y
            det_msg.confidence = confidence
            det_msg.class_name = class_name

            det_msg.image_width = width
            det_msg.image_height = height

            self.publisher_.publish(det_msg)

            self.get_logger().info(
                f'[{self.drone_id}] PERSON detected '
                f'ID={det_id} '
                f'confidence={confidence:.2f} '
                f'pixel=({center_x:.1f}, {center_y:.1f})'
            )

            self.detection_counter += 1


def main(args=None):

    rclpy.init(args=args)

    node = DetectionNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
