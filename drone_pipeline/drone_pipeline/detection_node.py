import os
import cv2
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from rfdetr import RFDETRMedium

from drone_msgs.msg import RawDetection


# ============================================================
# CONFIGURATION
# ============================================================

MODEL_PATH = '/home/tanmayi_unix/nidar20_ws/models/best.pth'

CONFIDENCE_THRESHOLD = 0.5

SAVE_DIR = '/home/tanmayi_unix/nidar20_ws/detections'

# RF-DETR model is trained for survivor/person detection.
# We assume person is class 0 in the trained model.
PERSON_CLASS_ID = 0


# ============================================================
# NODE
# ============================================================

class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')

        # ----------------------------------------------------
        # Parameters
        # ----------------------------------------------------

        self.declare_parameter('drone_id', 'rudra')

        self.drone_id = (
            self.get_parameter('drone_id')
            .get_parameter_value()
            .string_value
        )

        # ----------------------------------------------------
        # Paths
        # ----------------------------------------------------

        self.save_dir = os.path.join(
            SAVE_DIR,
            self.drone_id
        )

        os.makedirs(self.save_dir, exist_ok=True)

        # ----------------------------------------------------
        # RF-DETR
        # ----------------------------------------------------

        self.get_logger().info(
            f'Loading RF-DETR Medium model from: {MODEL_PATH}'
        )

        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(
                f'RF-DETR model not found: {MODEL_PATH}'
            )
            raise FileNotFoundError(MODEL_PATH)

        self.model = RFDETRMedium(
            pretrain_weights=MODEL_PATH
        )

        self.get_logger().info(
            'RF-DETR Medium model loaded successfully.'
        )

        # ----------------------------------------------------
        # ROS
        # ----------------------------------------------------

        self.bridge = CvBridge()

        self.scan_started = False
        self.detection_counter = 0

        image_topic = f'/{self.drone_id}/image_raw'
        scan_start_topic = f'/{self.drone_id}/scan_start'
        detection_topic = f'/{self.drone_id}/raw_detections'

        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.scan_subscription = self.create_subscription(
            __import__('std_msgs.msg', fromlist=['Bool']).Bool,
            scan_start_topic,
            self.scan_start_callback,
            10
        )

        self.detection_publisher = self.create_publisher(
            RawDetection,
            detection_topic,
            10
        )

        self.get_logger().info(
            f'Detection node started for "{self.drone_id}"'
        )

        self.get_logger().info(
            f'Waiting for scan start on {scan_start_topic}'
        )

    # ========================================================
    # SCAN START
    # ========================================================

    def scan_start_callback(self, msg):

        if msg.data and not self.scan_started:

            self.scan_started = True

            self.get_logger().info(
                f'[{self.drone_id}] Scan started. '
                f'RF-DETR detection is now active.'
            )

    # ========================================================
    # IMAGE CALLBACK
    # ========================================================

    def image_callback(self, msg):

        # Do not run detection before mission scan starts.
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

        image_height, image_width = frame.shape[:2]

        # ----------------------------------------------------
        # RF-DETR expects RGB images.
        # OpenCV gives us BGR.
        # ----------------------------------------------------

        rgb_frame = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB
        )

        try:

            detections = self.model.predict(
                rgb_frame,
                threshold=CONFIDENCE_THRESHOLD
            )

        except Exception as e:

            self.get_logger().error(
                f'RF-DETR inference failed: {e}'
            )

            return

        # ----------------------------------------------------
        # Process detections
        # ----------------------------------------------------

        boxes = detections.xyxy
        confidences = detections.confidence
        class_ids = detections.class_id

        detection_count = 0

        for box, confidence, class_id in zip(
            boxes,
            confidences,
            class_ids
        ):

            confidence = float(confidence)
            class_id = int(class_id)

            # We only want persons/survivors.
            if class_id != PERSON_CLASS_ID:
                continue

            if confidence < CONFIDENCE_THRESHOLD:
                continue

            # ------------------------------------------------
            # Bounding box
            # ------------------------------------------------

            x1, y1, x2, y2 = box

            x1 = int(x1)
            y1 = int(y1)
            x2 = int(x2)
            y2 = int(y2)

            # ------------------------------------------------
            # Bounding-box center
            # ------------------------------------------------

            pixel_x = (x1 + x2) / 2.0
            pixel_y = (y1 + y2) / 2.0

            # ------------------------------------------------
            # Detection ID
            # ------------------------------------------------

            self.detection_counter += 1

            detection_id = self.detection_counter

            # ------------------------------------------------
            # Create ROS RawDetection message
            # ------------------------------------------------

            detection_msg = RawDetection()

            detection_msg.header = msg.header

            detection_msg.detection_id = detection_id

            detection_msg.pixel_x = pixel_x
            detection_msg.pixel_y = pixel_y

            detection_msg.confidence = confidence

            detection_msg.class_name = 'person'

            detection_msg.image_width = image_width
            detection_msg.image_height = image_height

            self.detection_publisher.publish(
                detection_msg
            )

            detection_count += 1

            # ------------------------------------------------
            # Draw detection
            # ------------------------------------------------

            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2
            )

            label = (
                f'person {confidence:.2f} '
                f'ID:{detection_id}'
            )

            cv2.putText(
                frame,
                label,
                (x1, max(y1 - 10, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            self.get_logger().info(
                f'[{self.drone_id}] '
                f'Person detected: '
                f'ID={detection_id}, '
                f'confidence={confidence:.2f}, '
                f'center=({pixel_x:.1f}, {pixel_y:.1f})'
            )

        # ----------------------------------------------------
        # Save frame only when a person was detected
        # ----------------------------------------------------

        if detection_count > 0:

            filename = os.path.join(
                self.save_dir,
                f'detection_{self.detection_counter}.jpg'
            )

            cv2.imwrite(
                filename,
                frame
            )


# ============================================================
# MAIN
# ============================================================

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
