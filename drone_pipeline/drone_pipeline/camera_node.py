import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

DEFAULT_RTSP_URL = 'rtsp://192.168.144.25:8554/main.264'

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('drone_id', 'barbarik')
        self.declare_parameter('rtsp_url', DEFAULT_RTSP_URL)
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value
        rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value

        image_topic = f'/{self.drone_id}/image_raw'
        self.publisher_ = self.create_publisher(Image, image_topic, 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open RTSP stream at {rtsp_url}')

        timer_period = 1.0 / 15.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f'Camera node started for "{self.drone_id}", publishing on {image_topic}')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('Failed to read frame from RTSP stream')

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
