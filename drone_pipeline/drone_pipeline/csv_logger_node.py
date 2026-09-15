import rclpy
from rclpy.node import Node
from drone_msgs.msg import Detection, TargetList
from std_msgs.msg import Int32
import csv
import os
from datetime import datetime

LOG_DIR = '/home/tanmayi_unix/nidar_ros_ws/logs'

class CsvLoggerNode(Node):
    def __init__(self):
        super().__init__('csv_logger_node')

        os.makedirs(LOG_DIR, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.detections_path = os.path.join(LOG_DIR, f'detections_{timestamp}.csv')
        self.drops_path = os.path.join(LOG_DIR, f'drops_{timestamp}.csv')

        with open(self.detections_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'ros_time', 'drone_id', 'detection_id', 'latitude', 'longitude',
                'altitude', 'confidence', 'class_name'
            ])

        with open(self.drops_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['ros_time', 'target_id', 'latitude', 'longitude', 'confidence', 'source_drone_id'])

        self.create_subscription(Detection, '/flamingo/detections', self.detection_callback, 10)
        self.create_subscription(Detection, '/rudra/detections', self.detection_callback, 10)
        self.create_subscription(TargetList, '/rudra/target_list', self.target_list_callback, 10)
        self.create_subscription(Int32, '/rudra/trigger_drop', self.drop_callback, 10)

        self.latest_targets = {}

        self.get_logger().info(f'CSV logger started. Detections -> {self.detections_path}, Drops -> {self.drops_path}')

    def detection_callback(self, msg):
        row = [
            self.get_clock().now().to_msg().sec,
            msg.drone_id,
            msg.detection_id,
            msg.latitude,
            msg.longitude,
            msg.altitude,
            msg.confidence,
            msg.class_name,
        ]
        with open(self.detections_path, 'a', newline='') as f:
            csv.writer(f).writerow(row)
        self.get_logger().info(f'Logged detection #{msg.detection_id} from {msg.drone_id}')

    def target_list_callback(self, msg):
        self.latest_targets = {t.target_id: t for t in msg.targets}
        self.get_logger().info(f'Cached {len(self.latest_targets)} targets for drop logging')

    def drop_callback(self, msg):
        target_id = msg.data
        target = self.latest_targets.get(target_id)
        row = [
            self.get_clock().now().to_msg().sec,
            target_id,
            target.latitude if target else '',
            target.longitude if target else '',
            target.confidence if target else '',
            target.source_drone_id if target else '',
        ]
        with open(self.drops_path, 'a', newline='') as f:
            csv.writer(f).writerow(row)
        self.get_logger().info(f'Logged payload drop for target #{target_id}')

def main(args=None):
    rclpy.init(args=args)
    node = CsvLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
