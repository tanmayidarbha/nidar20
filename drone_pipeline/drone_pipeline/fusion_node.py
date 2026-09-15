import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from drone_msgs.msg import Detection, TargetList, Target
import math

DEDUP_DISTANCE_METERS = 10.0
FIRST_TARGET_ID = 101


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        self.scout_done = False
        self.payload_done = False
        self.all_detections = []
        self.fused_published = False

        self.create_subscription(
            Bool,
            '/flamingo/scan_complete',
            self.scout_done_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/rudra/scan_complete',
            self.payload_done_callback,
            10
        )

        self.create_subscription(
            Detection,
            '/flamingo/detections',
            self.detection_callback,
            10
        )

        self.create_subscription(
            Detection,
            '/rudra/detections',
            self.detection_callback,
            10
        )

        self.targets_pub = self.create_publisher(
            TargetList,
            '/rudra/target_list',
            10
        )

        self.get_logger().info(
            'Fusion node started, collecting detections from both drones'
        )

    def scout_done_callback(self, msg):
        if msg.data:
            self.scout_done = True
            self.get_logger().info(
                'Flamingo scan complete signal received'
            )
            self.try_fuse()

    def payload_done_callback(self, msg):
        if msg.data:
            self.payload_done = True
            self.get_logger().info(
                'Rudra scan complete signal received'
            )
            self.try_fuse()

    def detection_callback(self, msg):
        self.all_detections.append(msg)

        self.get_logger().info(
            f'Received detection #{msg.detection_id} '
            f'from {msg.drone_id}'
        )

    def try_fuse(self):
        if self.fused_published:
            return

        if not (self.scout_done and self.payload_done):
            return

        self.get_logger().info(
            f'Both scans complete. '
            f'Fusing {len(self.all_detections)} detections'
        )

        targets = self.dedup_detections(self.all_detections)

        target_list_msg = TargetList()
        target_list_msg.targets = targets

        self.targets_pub.publish(target_list_msg)

        self.fused_published = True

        self.get_logger().info(
            f'Published {len(targets)} fused targets '
            f'on /rudra/target_list'
        )

        for target in targets:
            self.get_logger().info(
                f'Target {target.target_id}: '
                f'{target.latitude:.6f}, '
                f'{target.longitude:.6f} '
                f'confidence={target.confidence:.2f} '
                f'source={target.source_drone_id}'
            )

    def dedup_detections(self, detections):
        clusters = []

        for det in detections:
            placed = False

            for cluster in clusters:
                representative = max(
                    cluster,
                    key=lambda d: d.confidence
                )

                distance = self.haversine_m(
                    representative.latitude,
                    representative.longitude,
                    det.latitude,
                    det.longitude
                )

                if distance <= DEDUP_DISTANCE_METERS:
                    cluster.append(det)
                    placed = True
                    break

            if not placed:
                clusters.append([det])

        targets = []

        for index, cluster in enumerate(clusters):
            best = max(
                cluster,
                key=lambda d: d.confidence
            )

            target = Target()

            # Persistent NIDAR survivor ID
            target.target_id = FIRST_TARGET_ID + index

            target.latitude = best.latitude
            target.longitude = best.longitude
            target.confidence = best.confidence
            target.source_drone_id = best.drone_id

            targets.append(target)

        return targets

    def haversine_m(self, lat1, lon1, lat2, lon2):
        R = 6371000.0

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)

        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = (
            math.sin(dphi / 2) ** 2
            + math.cos(phi1)
            * math.cos(phi2)
            * math.sin(dlambda / 2) ** 2
        )

        return 2 * R * math.asin(math.sqrt(a))


def main(args=None):
    rclpy.init(args=args)

    node = FusionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
