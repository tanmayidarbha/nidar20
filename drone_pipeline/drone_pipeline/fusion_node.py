import math

import rclpy
from rclpy.node import Node

from drone_msgs.msg import Detection, Target, TargetList


DEDUP_DISTANCE_METERS = 10.0
FIRST_TARGET_ID = 101


class FusionNode(Node):

    def __init__(self):
        super().__init__('fusion_node')

        self.next_target_id = FIRST_TARGET_ID
        self.targets = []

        self.detection_sub = self.create_subscription(
            Detection,
            '/barbarik/detections',
            self.detection_callback,
            10
        )

        self.target_pub = self.create_publisher(
            TargetList,
            '/rudra/target_list',
            10
        )

        self.get_logger().info(
            'Fusion node started: Barbarik detections -> Rudra targets'
        )

    def detection_callback(self, msg):

        # Check whether this detection is close to an existing target
        for target in self.targets:

            distance = self.haversine_distance(
                msg.latitude,
                msg.longitude,
                target.latitude,
                target.longitude
            )

            if distance <= DEDUP_DISTANCE_METERS:

                # Keep the higher-confidence detection
                if msg.confidence > target.confidence:
                    target.latitude = msg.latitude
                    target.longitude = msg.longitude
                    target.confidence = msg.confidence
                    target.source_drone_id = msg.drone_id

                    self.publish_targets()

                    self.get_logger().info(
                        f'Updated target {target.target_id} '
                        f'with higher confidence {msg.confidence:.2f}'
                    )

                return

        # New target
        target = Target()

        target.target_id = self.next_target_id
        target.latitude = msg.latitude
        target.longitude = msg.longitude
        target.confidence = msg.confidence
        target.source_drone_id = msg.drone_id

        self.targets.append(target)
        self.next_target_id += 1

        self.publish_targets()

        self.get_logger().info(
            f'NEW TARGET {target.target_id}: '
            f'lat={target.latitude:.7f}, '
            f'lon={target.longitude:.7f}, '
            f'confidence={target.confidence:.2f}'
        )

    def publish_targets(self):

        msg = TargetList()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.targets = self.targets

        self.target_pub.publish(msg)

    @staticmethod
    def haversine_distance(lat1, lon1, lat2, lon2):

        R = 6371000.0

        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)

        dlat = lat2 - lat1
        dlon = math.radians(lon2 - lon1)

        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(lat1)
            * math.cos(lat2)
            * math.sin(dlon / 2) ** 2
        )

        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c


def main(args=None):
    rclpy.init(args=args)

    node = FusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
