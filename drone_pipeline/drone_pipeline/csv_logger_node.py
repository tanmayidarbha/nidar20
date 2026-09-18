import csv
import os

import rclpy
from rclpy.node import Node

from drone_msgs.msg import TargetList


CSV_PATH = os.path.expanduser(
    '~/nidar20_ws/logs/targets.csv'
)


class CSVLoggerNode(Node):

    def __init__(self):
        super().__init__('csv_logger_node')

        os.makedirs(
            os.path.dirname(CSV_PATH),
            exist_ok=True
        )

        self.logged_target_ids = set()

        self.target_sub = self.create_subscription(
            TargetList,
            '/rudra/target_list',
            self.target_callback,
            10
        )

        self.create_csv_if_needed()

        self.get_logger().info(
            f'CSV logger started: {CSV_PATH}'
        )

    def create_csv_if_needed(self):

        if not os.path.exists(CSV_PATH):

            with open(
                CSV_PATH,
                'w',
                newline=''
            ) as file:

                writer = csv.writer(file)

                writer.writerow([
                    'target_id',
                    'latitude',
                    'longitude',
                    'confidence',
                    'source_drone'
                ])

    def target_callback(self, msg):

        for target in msg.targets:

            if target.target_id in self.logged_target_ids:
                continue

            with open(
                CSV_PATH,
                'a',
                newline=''
            ) as file:

                writer = csv.writer(file)

                writer.writerow([
                    target.target_id,
                    f'{target.latitude:.7f}',
                    f'{target.longitude:.7f}',
                    f'{target.confidence:.3f}',
                    target.source_drone_id
                ])

            self.logged_target_ids.add(
                target.target_id
            )

            self.get_logger().info(
                f'Logged target {target.target_id} '
                f'to targets.csv'
            )


def main(args=None):

    rclpy.init(args=args)

    node = CSVLoggerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
