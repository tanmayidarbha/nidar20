import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from drone_msgs.msg import TargetList

from pymavlink import mavutil

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "custom_dialect"

from . import custom_dialect

mavutil.mavlink = custom_dialect


class RFDSenderNode(Node):

    def __init__(self):
        super().__init__('rfd_sender_node')

        self.declare_parameter(
            'serial_port',
            '/dev/REPLACE_ME'
        )

        self.declare_parameter(
            'baudrate',
            115200
        )

        self.serial_port = (
            self.get_parameter('serial_port')
            .get_parameter_value()
            .string_value
        )

        self.baudrate = (
            self.get_parameter('baudrate')
            .get_parameter_value()
            .integer_value
        )

        self.source_system = 2
        self.connection = None

        self.target_sub = self.create_subscription(
            TargetList,
            '/rudra/target_list',
            self.target_callback,
            10
        )

        self.scan_complete_sub = self.create_subscription(
            Bool,
            '/rudra/scan_complete',
            self.scan_complete_callback,
            10
        )

        self.connect_mavlink()

    # =============================================================
    # MAVLINK CONNECTION
    # =============================================================

    def connect_mavlink(self):

        try:

            self.connection = mavutil.mavlink_connection(
                self.serial_port,
                baud=self.baudrate,
                source_system=self.source_system
            )

            self.get_logger().info(
                f'RFD MAVLink link opened on '
                f'{self.serial_port} @ {self.baudrate}'
            )

        except Exception as e:

            self.connection = None

            self.get_logger().error(
                f'Failed to open MAVLink link: {e}'
            )

    # =============================================================
    # TARGET DETECTION
    # =============================================================

    def target_callback(self, msg):

        if self.connection is None:
            self.get_logger().warn(
                'RFD MAVLink connection unavailable.'
            )
            return

        for target in msg.targets:

            try:

                self.connection.mav.target_detection_data_send(
                    target_id=int(target.target_id),
                    confidence=int(
                        max(
                            0,
                            min(
                                100,
                                target.confidence * 100
                            )
                        )
                    ),

                    # IMPORTANT:
                    # custom_dialect.xml uses lat/lon,
                    # not latitude/longitude.
                    lat=int(target.latitude * 1e7),
                    lon=int(target.longitude * 1e7)
                )

                self.get_logger().info(
                    f'Sent target {target.target_id} '
                    f'to Barbarik: '
                    f'{target.latitude:.7f}, '
                    f'{target.longitude:.7f}'
                )

            except Exception as e:

                self.get_logger().error(
                    f'Failed to send target '
                    f'{target.target_id}: {e}'
                )

    # =============================================================
    # SCAN COMPLETE
    # =============================================================

    def scan_complete_callback(self, msg):

        if not msg.data:
            return

        if self.connection is None:
            self.get_logger().warn(
                'Cannot send SCAN_COMPLETE: '
                'RFD MAVLink connection unavailable.'
            )
            return

        try:

            self.connection.mav.swarm_status_ack_send(
                target_id=0,
                event_type=3,
                lat=0,
                lon=0
            )

            self.get_logger().info(
                'Sent SCAN_COMPLETE to Barbarik.'
            )

        except Exception as e:

            self.get_logger().error(
                f'Failed to send SCAN_COMPLETE: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = RFDSenderNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
