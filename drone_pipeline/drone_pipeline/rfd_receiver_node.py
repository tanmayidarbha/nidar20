import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

from drone_msgs.msg import Target, TargetList

from pymavlink import mavutil

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "custom_dialect"

from . import custom_dialect

mavutil.mavlink = custom_dialect


class RFDReceiverNode(Node):

    def __init__(self):
        super().__init__('rfd_receiver_node')

        self.declare_parameter(
            'serial_port',
            '/dev/serial0'
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

        # Rudra = SYSID 2
        self.expected_source_system = 2

        self.connection = None

        # Prevent duplicate target delivery.
        self.received_target_ids = set()

        # ---------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------

        self.target_pub = self.create_publisher(
            TargetList,
            '/barbarik/target_list',
            10
        )

        self.scan_complete_pub = self.create_publisher(
            Bool,
            '/barbarik/scan_complete',
            10
        )

        self.connect_mavlink()

        # Poll MAVLink messages.
        self.timer = self.create_timer(
            0.01,
            self.receive_messages
        )

    # =============================================================
    # MAVLINK CONNECTION
    # =============================================================

    def connect_mavlink(self):

        try:

            self.connection = mavutil.mavlink_connection(
                self.serial_port,
                baud=self.baudrate,
                source_system=1
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
    # RECEIVE
    # =============================================================

    def receive_messages(self):

        if self.connection is None:
            return

        try:

            msg = self.connection.recv_match(
                blocking=False
            )

            if msg is None:
                return

            source_system = msg.get_srcSystem()
            message_type = msg.get_type()

            # Only accept messages from Rudra SYSID 2.
            if source_system != self.expected_source_system:
                return

            # -----------------------------------------------------
            # TARGET DETECTION
            # -----------------------------------------------------

            if message_type == 'TARGET_DETECTION_DATA':

                target_id = int(msg.target_id)

                if target_id in self.received_target_ids:
                    return

                self.received_target_ids.add(target_id)

                latitude = msg.lat / 1e7
                longitude = msg.lon / 1e7

                confidence = float(msg.confidence) / 100.0

                target = Target()

                target.target_id = target_id
                target.latitude = latitude
                target.longitude = longitude
                target.confidence = confidence
                target.source_drone_id = 'rudra'

                target_list = TargetList()
                target_list.targets.append(target)

                self.target_pub.publish(target_list)

                self.get_logger().info(
                    f'Received target {target_id} '
                    f'from Rudra: '
                    f'{latitude:.7f}, '
                    f'{longitude:.7f}'
                )

                # Send acknowledgement back through MAVLink.
                self.send_target_received_ack(
                    target_id,
                    msg.lat,
                    msg.lon
                )

            # -----------------------------------------------------
            # SWARM STATUS
            # -----------------------------------------------------

            elif message_type == 'SWARM_STATUS_ACK':

                event_type = int(msg.event_type)

                # 3 = SCAN_COMPLETE
                if event_type == 3:

                    complete_msg = Bool()
                    complete_msg.data = True

                    self.scan_complete_pub.publish(
                        complete_msg
                    )

                    self.get_logger().info(
                        'Received SCAN_COMPLETE from Rudra.'
                    )

        except Exception as e:

            self.get_logger().error(
                f'MAVLink receive error: {e}'
            )

    # =============================================================
    # TARGET RECEIVED ACK
    # =============================================================

    def send_target_received_ack(
        self,
        target_id,
        latitude,
        longitude
    ):

        if self.connection is None:
            return

        try:

            self.connection.mav.swarm_status_ack_send(
                target_id=int(target_id),
                event_type=1,
                lat=int(latitude),
                lon=int(longitude)
            )

            self.get_logger().info(
                f'Sent TARGET_RECEIVED ACK '
                f'for target {target_id}'
            )

        except Exception as e:

            self.get_logger().error(
                f'Failed to send TARGET_RECEIVED ACK: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = RFDReceiverNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
