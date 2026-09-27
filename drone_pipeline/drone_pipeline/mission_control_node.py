import rclpy
from rclpy.node import Node

from mavros_msgs.msg import (
    WaypointReached,
    WaypointList
)

from std_msgs.msg import Bool


# First waypoint = index 0
SCAN_START_WAYPOINT = 2


class MissionControlNode(Node):

    def __init__(self):

        super().__init__(
            'mission_control_node'
        )

        # Rudra = scout
        self.declare_parameter(
            'drone_id',
            'rudra'
        )

        self.drone_id = (
            self.get_parameter('drone_id')
            .get_parameter_value()
            .string_value
        )

        self.total_waypoints = None

        self.scan_started = False
        self.scan_complete_sent = False

        mavros_prefix = (
            f'/{self.drone_id}'
        )

        # ---------------------------------------------------------
        # Topics
        # ---------------------------------------------------------

        self.scan_start_pub = self.create_publisher(
            Bool,
            f'/{self.drone_id}/scan_start',
            10
        )

        self.scan_complete_pub = self.create_publisher(
            Bool,
            f'/{self.drone_id}/scan_complete',
            10
        )

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------

        self.create_subscription(
            WaypointList,
            f'{mavros_prefix}/mission/waypoints',
            self.waypoints_callback,
            10
        )

        self.create_subscription(
            WaypointReached,
            f'{mavros_prefix}/mission/reached',
            self.reached_callback,
            10
        )

        self.get_logger().info(
            f'Mission control observer started for '
            f'"{self.drone_id}"'
        )

        self.get_logger().info(
            'Flight control is handled by Mission Planner / FC.'
        )

        self.get_logger().info(
            'Scanning will start at waypoint #0.'
        )

    # =============================================================
    # WAYPOINT LIST
    # =============================================================

    def waypoints_callback(self, msg):

        self.total_waypoints = (
            len(msg.waypoints)
        )

        self.get_logger().info(
            f'[{self.drone_id}] '
            f'Mission loaded: '
            f'{self.total_waypoints} waypoints'
        )

    # =============================================================
    # WAYPOINT REACHED
    # =============================================================

    def reached_callback(self, msg):

        wp = msg.wp_seq

        self.get_logger().info(
            f'[{self.drone_id}] '
            f'Reached waypoint #{wp}'
        )

        # ---------------------------------------------------------
        # START SCANNING
        # ---------------------------------------------------------

        if (
            wp == SCAN_START_WAYPOINT
            and not self.scan_started
        ):

            self.scan_start_pub.publish(
                Bool(data=True)
            )

            self.scan_started = True

            self.get_logger().info(
                f'[{self.drone_id}] '
                f'Scan started at waypoint #{wp}'
            )

        # ---------------------------------------------------------
        # SCAN COMPLETE
        # ---------------------------------------------------------

        if (
            self.total_waypoints is not None
            and wp == self.total_waypoints - 1
            and not self.scan_complete_sent
        ):

            self.scan_complete_pub.publish(
                Bool(data=True)
            )

            self.scan_complete_sent = True

            self.get_logger().info(
                f'[{self.drone_id}] '
                f'Final waypoint reached. '
                f'Scan complete sent.'
            )


def main(args=None):

    rclpy.init(args=args)

    node = MissionControlNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
