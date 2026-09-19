import rclpy
from rclpy.node import Node

from mavros_msgs.msg import (
    State,
    WaypointReached,
    WaypointList
)

from mavros_msgs.srv import (
    CommandBool,
    SetMode
)

from std_msgs.msg import Bool


# Third waypoint = index 2
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

        self.connected = False
        self.armed = False

        self.mode_set = False

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
            State,
            f'{mavros_prefix}/state',
            self.state_callback,
            10
        )

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

        # ---------------------------------------------------------
        # Services
        # ---------------------------------------------------------

        self.arm_client = self.create_client(
            CommandBool,
            f'{mavros_prefix}/cmd/arming'
        )

        self.mode_client = self.create_client(
            SetMode,
            f'{mavros_prefix}/set_mode'
        )

        # ---------------------------------------------------------
        # Startup timer
        # ---------------------------------------------------------

        self.timer = self.create_timer(
            2.0,
            self.startup_sequence
        )

        self.get_logger().info(
            f'Mission control started for '
            f'"{self.drone_id}"'
        )

    # =============================================================
    # STATE
    # =============================================================

    def state_callback(self, msg):

        self.connected = msg.connected
        self.armed = msg.armed

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

    # =============================================================
    # STARTUP
    # =============================================================

    def startup_sequence(self):

        if not self.connected:

            self.get_logger().info(
                f'[{self.drone_id}] '
                f'Waiting for FCU connection...'
            )

            return

        # ---------------------------------------------------------
        # ARM
        # ---------------------------------------------------------

        if not self.armed:

            self.arm_drone()

            return

        # ---------------------------------------------------------
        # AUTO
        # ---------------------------------------------------------

        if not self.mode_set:

            self.set_auto_mode()

            return

    # =============================================================
    # ARM
    # =============================================================

    def arm_drone(self):

        if not self.arm_client.wait_for_service(
            timeout_sec=1.0
        ):
            return

        request = (
            CommandBool.Request()
        )

        request.value = True

        future = (
            self.arm_client
            .call_async(request)
        )

        future.add_done_callback(
            self.arm_response
        )

    def arm_response(self, future):

        try:

            result = future.result()

            if (
                result
                and result.success
            ):

                self.get_logger().info(
                    f'[{self.drone_id}] '
                    f'Arming command accepted'
                )

            else:

                self.get_logger().warn(
                    f'[{self.drone_id}] '
                    f'Arming command failed'
                )

        except Exception as e:

            self.get_logger().error(
                f'Arming error: {e}'
            )

    # =============================================================
    # AUTO MODE
    # =============================================================

    def set_auto_mode(self):

        if not self.mode_client.wait_for_service(
            timeout_sec=1.0
        ):
            return

        request = (
            SetMode.Request()
        )

        request.custom_mode = 'AUTO'

        future = (
            self.mode_client
            .call_async(request)
        )

        future.add_done_callback(
            self.mode_response
        )

    def mode_response(self, future):

        try:

            result = future.result()

            if (
                result
                and result.mode_sent
            ):

                self.mode_set = True

                self.get_logger().info(
                    f'[{self.drone_id}] '
                    f'AUTO mode command sent'
                )

            else:

                self.get_logger().warn(
                    f'[{self.drone_id}] '
                    f'Failed to set AUTO'
                )

        except Exception as e:

            self.get_logger().error(
                f'AUTO mode error: {e}'
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
