import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, WaypointReached, WaypointList
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Bool


# 3rd waypoint = index 2 because waypoint numbering starts at 0
SCAN_START_WAYPOINT = 2


class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')

        self.declare_parameter('drone_id', 'flamingo')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value

        self.connected = False
        self.armed = False
        self.mode_set = False
        self.total_waypoints = None

        self.scan_started = False
        self.scan_complete_sent = False

        mavros_prefix = f'/{self.drone_id}'

        scan_start_topic = f'/{self.drone_id}/scan_start'
        scan_complete_topic = f'/{self.drone_id}/scan_complete'

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

        # NEW: tells detection node when scanning should begin
        self.scan_start_pub = self.create_publisher(
            Bool,
            scan_start_topic,
            10
        )

        self.scan_complete_pub = self.create_publisher(
            Bool,
            scan_complete_topic,
            10
        )

        self.arm_client = self.create_client(
            CommandBool,
            f'{mavros_prefix}/cmd/arming'
        )

        self.mode_client = self.create_client(
            SetMode,
            f'{mavros_prefix}/set_mode'
        )

        self.timer = self.create_timer(
            2.0,
            self.startup_sequence
        )

        self.get_logger().info(
            f'Mission control node started for "{self.drone_id}", '
            f'using {mavros_prefix}'
        )

    def state_callback(self, msg):
        self.connected = msg.connected
        self.armed = msg.armed

    def waypoints_callback(self, msg):
        self.total_waypoints = len(msg.waypoints)

        self.get_logger().info(
            f'[{self.drone_id}] Mission loaded: '
            f'{self.total_waypoints} waypoints'
        )

    def reached_callback(self, msg):
        wp = msg.wp_seq

        self.get_logger().info(
            f'[{self.drone_id}] Reached waypoint #{wp}'
        )

        # ============================================================
        # START SCANNING AT 3RD WAYPOINT
        # ============================================================
        if wp == SCAN_START_WAYPOINT and not self.scan_started:

            self.scan_start_pub.publish(
                Bool(data=True)
            )

            self.scan_started = True

            self.get_logger().info(
                f'[{self.drone_id}] Reached scan-start waypoint '
                f'#{SCAN_START_WAYPOINT} — published scan_start'
            )

        # ============================================================
        # FINISH SCANNING AT FINAL WAYPOINT
        # ============================================================
        if (
            self.total_waypoints is not None
            and wp == self.total_waypoints - 1
        ):
            if not self.scan_complete_sent:

                self.scan_complete_pub.publish(
                    Bool(data=True)
                )

                self.scan_complete_sent = True

                self.get_logger().info(
                    f'[{self.drone_id}] Final waypoint reached — '
                    f'published scan_complete'
                )

    def startup_sequence(self):

        if not self.connected:
            self.get_logger().info(
                f'[{self.drone_id}] Waiting for FCU connection...'
            )
            return

        if not self.armed:
            self.arm_drone()
            return

        if not self.mode_set:
            self.set_auto_mode()
            return

    def arm_drone(self):

        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            return

        req = CommandBool.Request()
        req.value = True

        future = self.arm_client.call_async(req)
        future.add_done_callback(self.arm_response)

    def arm_response(self, future):

        result = future.result()

        if result and result.success:
            self.get_logger().info(
                f'[{self.drone_id}] Arming command accepted'
            )
        else:
            self.get_logger().warn(
                f'[{self.drone_id}] Arming command failed, will retry'
            )

    def set_auto_mode(self):

        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            return

        req = SetMode.Request()
        req.custom_mode = 'AUTO'

        future = self.mode_client.call_async(req)
        future.add_done_callback(self.mode_response)

    def mode_response(self, future):

        result = future.result()

        if result and result.mode_sent:

            self.mode_set = True

            self.get_logger().info(
                f'[{self.drone_id}] AUTO mode set — '
                f'mission should now begin'
            )

        else:
            self.get_logger().warn(
                f'[{self.drone_id}] Set mode failed, will retry'
            )


def main(args=None):
    rclpy.init(args=args)

    node = MissionControlNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
