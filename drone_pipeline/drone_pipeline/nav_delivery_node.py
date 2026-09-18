import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt8
from mavros_msgs.msg import GlobalPositionTarget
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

from drone_msgs.msg import TargetList


# Rudra flight altitude = 20 ft
RUDRA_ALTITUDE = 6.096

# Consider target reached within 3 metres
ARRIVAL_THRESHOLD_METERS = 3.0

# Position only
TYPE_MASK_POSITION_ONLY = 4088


STATE_WAITING = 'WAITING'
STATE_NAVIGATING = 'NAVIGATING'
STATE_DROPPING = 'DROPPING'
STATE_HOVERING = 'HOVERING'


class NavDeliveryNode(Node):

    def __init__(self):
        super().__init__('nav_delivery_node')

        self.declare_parameter('drone_id', 'rudra')
        self.drone_id = (
            self.get_parameter('drone_id')
            .get_parameter_value()
            .string_value
        )

        mavros_prefix = f'/{self.drone_id}/mavros'

        # --------------------------------------------------
        # STATE
        # --------------------------------------------------

        self.state = STATE_WAITING

        self.current_target = None
        self.remaining_targets = []

        self.current_lat = None
        self.current_lon = None
        self.current_alt = None

        self.vehicle_state = None

        self.arm_requested = False
        self.guided_requested = False
        self.takeoff_started = False

        # --------------------------------------------------
        # SUBSCRIBERS
        # --------------------------------------------------

        self.create_subscription(
            TargetList,
            f'/{self.drone_id}/target_list',
            self.target_list_callback,
            10
        )

        self.create_subscription(
            NavSatFix,
            f'{mavros_prefix}/global_position/global',
            self.gps_callback,
            10
        )

        self.create_subscription(
            State,
            f'{mavros_prefix}/state',
            self.state_callback,
            10
        )

        # --------------------------------------------------
        # PUBLISHERS
        # --------------------------------------------------

        self.setpoint_pub = self.create_publisher(
            GlobalPositionTarget,
            f'{mavros_prefix}/setpoint_position/global',
            10
        )

        self.drop_pub = self.create_publisher(
            UInt8,
            f'/{self.drone_id}/trigger_drop',
            10
        )

        # --------------------------------------------------
        # SERVICES
        # --------------------------------------------------

        self.mode_client = self.create_client(
            SetMode,
            f'{mavros_prefix}/set_mode'
        )

        self.arm_client = self.create_client(
            CommandBool,
            f'{mavros_prefix}/cmd/arming'
        )

        # --------------------------------------------------
        # CONTROL LOOP
        # --------------------------------------------------

        self.timer = self.create_timer(
            0.5,
            self.control_loop
        )

        self.get_logger().info(
            'Rudra navigation started - waiting for target'
        )

    # ======================================================
    # CALLBACKS
    # ======================================================

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude

    def state_callback(self, msg):
        self.vehicle_state = msg

    def target_list_callback(self, msg):

        if not msg.targets:
            return

        existing_ids = set()

        if self.current_target is not None:
            existing_ids.add(self.current_target.target_id)

        for target in self.remaining_targets:
            existing_ids.add(target.target_id)

        new_targets = []

        for target in msg.targets:

            if target.target_id not in existing_ids:
                self.remaining_targets.append(target)
                existing_ids.add(target.target_id)
                new_targets.append(target)

        if new_targets:
            self.get_logger().info(
                f'Received {len(new_targets)} new target(s)'
            )

            for target in new_targets:
                self.get_logger().info(
                    f'Queued target {target.target_id}: '
                    f'lat={target.latitude:.7f}, '
                    f'lon={target.longitude:.7f}'
                )

        # If Rudra is waiting on the ground, begin flight
        if self.state == STATE_WAITING and self.remaining_targets:
            self.get_logger().info(
                'Target received - starting Rudra flight'
            )

    # ======================================================
    # MAIN CONTROL LOOP
    # ======================================================

    def control_loop(self):

        # Need GPS before doing anything
        if self.current_lat is None or self.current_lon is None:
            return

        # ----------------------------------------------
        # WAITING
        # ----------------------------------------------

        if self.state == STATE_WAITING:

            if not self.remaining_targets:
                return

            # Start flight only after target received
            self.start_flight()

            return

        # ----------------------------------------------
        # NAVIGATING
        # ----------------------------------------------

        if self.state == STATE_NAVIGATING:

            if self.current_target is None:

                if self.remaining_targets:
                    self.pick_next_target()
                else:
                    self.state = STATE_HOVERING
                    return

            # Keep publishing target setpoint
            self.publish_target_setpoint(
                self.current_target
            )

            distance = self.haversine_m(
                self.current_lat,
                self.current_lon,
                self.current_target.latitude,
                self.current_target.longitude
            )

            if distance <= ARRIVAL_THRESHOLD_METERS:

                self.get_logger().info(
                    f'Arrived at target '
                    f'{self.current_target.target_id} '
                    f'({distance:.1f} m)'
                )

                self.state = STATE_DROPPING

            return

        # ----------------------------------------------
        # DROPPING
        # ----------------------------------------------

        if self.state == STATE_DROPPING:

            self.drop_payload()

            return

        # ----------------------------------------------
        # HOVERING
        # ----------------------------------------------

        if self.state == STATE_HOVERING:

            # If a new target arrives, immediately continue
            if self.remaining_targets:

                self.get_logger().info(
                    'New target received while hovering'
                )

                self.state = STATE_NAVIGATING
                self.pick_next_target()

                return

            # Otherwise remain at 20 ft
            self.publish_hover_setpoint()

    # ======================================================
    # START FLIGHT
    # ======================================================

    def start_flight(self):

        if self.vehicle_state is None:
            return

        if not self.vehicle_state.connected:
            self.get_logger().warn(
                'Waiting for Rudra FCU connection'
            )
            return

        # ARM
        if not self.vehicle_state.armed:

            self.arm_drone()
            return

        # GUIDED
        if self.vehicle_state.mode != 'GUIDED':

            self.set_guided_mode()
            return

        # Already armed + GUIDED
        self.get_logger().info(
            'Rudra armed and in GUIDED mode - '
            'starting delivery'
        )

        self.state = STATE_NAVIGATING

        self.pick_next_target()

    # ======================================================
    # ARM
    # ======================================================

    def arm_drone(self):

        if self.arm_requested:
            return

        if not self.arm_client.wait_for_service(
            timeout_sec=0.2
        ):
            return

        request = CommandBool.Request()
        request.value = True

        self.arm_requested = True

        future = self.arm_client.call_async(request)
        future.add_done_callback(
            self.arm_response
        )

        self.get_logger().info(
            'ARM command sent to Rudra'
        )

    def arm_response(self, future):

        self.arm_requested = False

        try:
            result = future.result()

            if result and result.success:
                self.get_logger().info(
                    'Rudra ARM successful'
                )
            else:
                self.get_logger().warn(
                    'Rudra ARM failed - retrying'
                )

        except Exception as e:
            self.get_logger().warn(
                f'ARM service error: {e}'
            )

    # ======================================================
    # GUIDED MODE
    # ======================================================

    def set_guided_mode(self):

        if self.guided_requested:
            return

        if not self.mode_client.wait_for_service(
            timeout_sec=0.2
        ):
            return

        request = SetMode.Request()
        request.custom_mode = 'GUIDED'

        self.guided_requested = True

        future = self.mode_client.call_async(request)
        future.add_done_callback(
            self.guided_response
        )

        self.get_logger().info(
            'GUIDED mode command sent to Rudra'
        )

    def guided_response(self, future):

        self.guided_requested = False

        try:
            result = future.result()

            if result and result.mode_sent:
                self.get_logger().info(
                    'GUIDED mode accepted'
                )
            else:
                self.get_logger().warn(
                    'GUIDED mode failed - retrying'
                )

        except Exception as e:
            self.get_logger().warn(
                f'GUIDED service error: {e}'
            )

    # ======================================================
    # TARGET SELECTION
    # ======================================================

    def pick_next_target(self):

        if not self.remaining_targets:
            self.current_target = None
            self.state = STATE_HOVERING
            return

        # Choose nearest queued survivor
        self.current_target = min(
            self.remaining_targets,
            key=lambda target:
                self.haversine_m(
                    self.current_lat,
                    self.current_lon,
                    target.latitude,
                    target.longitude
                )
        )

        self.remaining_targets.remove(
            self.current_target
        )

        self.get_logger().info(
            f'Navigating to target '
            f'{self.current_target.target_id}'
        )

    # ======================================================
    # SETPOINT TO TARGET
    # ======================================================

    def publish_target_setpoint(self, target):

        msg = GlobalPositionTarget()

        msg.coordinate_frame = (
            GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        )

        msg.type_mask = TYPE_MASK_POSITION_ONLY

        msg.latitude = target.latitude
        msg.longitude = target.longitude

        # Rudra always flies at 20 ft
        msg.altitude = RUDRA_ALTITUDE

        self.setpoint_pub.publish(msg)

    # ======================================================
    # HOVER
    # ======================================================

    def publish_hover_setpoint(self):

        msg = GlobalPositionTarget()

        msg.coordinate_frame = (
            GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        )

        msg.type_mask = TYPE_MASK_POSITION_ONLY

        # Hold current position
        msg.latitude = self.current_lat
        msg.longitude = self.current_lon

        # Hold 20 ft altitude
        msg.altitude = RUDRA_ALTITUDE

        self.setpoint_pub.publish(msg)

    # ======================================================
    # PAYLOAD DROP
    # ======================================================

    def drop_payload(self):

        if self.current_target is None:
            return

        target_id = self.current_target.target_id

        self.get_logger().info(
            f'Dropping payload for target {target_id}'
        )

        # Send target ID to payload controller
        self.drop_pub.publish(
            UInt8(data=target_id)
        )

        # Delivery completed
        self.current_target = None

        # Immediately continue to next target
        if self.remaining_targets:

            self.get_logger().info(
                'Next target already queued - continuing'
            )

            self.state = STATE_NAVIGATING
            self.pick_next_target()

        else:

            self.get_logger().info(
                'No targets remaining - '
                'Rudra hovering at 20 ft'
            )

            self.state = STATE_HOVERING

    # ======================================================
    # HAVERSINE DISTANCE
    # ======================================================

    @staticmethod
    def haversine_m(
        lat1,
        lon1,
        lat2,
        lon2
    ):

        R = 6371000.0

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)

        dphi = math.radians(
            lat2 - lat1
        )

        dlambda = math.radians(
            lon2 - lon1
        )

        a = (
            math.sin(dphi / 2) ** 2
            +
            math.cos(phi1)
            * math.cos(phi2)
            * math.sin(dlambda / 2) ** 2
        )

        return (
            2
            * R
            * math.asin(math.sqrt(a))
        )


def main(args=None):

    rclpy.init(args=args)

    node = NavDeliveryNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
