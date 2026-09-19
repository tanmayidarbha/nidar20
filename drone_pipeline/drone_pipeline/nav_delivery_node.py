import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt8, Bool

from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import SetMode, CommandBool

from drone_msgs.msg import TargetList


BARBARIK_ALTITUDE = 6.096       # 20 ft in meters
ARRIVAL_THRESHOLD_METERS = 3.0
MAX_PAYLOADS = 4

# Position only:
# Ignore velocity, acceleration/force, yaw and yaw-rate.
POSITION_ONLY_TYPE_MASK = 3576


class NavDeliveryNode(Node):

    def __init__(self):
        super().__init__('nav_delivery_node')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------
        self.declare_parameter('drone_id', 'barbarik')

        self.drone_id = (
            self.get_parameter('drone_id')
            .get_parameter_value()
            .string_value
        )

        self.mavros_prefix = f'/{self.drone_id}/mavros'

        # ---------------------------------------------------------
        # State
        # ---------------------------------------------------------
        self.vehicle_state = State()

        self.current_lat = None
        self.current_lon = None
        self.current_alt = None

        self.target_queue = []

        # Payload bay index:
        # 0 -> first payload
        # 1 -> second payload
        # 2 -> third payload
        # 3 -> fourth payload
        self.next_bay_index = 0

        self.current_target = None

        self.scan_complete = False

        # Barbarik should only RTL after it has actually started flying.
        self.mission_started = False
        self.rtl_requested = False

        self.state = 'WAITING'

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------

        self.target_sub = self.create_subscription(
            TargetList,
            f'/{self.drone_id}/target_list',
            self.target_callback,
            10
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            f'{self.mavros_prefix}/global_position/global',
            self.gps_callback,
            10
        )

        self.state_sub = self.create_subscription(
            State,
            f'{self.mavros_prefix}/state',
            self.state_callback,
            10
        )

        self.scan_complete_sub = self.create_subscription(
            Bool,
            f'/{self.drone_id}/scan_complete',
            self.scan_complete_callback,
            10
        )

        # ---------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------

        self.setpoint_pub = self.create_publisher(
            GlobalPositionTarget,
            f'{self.mavros_prefix}/setpoint_position/global',
            10
        )

        self.drop_pub = self.create_publisher(
            UInt8,
            f'/{self.drone_id}/trigger_drop',
            10
        )

        # ---------------------------------------------------------
        # Services
        # ---------------------------------------------------------

        self.arming_client = self.create_client(
            CommandBool,
            f'{self.mavros_prefix}/cmd/arming'
        )

        self.set_mode_client = self.create_client(
            SetMode,
            f'{self.mavros_prefix}/set_mode'
        )

        # ---------------------------------------------------------
        # Main loop
        # ---------------------------------------------------------

        self.timer = self.create_timer(
            0.5,
            self.control_loop
        )

        self.get_logger().info(
            f'Barbarik navigation node started for "{self.drone_id}"'
        )

    # =============================================================
    # CALLBACKS
    # =============================================================

    def state_callback(self, msg):
        self.vehicle_state = msg

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

        if not math.isnan(msg.altitude):
            self.current_alt = msg.altitude

    def scan_complete_callback(self, msg):
        if msg.data and not self.scan_complete:
            self.scan_complete = True

            self.get_logger().info(
                'Rudra scan complete received.'
            )

    def target_callback(self, msg):
        """
        Receive targets from Rudra.

        Target IDs are used to prevent duplicate processing.
        """

        for target in msg.targets:

            # Ignore a target that was already processed.
            already_known = False

            if self.current_target is not None:
                if target.target_id == self.current_target.target_id:
                    already_known = True

            for queued_target in self.target_queue:
                if target.target_id == queued_target.target_id:
                    already_known = True
                    break

            if already_known:
                continue

            # Maximum four payloads.
            if self.next_bay_index + len(self.target_queue) >= MAX_PAYLOADS:
                self.get_logger().warn(
                    'Maximum payload capacity reached. '
                    'Ignoring additional target.'
                )
                continue

            self.target_queue.append(target)

            self.get_logger().info(
                f'New target received: '
                f'ID={target.target_id}, '
                f'Lat={target.latitude:.7f}, '
                f'Lon={target.longitude:.7f}'
            )

    # =============================================================
    # MAIN CONTROL LOOP
    # =============================================================

    def control_loop(self):

        if self.rtl_requested:
            return

        # ---------------------------------------------------------
        # WAITING
        # ---------------------------------------------------------

        if self.state == 'WAITING':

            if len(self.target_queue) > 0:

                self.start_flight()

            # If there are no targets and Rudra finishes scanning,
            # Barbarik simply stays on the ground.
            return

        # ---------------------------------------------------------
        # NAVIGATING
        # ---------------------------------------------------------

        elif self.state == 'NAVIGATING':

            if self.current_target is None:
                self.state = 'WAITING'
                return

            if self.current_lat is None or self.current_lon is None:
                return

            distance = self.calculate_distance(
                self.current_lat,
                self.current_lon,
                self.current_target.latitude,
                self.current_target.longitude
            )

            self.publish_target_setpoint(
                self.current_target.latitude,
                self.current_target.longitude,
                BARBARIK_ALTITUDE
            )

            self.get_logger().debug(
                f'Distance to target: {distance:.2f} m'
            )

            if distance <= ARRIVAL_THRESHOLD_METERS:

                self.get_logger().info(
                    f'Arrived at target {self.current_target.target_id}'
                )

                self.state = 'DROPPING'

        # ---------------------------------------------------------
        # DROPPING
        # ---------------------------------------------------------

        elif self.state == 'DROPPING':

            self.drop_payload()

        # ---------------------------------------------------------
        # HOVERING
        # ---------------------------------------------------------

        elif self.state == 'HOVERING':

            if self.current_target is not None:

                self.publish_target_setpoint(
                    self.current_target.latitude,
                    self.current_target.longitude,
                    BARBARIK_ALTITUDE
                )

            # More targets arrived while Barbarik was flying.
            if len(self.target_queue) > 0:

                self.pick_next_target()

            # Scan finished and all targets have been delivered.
            elif self.scan_complete and self.mission_started:

                self.return_to_launch()

    # =============================================================
    # FLIGHT START
    # =============================================================

    def start_flight(self):

        if self.next_bay_index >= MAX_PAYLOADS:
            self.get_logger().warn(
                'All four payload bays have already been used.'
            )
            return

        self.pick_next_target()

        if self.current_target is None:
            return

        self.mission_started = True

        self.get_logger().info(
            'Starting Barbarik delivery mission.'
        )

        # ---------------------------------------------------------
        # ARM
        # ---------------------------------------------------------

        if not self.vehicle_state.armed:

            if not self.arming_client.wait_for_service(
                timeout_sec=1.0
            ):
                self.get_logger().warn(
                    'Arming service not available.'
                )
                return

            request = CommandBool.Request()
            request.value = True

            future = self.arming_client.call_async(request)

            future.add_done_callback(
                self.arm_response_callback
            )

            return

        # ---------------------------------------------------------
        # GUIDED
        # ---------------------------------------------------------

        if self.vehicle_state.mode != 'GUIDED':

            if not self.set_mode_client.wait_for_service(
                timeout_sec=1.0
            ):
                self.get_logger().warn(
                    'Set mode service not available.'
                )
                return

            request = SetMode.Request()
            request.custom_mode = 'GUIDED'

            future = self.set_mode_client.call_async(request)

            future.add_done_callback(
                self.guided_response_callback
            )

            return

        # ---------------------------------------------------------
        # Already armed + GUIDED
        # ---------------------------------------------------------

        self.state = 'NAVIGATING'

        self.get_logger().info(
            f'Navigating to target '
            f'{self.current_target.target_id}'
        )

    # =============================================================
    # ARM RESPONSE
    # =============================================================

    def arm_response_callback(self, future):

        try:
            response = future.result()

            if response.success:

                self.get_logger().info(
                    'Barbarik armed.'
                )

                # GUIDED will be requested on the next loop.
                return

            self.get_logger().error(
                'Barbarik arming failed.'
            )

        except Exception as e:

            self.get_logger().error(
                f'Arming service error: {e}'
            )

    # =============================================================
    # GUIDED RESPONSE
    # =============================================================

    def guided_response_callback(self, future):

        try:
            response = future.result()

            if response.mode_sent:

                self.get_logger().info(
                    'GUIDED mode command sent.'
                )

            else:

                self.get_logger().error(
                    'Failed to send GUIDED mode command.'
                )

        except Exception as e:

            self.get_logger().error(
                f'GUIDED mode service error: {e}'
            )

    # =============================================================
    # PICK NEXT TARGET
    # =============================================================

    def pick_next_target(self):

        if len(self.target_queue) == 0:

            self.current_target = None

            if self.scan_complete and self.mission_started:
                self.return_to_launch()
            else:
                self.state = 'HOVERING'

            return

        if self.next_bay_index >= MAX_PAYLOADS:

            self.get_logger().warn(
                'No payload bays remaining.'
            )

            self.target_queue.clear()
            self.current_target = None

            if self.scan_complete and self.mission_started:
                self.return_to_launch()
            else:
                self.state = 'HOVERING'

            return

        self.current_target = self.target_queue.pop(0)

        self.get_logger().info(
            f'Selected target '
            f'{self.current_target.target_id} '
            f'for payload bay {self.next_bay_index}'
        )

        self.state = 'NAVIGATING'

    # =============================================================
    # DROP PAYLOAD
    # =============================================================

    def drop_payload(self):

        if self.current_target is None:
            self.state = 'WAITING'
            return

        if self.next_bay_index >= MAX_PAYLOADS:

            self.get_logger().error(
                'Payload limit reached. No more drops allowed.'
            )

            self.current_target = None

            if self.scan_complete:
                self.return_to_launch()
            else:
                self.state = 'HOVERING'

            return

        bay_index = self.next_bay_index

        self.get_logger().info(
            f'Dropping payload for target '
            f'{self.current_target.target_id} '
            f'using bay {bay_index}'
        )

        msg = UInt8()
        msg.data = bay_index

        self.drop_pub.publish(msg)

        # Move to next payload bay.
        self.next_bay_index += 1

        # Current target completed.
        self.current_target = None

        # ---------------------------------------------------------
        # More targets already queued
        # ---------------------------------------------------------

        if len(self.target_queue) > 0:

            self.pick_next_target()

        # ---------------------------------------------------------
        # No targets left
        # ---------------------------------------------------------

        else:

            if self.scan_complete:

                self.get_logger().info(
                    'All detected targets delivered and '
                    'Rudra scan is complete. RTL.'
                )

                self.return_to_launch()

            else:

                self.get_logger().info(
                    'Waiting for additional targets from Rudra.'
                )

                self.state = 'HOVERING'

    # =============================================================
    # RETURN TO LAUNCH
    # =============================================================

    def return_to_launch(self):

        if self.rtl_requested:
            return

        if not self.mission_started:
            return

        self.rtl_requested = True

        self.get_logger().info(
            'Commanding Barbarik to RTL.'
        )

        if not self.set_mode_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().error(
                'Set mode service unavailable. '
                'Cannot command RTL.'
            )
            self.rtl_requested = False
            return

        request = SetMode.Request()
        request.custom_mode = 'RTL'

        future = self.set_mode_client.call_async(request)

        future.add_done_callback(
            self.rtl_response_callback
        )

    def rtl_response_callback(self, future):

        try:

            response = future.result()

            if response.mode_sent:

                self.get_logger().info(
                    'RTL command sent to Barbarik.'
                )

            else:

                self.get_logger().error(
                    'Failed to send RTL command.'
                )

                self.rtl_requested = False

        except Exception as e:

            self.get_logger().error(
                f'RTL service error: {e}'
            )

            self.rtl_requested = False

    # =============================================================
    # SETPOINT
    # =============================================================

    def publish_target_setpoint(
        self,
        latitude,
        longitude,
        altitude
    ):

        msg = GlobalPositionTarget()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.coordinate_frame = (
            GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        )

        msg.type_mask = POSITION_ONLY_TYPE_MASK

        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude

        self.setpoint_pub.publish(msg)

    # =============================================================
    # DISTANCE CALCULATION
    # =============================================================

    @staticmethod
    def calculate_distance(
        lat1,
        lon1,
        lat2,
        lon2
    ):

        R = 6371000.0

        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)

        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)

        a = (
            math.sin(dlat / 2) ** 2
            +
            math.cos(lat1_rad)
            * math.cos(lat2_rad)
            * math.sin(dlon / 2) ** 2
        )

        c = 2 * math.atan2(
            math.sqrt(a),
            math.sqrt(1 - a)
        )

        return R * c


def main(args=None):

    rclpy.init(args=args)

    node = NavDeliveryNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
