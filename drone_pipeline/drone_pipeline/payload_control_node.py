import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8
from mavros_msgs.srv import CommandLong


MAV_CMD_DO_SET_SERVO = 183

PWM_OPEN = 1900
PWM_CLOSED = 1100

# Payload bay -> servo channel
SERVO_CHANNEL_MAP = {
    0: 9,
    1: 9,
    2: 11,
    3: 11,
}


class PayloadControlNode(Node):

    def __init__(self):

        super().__init__('payload_control_node')

        # Barbarik = payload drone
        self.declare_parameter(
            'drone_id',
            'barbarik'
        )

        self.drone_id = (
            self.get_parameter('drone_id')
            .get_parameter_value()
            .string_value
        )

        self.command_client = self.create_client(
            CommandLong,
            f'/{self.drone_id}/cmd/command'
        )

        self.create_subscription(
            UInt8,
            f'/{self.drone_id}/trigger_drop',
            self.trigger_callback,
            10
        )

        self.get_logger().info(
            'Payload control started for Barbarik. '
            'Waiting for drop commands.'
        )

    def trigger_callback(self, msg):

        bay_index = int(msg.data)

        if bay_index not in SERVO_CHANNEL_MAP:

            self.get_logger().error(
                f'Invalid payload bay index: {bay_index}'
            )

            return

        channel = SERVO_CHANNEL_MAP[bay_index]

        self.get_logger().info(
            f'Opening payload bay {bay_index} '
            f'using servo channel {channel}'
        )

        self.set_servo(channel, PWM_OPEN)

    def set_servo(self, channel, pwm):

        if not self.command_client.wait_for_service(
            timeout_sec=2.0
        ):

            self.get_logger().error(
                'MAVROS command service not available'
            )

            return

        request = CommandLong.Request()

        request.command = MAV_CMD_DO_SET_SERVO
        request.param1 = float(channel)
        request.param2 = float(pwm)

        future = self.command_client.call_async(request)

        future.add_done_callback(
            lambda f: self.command_response(
                f,
                channel,
                pwm
            )
        )

    def command_response(
        self,
        future,
        channel,
        pwm
    ):

        try:

            result = future.result()

            if result and result.success:

                self.get_logger().info(
                    f'Servo {channel} set to {pwm} successfully'
                )

            else:

                self.get_logger().warn(
                    f'Failed to set servo {channel} to {pwm}'
                )

        except Exception as e:

            self.get_logger().error(
                f'Servo command failed: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = PayloadControlNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
