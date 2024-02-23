import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class KeyboardTeleopSubscriber(Node):

    def __init__(self):
        super().__init__('keyboard_teleop_service')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print(msg)


def main(args=None):
    rclpy.init(args=args)

    keyboard_teleop_service = KeyboardTeleopSubscriber()

    rclpy.spin(keyboard_teleop_service)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keyboard_teleop_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()