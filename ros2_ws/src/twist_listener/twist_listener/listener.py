import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistListener(Node):
    def __init__(self):
        super().__init__('twist_listener')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.callback,
            10)
        self.get_logger().info('Twist Listener started')

    def callback(self, msg):
        self.get_logger().info('Received Data')
        self.get_logger().info(f'Sending {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = TwistListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

