import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import board
import busio
import adafruit_pca9685
from adafruit_motor import motor

class TwistListener(Node):
    def __init__(self):
        super().__init__('twist_listener')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.callback,
            10)
        self.get_logger().info('Twist Listener started')

        # Initialize I2C and PCA9685
        i2c = busio.I2C(board.SCL, board.SDA)
        pca = adafruit_pca9685.PCA9685(i2c)
        pca.frequency = 60

        # Initialize motors
        self.channel1 = pca.channels[12]
        self.channel2 = pca.channels[13]
        self.motor1 = motor.DCMotor(self.channel1, self.channel2)

        self.channel3 = pca.channels[14]
        self.channel4 = pca.channels[15]
        self.motor2 = motor.DCMotor(self.channel3, self.channel4)

        # Stop the motors initially
        self.motor1.throttle = 0.0
        self.motor2.throttle = 0.0

    def callback(self, msg):
        self.get_logger().info('Received Data: %s' % msg)
        # Extract linear velocity in the x direction
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate throttle values based on linear_x and angular_z
        throttle1 = linear_x - angular_z
        throttle2 = linear_x + angular_z

        # Clip throttle values to be within the range [-1, 1]
        if throttle1 > 1.0:
            self.get_logger().warn(f'Throttle1 value {throttle1} exceeds 1.0, clipping to 1.0')
            throttle1 = 1.0
        elif throttle1 < -1.0:
            self.get_logger().warn(f'Throttle1 value {throttle1} is below -1.0, clipping to -1.0')
            throttle1 = -1.0

        if throttle2 > 1.0:
            self.get_logger().warn(f'Throttle2 value {throttle2} exceeds 1.0, clipping to 1.0')
            throttle2 = 1.0
        elif throttle2 < -1.0:
            self.get_logger().warn(f'Throttle2 value {throttle2} is below -1.0, clipping to -1.0')
            throttle2 = -1.0

        # Set motor throttle based on the calculated throttle values
        self.get_logger().info(f'Setting motor throttles: motor1 = {throttle1}, motor2 = {throttle2}')
        self.motor1.throttle = throttle1
        self.motor2.throttle = throttle2

def main(args=None):
    rclpy.init(args=args)
    node = TwistListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

