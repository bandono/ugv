import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import board
import busio
import adafruit_pca9685
from adafruit_motor import motor

class ControllerSubscriber(Node):
    def __init__(self):
        super().__init__('controller_subscriber')
        
        # Initialize the I2C bus and PCA9685
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(self.i2c)
        self.pca.frequency = 60

        # Initialize motors
        self.channel1 = self.pca.channels[14]
        self.channel2 = self.pca.channels[15]
        self.channel3 = self.pca.channels[12]
        self.channel4 = self.pca.channels[13]
        self.motor1 = motor.DCMotor(self.channel1, self.channel2)
        self.motor2 = motor.DCMotor(self.channel3, self.channel4)

        # Create ROS 2 subscription
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'controller',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        gas_value = msg.data[0]
        brake_value = msg.data[1]

        # Log the received values
        self.get_logger().info(f'RIGHT: {gas_value:.3f}, LEFT: {brake_value:.3f}')

        # Control motors based on received values
        self.motor1.throttle = gas_value
        self.motor2.throttle = brake_value

def main(args=None):
    rclpy.init(args=args)
    
    # Create and run the ControllerSubscriber node
    subscriber = ControllerSubscriber()
    rclpy.spin(subscriber)

    # Destroy the node explicitly
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

