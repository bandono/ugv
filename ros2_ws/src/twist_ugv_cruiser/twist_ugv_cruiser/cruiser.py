import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import threading

#import board
#import busio
#import adafruit_pca9685
#from adafruit_motor import motor

# Constants
SERIAL_PORT = '/dev/ttyACM1'  # Modify this to your serial port
BAUD_RATE = 9600  # Baud rate for serial communication
STOP_DELAY = 0.5  # Delay in seconds before sending the stop command
LOW_SPEED = 0.14  # Minimum speed
MAX_SPEED = 1.7   # Maximum speed

class UGVCruiser(Node):
    def __init__(self):
        super().__init__('twist_ugv_cruiser')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.callback,
            10)
        self.get_logger().info('Twist UGV Cruiser started')

        # Set up serial communication
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        self.moving_forward = False  # Track if moving forward
        self.current_speed = 0.5  # Start with 0.5 as initial speed
        self.stop_timer = None  # Timer for stopping the robot

    def callback(self, msg):
        self.get_logger().info('Received Data: %s' % msg)

        # Extract linear velocity in the x direction
        linear_x = msg.linear.x

        # First interpretation: move forward or stop
        if linear_x > 0:  # Forward motion
            self.move_forward()
        elif linear_x == 0:  # Stop condition (message explicitly tells to stop)
            self.stop_robot()

        # Handle speed interpretation
        self.handle_speed(linear_x)

    def move_forward(self):
        if not self.moving_forward:
            # Send "0x49" to serial to move forward
            self.ser.write(b'\x49')
            self.get_logger().info('Sending move command (0x49)')
            self.moving_forward = True

        # Reset the stop timer every time we receive a forward command
        if self.stop_timer is not None:
            self.stop_timer.cancel()

        # Start the stop timer to send "0x6B" after STOP_DELAY
        self.stop_timer = threading.Timer(STOP_DELAY, self.stop_robot)
        self.stop_timer.start()

    def stop_robot(self):
        # Send "0x6B" to serial to stop
        self.ser.write(b'\x6B')
        self.get_logger().info('Sending stop command (0x6B)')
        self.moving_forward = False

    def handle_speed(self, linear_x):
        # Only proceed if linear_x is within the defined range
        if LOW_SPEED < linear_x < MAX_SPEED:
            # If the new speed is greater than the current speed, increase it
            if linear_x > self.current_speed:
                self.current_speed = linear_x
                
                # Log the new speed
                self.get_logger().info(f'Speed increased to: {self.current_speed}')
                
                # Send serial command to increase speed
                self.ser.write(b'\x51')  # Example: 0x51 for speed increase
                self.get_logger().info(f'Sending increase speed command (0x51)')

            # If the new speed is less than the current speed, decrease it
            elif linear_x < self.current_speed:
                self.current_speed = linear_x

                # Log the new speed
                self.get_logger().info(f'Speed decreased to: {self.current_speed}')
                
                # Send serial command to decrease speed
                self.ser.write(b'\x41')  # Example: 0x41 for speed decrease
                self.get_logger().info(f'Sending decrease speed command (0x41)')



def main(args=None):
    rclpy.init(args=args)
    node = UGVCruiser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
