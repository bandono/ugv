#!/usr/bin/python3
import board
import busio
import adafruit_pca9685
from adafruit_motor import motor
import time

import rclpy
from rclpy.node import Node

from ugv.msg import PWM


class PWM_Subsciber(Node):

    def __init__(self):
        super().__init__('pwm')
        self.subscription = self.create_subscription(
            PWM,
            'respati/ugv/pwm',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(i2c)

        self.pca.frequency = 60

    def listener_callback(self, msg):
        channel1 = self.pca.channels[msg.ch1]
        channel2 = self.pca.channels[msg.ch2]
        motor1 = motor.DCMotor(channel1, channel2)

        motor1.throttle = msg.throttle


def main(args=None):
    rclpy.init(args=args)

    subscriber = PWM_Subsciber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()