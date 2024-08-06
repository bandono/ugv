import argparse
import board
import busio
import adafruit_pca9685
from adafruit_motor import motor
import time

def main():
    parser = argparse.ArgumentParser(description="")
    parser.add_argument('--ch1', type=int, required=True, help="Channel 1")
    parser.add_argument('--ch2', type=int, required=True, help="Chaanel 2")
    parser.add_argument('--throttle', type=int, required=True, help="Throttle Value")

    args = parser.parse_args()

    i2c = busio.I2C(board.SCL, board.SDA)
    pca = adafruit_pca9685.PCA9685(i2c)

    pca.frequency = 60

    channel1 = pca.channels[args.ch1]
    channel2 = pca.channels[args.ch2]
    motor1 = motor.DCMotor(channel1, channel2)

    motor1.throttle = args.throttle

if __name__ == "__main__":
    main()