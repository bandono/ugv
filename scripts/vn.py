#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import serial

class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader')
        # Configure the serial connection (adjust parameters as needed)
        self.ser = serial.Serial(
            port='/dev/ttyTHS1',  # Change this to your serial port
            baudrate=115200,      # Check VN-100 documentation for baud rate
            timeout=1
        )
        self.timer = self.create_timer(0.1, self.read_serial_data)  # Read every second

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            try:
                data = self.ser.read(self.ser.in_waiting).decode('utf-8')
                data = data.split(',')
                if data[0] == "$VNYMR":
                    print(f'[{data[1]}, {data[2]}, {data[3]}]')
                else:
                    pass
            except:
                pass
            

def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
