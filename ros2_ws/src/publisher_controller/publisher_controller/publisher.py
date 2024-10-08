import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import os, struct, array
from fcntl import ioctl

# Iterate over the joystick devices
print('Available devices:')

for fn in os.listdir('/dev/input'):
    if fn.startswith('js'):
        print('  /dev/input/%s' % (fn))

# These constants were borrowed from linux/input.h
axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'throttle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}

button_names = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}

class ControllerPublisher(Node):
    def __init__(self):
        super().__init__('controller_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'controller', 10)

        # To store the states
        self.axis_states = {}
        self.button_states = {}
        self.axis_map = []
        self.button_map = []
        
        # Initialize the state variables
        self.rightForward = True
        self.leftForward = True

        # Open the controller.
        fn = '/dev/input/js0'
        print('Opening %s...' % fn)
        self.jsdev = open(fn, 'rb')

        # Print the controller type name
        buf = array.array('B', [0] * 64)
        ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
        js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
        print('Device name: %s' % js_name)

        # Number of axes and buttons
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a11, buf) # JSIOCGAXES
        num_axes = buf[0]

        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
        num_buttons = buf[0]

        # Axis map
        buf = array.array('B', [0] * 0x40)
        ioctl(self.jsdev, 0x80406a32, buf) # JSIOCGAXMAP

        for axis in buf[:num_axes]:
            axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(axis_name)
            self.axis_states[axis_name] = 0.0

        # Button map.
        buf = array.array('H', [0] * 200)
        ioctl(self.jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

        for btn in buf[:num_buttons]:
            btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0

        print('%d axes found: %s' % (num_axes, ', '.join(self.axis_map)))
        print('%d buttons found: %s' % (num_buttons, ', '.join(self.button_map)))

        while True:
            self.timer_callback()

    # Read 8 bytes from the controller
    def timer_callback(self):
        evbuf = self.jsdev.read(8)

        if evbuf:
            time, value, types, number = struct.unpack('IhBB', evbuf)

            if types & 0x80:
                print("(initial)", end="")
            # Check if the event is a button event
            if types & 0x01:
                button = self.button_map[number]
                if button:
                    self.button_states[button] = value
                    if value:
                        if button == 'tr':
                            self.rightForward = False
                        elif button == 'tl':
                            self.leftForward = False
                        print("%s pressed" % (button))
                    else:
                        if button == 'tr':
                            self.rightForward = True
                        elif button == 'tl':
                            self.leftForward = True
                        print("%s released" % (button))

            # Check if the event is an axis event
            if types & 0x02:
                axis = self.axis_map[number]
                if axis:
                    fvalue = value / 32767.0
                    fvalue = (fvalue + 1) / 2
                    if self.leftForward == False and axis == "brake":
                        fvalue = -1 * fvalue
                    if self.rightForward == False and axis == "gas":
                        fvalue = -1 * fvalue

                    self.axis_states[axis] = fvalue
                    print("%s: %.3f" % (axis, fvalue))

        # Prepare the message with the current state of all relevant axes
        msg = Float32MultiArray()
        msg.data = [self.axis_states.get('gas', 0), self.axis_states.get('brake', 0)]

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    publisher = ControllerPublisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
