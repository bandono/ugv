import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Declare parameters with default values
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)

        # Retrieve parameters
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        fps = self.get_parameter('fps').get_parameter_value().integer_value

        # Create a publisher for compressed images
        self.publisher_ = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', 10)
        
        # Create a timer with the interval based on the FPS parameter
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        
        # Open the camera and set resolution
        self.cap = cv2.VideoCapture('/dev/video2')  # Change to your camera index or URL
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture image')
            return

        # Compress the image
        compressed_msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')  # Adjust quality as needed
        
        # Publish the compressed image message
        self.publisher_.publish(compressed_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

