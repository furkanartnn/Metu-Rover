import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge #bridge opencv to ros
from sensor_msgs.msg import Image
import argparse

class CameraNode(Node):
    def __init__(self, ip):
        super().__init__('ip_stream_node')

        self.publisher_ = self.create_publisher(Image, 'ip_stream_image', 1)
        self.timer = self.create_timer(0.001, self.timer_callback) #100 fps
        self.cap = cv2.VideoCapture(f'http://{ip}/video') #write here ip adress

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open video capture')
            rclpy.shutdown()

        self.bridge = CvBridge()  # Initialize the bridge cv2 to ros 

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_NEAREST)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")  # convert img to ros
            self.publisher_.publish(msg)
        else:
            self.get_logger().error('Failed to capture image')

def main(args=None):
    parser = argparse.ArgumentParser(description='ROS2 IP Camera Streamer')
    parser.add_argument('--ip', type=str, required=True, help='IP address and port of the IP camera (e.g., 192.168.0.180:8080)')
    cli_args = parser.parse_args()

    rclpy.init(args=args)
    node = CameraNode(cli_args.ip)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
