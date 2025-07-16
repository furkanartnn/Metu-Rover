import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from flask import Flask, Response
import threading # work for two diff. thread

app = Flask(__name__)
latest_frame = None

class ImageNode(Node):
	def __init__(self):
		super().__init__('img_sub_node')
		self.subscription = self.create_subscription(Image, 'webcam_image', self.listener_callback, 10)
		self.bridge = CvBridge()
	def listener_callback(self,msg):
		global latest_frame
		cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = "bgr8")
		ret, jpeg = cv2.imencode('.jpg', cv_image)
		if ret:
			latest_frame = jpeg.tobytes()


@app.route('/video_feed')
def video_feed():
    def generate():
        global latest_frame
        while True:
            if latest_frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


def ros_thread():
    rclpy.init()
    node = ImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main():
	threading.Thread(target = ros_thread, daemon=True).start() #init on different thread on same time 
	app.run(host='0.0.0.0', port = 5000, debug =False)

if __name__ == '__main__':
	main()
