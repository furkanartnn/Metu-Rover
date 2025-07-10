# gps_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.ser = serial.Serial('/dev/ttyS0', 4800, timeout=1)
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def convert_to_degrees(self, raw_value, direction):
        if not raw_value:
            return 0.0
        try:
            if direction in ['N', 'S']:
                degrees = int(raw_value[0:2])
                minutes = float(raw_value[2:])
            else:  # E or W
                degrees = int(raw_value[0:3])
                minutes = float(raw_value[3:])
            decimal = degrees + minutes / 60.0
            return -decimal if direction in ['S', 'W'] else decimal
        except:
            return 0.0

    def timer_callback(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith('$GPGGA'):
            data = line.split(',')
            if len(data) < 10:
                return
            latitude = self.convert_to_degrees(data[2], data[3])
            longitude = self.convert_to_degrees(data[4], data[5])
            altitude = float(data[9]) if data[9] else 0.0

            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps_link"
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
            msg.latitude = latitude
            msg.longitude = longitude
            msg.altitude = altitude
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            self.publisher_.publish(msg)
            self.get_logger().info(f"Published GPS: lat={latitude}, lon={longitude}, alt={altitude}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
