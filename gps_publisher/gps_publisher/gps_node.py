import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        try:
            
            self.ser = serial.Serial('/dev/ttyS0', 4800, timeout=1)
            self.get_logger().info("Serial port açıldı")
        except Exception as e:
            self.get_logger().error(f"Serial port açılırken hata: {e}")
            return

        
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)

       
        self.timer = self.create_timer(1.0, self.timer_callback)

    def convert_to_degrees(self, raw_value, direction):
      
        if not raw_value or raw_value == '':
            return 0.0
        try:
            if direction in ['N', 'S']:
                degrees = int(raw_value[0:2])
                minutes = float(raw_value[2:])
            elif direction in ['E', 'W']:
                degrees = int(raw_value[0:3])
                minutes = float(raw_value[3:])
            else:
                return 0.0

            dec = degrees + minutes / 60.0
            if direction == 'S' or direction == 'W':
                dec = -dec
            return dec
        except Exception as e:
            self.get_logger().error(f"Konversiyon hatası: {e}")
            return 0.0

    def timer_callback(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('$GPGGA'): 
                data = line.split(',')

                if len(data) < 10:
                    self.get_logger().warning("Eksik GPS verisi")
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

                self.get_logger().info(
                    f"Published GPS: lat={latitude:.6f}, lon={longitude:.6f}, alt={altitude:.2f}"
                )
        except Exception as e:
            self.get_logger().error(f"GPS okuma hatası: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

