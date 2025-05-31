import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from gps3 import gps3


class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)

        self.gps_socket = gps3.GPSDSocket()
        self.data_stream = gps3.DataStream()

        try:
            # Connect to gpsd via UNIX socket (preferred) or TCP
            # self.gps_socket.connect('/var/run/gpsd.sock')  # host must mount this into Docker
            self.gps_socket.connect(host='127.0.0.1', port=2947)  # alternative
            self.gps_socket.watch()
            self.get_logger().info("Connected to gpsd")
        except Exception as e:
            self.get_logger().error(f"Could not connect to gpsd: {e}")
            raise

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            for new_data in self.gps_socket:
                if new_data:
                    self.data_stream.unpack(new_data)

                    lat = self.data_stream.TPV.get('lat', 'n/a')
                    lon = self.data_stream.TPV.get('lon', 'n/a')
                    alt = self.data_stream.TPV.get('alt', 'n/a')

                    if lat != 'n/a' and lon != 'n/a':
                        msg = NavSatFix()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = 'gps_link'

                        msg.status.status = NavSatStatus.STATUS_FIX
                        msg.status.service = NavSatStatus.SERVICE_GPS

                        msg.latitude = float(lat)
                        msg.longitude = float(lon)
                        msg.altitude = float(alt) if alt != 'n/a' else 0.0

                        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                        self.publisher_.publish(msg)
                        self.get_logger().debug(f"Published GPS: {msg.latitude}, {msg.longitude}")
                        break  # only publish one valid fix per timer callback
        except Exception as e:
            self.get_logger().error(f"GPS read error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

