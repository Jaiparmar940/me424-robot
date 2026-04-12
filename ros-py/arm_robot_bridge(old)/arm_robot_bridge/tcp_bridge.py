import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

# PI Username: "ramses-user"
# PI Password: "ramses-pw"

# SSID: "surgical_clanker2.4"
# PW: "bone_saw"
# ESP 1 IP: 172.28.205.100 (as of 22:08 4/11/26)
# ESP 2 IP - Main Board: 172.28.205.101 (as of 22:08 4/11/26)
# ESP 3 IP: 172.28.205.102 (as of 22:08 4/11/26)
# PI IP: 172.28.205.104 (as of 22:08 4/11/26)

class TCPBridge(Node):
    def __init__(self):
        super().__init__('tcp_bridge')

        # 🔧 CHANGE THIS
        self.host = '172.28.205.101'   # ESP32 IP
        self.port = 3333

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            self.sock.connect((self.host, self.port))
            self.get_logger().info(f'Connected to ESP32 at {self.host}:{self.port}')
        except Exception as e:
            self.get_logger().error(f'Connection failed: {e}')
            return

        self.sock.setblocking(False)

        # ROS topics
        self.pub = self.create_publisher(String, 'esp32_feedback', 10)
        self.sub = self.create_subscription(String, 'esp32_cmd', self.send_command, 10)

        self.timer = self.create_timer(0.05, self.read_socket)

    def send_command(self, msg):
        try:
            cmd = msg.data.strip() + "\n"
            self.sock.send(cmd.encode())
            self.get_logger().info(f'Sent: {cmd.strip()}')
        except Exception as e:
            self.get_logger().error(f'Send failed: {e}')

    def read_socket(self):
        try:
            data = self.sock.recv(1024)
            if data:
                lines = data.decode().split('\n')
                for line in lines:
                    line = line.strip()
                    if line:
                        msg = String()
                        msg.data = line
                        self.pub.publish(msg)
                        self.get_logger().info(f'Received: {line}')
        except BlockingIOError:
            pass
        except Exception as e:
            self.get_logger().warn(f'Read error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TCPBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()