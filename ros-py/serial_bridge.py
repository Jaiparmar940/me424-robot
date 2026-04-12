######## DO NOT USE THIS --- USE THE TCP NODE INSTEAD ##########

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # CHANGE THIS to your port
        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 115200

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Connected to {self.serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            self.ser = None

        self.publisher = self.create_publisher(String, 'esp32_data', 10)
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                msg = String()
                msg.data = line
                self.publisher.publish(msg)
                self.get_logger().info(f'Received: {line}')
            except Exception as e:
                self.get_logger().warn(f'Read error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()