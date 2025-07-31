import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import logging

# Set up logger for Arduino commands
arduino_logger = logging.getLogger("arduino_commands")
arduino_logger.setLevel(logging.INFO)
file_handler = logging.FileHandler("arduino_commands.log")
formatter = logging.Formatter('%(asctime)s - %(message)s')
file_handler.setFormatter(formatter)
arduino_logger.addHandler(file_handler)

class ArduinoWheelsWriterNode(Node):
    def __init__(self):
        super().__init__('arduino_wheels_writer_node')
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1) # Adjust the port and baudrate as needed
        self.get_logger().info('Serial writer connected to Arduino.')
        self.subscription = self.create_subscription(
            String,
            'wheels_command_topic',
            self.command_callback,
            10
        )

    def command_callback(self, msg):
        # Log the command before sending
        arduino_logger.info(f"SENT TO ARDUINO: {msg.data.strip()}")
        self.ser.write(msg.data.encode())
        self.get_logger().info(f'Sent to Arduino: {msg.data.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoWheelsWriterNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()